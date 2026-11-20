/* Real motor I/O for a DRV8871 H-bridge + quadrature-encoder gearmotor.
 *
 * TIM2 (32-bit) counts encoder edges in hardware (x4 decoding), so the CPU
 * never misses a count no matter how busy it is.
 * TIM1 makes 20 kHz center-aligned PWM on CH1/CH2 (IN1/IN2 of the DRV8871).
 * ADC1 samples the DRV8871's current-sense output, triggered by TIM1 CH4 in the
 * middle of the PWM on-time, where the current ripple crosses its average. */
#include "board.h"
#include "motor_io.h"

#define PWM_HZ        20000u
/* Center-aligned: the counter goes up to ARR and back down, so f = clk / (2 * ARR). */
#define PWM_ARR       (BOARD_TIMCLK2_HZ / (2u * PWM_HZ)) /* 4500 */
#define V_SUPPLY      24.0f

/* DRV8871 current sense: I_IPROPI = I_motor * 1100 uA/A into R_IPROPI.
 * With R_IPROPI = 1.5 kOhm: 1.65 V/A, so 3.3 V full scale = 2 A. Pick R for your motor. */
#define ADC_VREF      3.3f
#define ADC_COUNTS    4096.0f
#define IPROPI_GAIN   1100e-6f
#define R_IPROPI      1500.0f

static float last_sign = 1.0f;

static void encoder_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    /* PA0/PA1 -> AF1 (TIM2_CH1/CH2), pull-ups for open-collector hall encoders. */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1)) |
                   (2u << GPIO_MODER_MODER0_Pos) | (2u << GPIO_MODER_MODER1_Pos);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1)) |
                   (1u << GPIO_PUPDR_PUPD0_Pos) | (1u << GPIO_PUPDR_PUPD1_Pos);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1)) |
                    (1u << GPIO_AFRL_AFSEL0_Pos) | (1u << GPIO_AFRL_AFSEL1_Pos);

    TIM2->ARR = 0xFFFFFFFFu;
    /* CC1S/CC2S = 01: TI1/TI2 as inputs. IC1F/IC2F = 0011: digital filter
     * (8 samples) to reject edge glitches. */
    TIM2->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos) |
                  (3u << TIM_CCMR1_IC1F_Pos) | (3u << TIM_CCMR1_IC2F_Pos);
    TIM2->CCER = 0; /* non-inverted */
    TIM2->SMCR = 3u << TIM_SMCR_SMS_Pos; /* encoder mode 3: count on both edges of both channels */
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;
}

static void pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;
    /* PA8/PA9 -> AF1 (TIM1_CH1/CH2). */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) |
                   (2u << GPIO_MODER_MODER8_Pos) | (2u << GPIO_MODER_MODER9_Pos);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9)) |
                    (1u << GPIO_AFRH_AFSEL8_Pos) | (1u << GPIO_AFRH_AFSEL9_Pos);

    TIM1->PSC = 0;
    TIM1->ARR = PWM_ARR;
    /* PWM mode 1 with preload on CH1, CH2 (outputs) and CH4 (ADC trigger only). */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    /* In center-aligned mode 1, compare flags fire only while counting down,
     * so CCR4 = 1 trips just before the counter valley = middle of the
     * on-pulse (PWM mode 1 is high while CNT < CCR). */
    TIM1->CCR4 = 1;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC4E;
    TIM1->BDTR = TIM_BDTR_MOE; /* advanced timers keep outputs off until Main Output Enable */
    TIM1->CR1 = TIM_CR1_ARPE | (1u << TIM_CR1_CMS_Pos);
    TIM1->EGR = TIM_EGR_UG; /* load the preloaded registers */
    TIM1->CR1 |= TIM_CR1_CEN;
}

static void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;
    GPIOA->MODER |= GPIO_MODER_MODER4; /* PA4 analog */

    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (1u << ADC_CCR_ADCPRE_Pos); /* 90 MHz / 4 = 22.5 MHz (max 36) */
    /* One injected conversion of channel 4. With JL = 0 the sequencer uses JSQ4. */
    ADC1->JSQR = 4u << ADC_JSQR_JSQ4_Pos;
    ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP4) | (4u << ADC_SMPR2_SMP4_Pos); /* 84 cycles: IPROPI is high-impedance */
    /* JEXTSEL = 0000 (TIM1_CC4), JEXTEN = 01 (rising edge): hardware-triggered, no CPU involvement. */
    ADC1->CR2 = ADC_CR2_ADON | (1u << ADC_CR2_JEXTEN_Pos);
}

int motor_io_init(void)
{
    encoder_init();
    adc_init();
    pwm_init();
    return 0;
}

int32_t motor_io_encoder(void)
{
    return (int32_t)TIM2->CNT; /* wraps cleanly; the controller differences counts */
}

float motor_io_current(void)
{
    /* JDR1 always holds the latest sample. IPROPI is unsigned, so the sign
     * comes from the drive direction. */
    const float volts = (float)(ADC1->JDR1 & 0xFFFu) * (ADC_VREF / ADC_COUNTS);
    return last_sign * volts / (IPROPI_GAIN * R_IPROPI);
}

void motor_io_set_voltage(float volts)
{
    float duty = volts / V_SUPPLY;
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    const uint32_t ccr = (uint32_t)((duty < 0.0f ? -duty : duty) * (float)PWM_ARR);
    /* Sign-magnitude drive: PWM one input, hold the other low (coast during
     * the off-time, i.e. fast decay). The preload registers mean the new duty
     * takes effect at the next PWM period, never mid-pulse. */
    if (duty >= 0.0f) {
        TIM1->CCR2 = 0;
        TIM1->CCR1 = ccr;
        last_sign = 1.0f;
    } else {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = ccr;
        last_sign = -1.0f;
    }
}

void motor_io_on_frame(const can_frame_t *f)
{
    (void)f;
}

uint8_t motor_io_extra_can_ids(const uint16_t **ids)
{
    *ids = 0;
    return 0;
}
