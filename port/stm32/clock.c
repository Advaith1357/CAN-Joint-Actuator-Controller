/* Clock tree: HSI (16 MHz) -> PLL -> 180 MHz.
 *
 * Order matters (RM0390, PWR and RCC chapters):
 *  1. Voltage scale 1 and over-drive. 180 MHz needs both; without them the
 *     core is only rated to 168 MHz.
 *  2. Flash wait states BEFORE raising the clock. At 180 MHz / 3.3 V flash
 *     needs 5 wait states; running faster than flash can deliver = garbage
 *     instructions.
 *  3. Bus prescalers, so APB1/APB2 never exceed 45/90 MHz, even momentarily.
 *  4. Switch SYSCLK to the PLL and wait for the switch to be confirmed. */
#include "clock.h"

#include "board.h"

/* PLL: VCO_in = 16 MHz / M = 2 MHz (ST recommends 2 MHz for lowest jitter),
 *      VCO_out = 2 MHz * N = 360 MHz, SYSCLK = VCO_out / P = 180 MHz,
 *      Q = 360 / 8 = 45 MHz (the 48 MHz USB clock isn't used here),
 *      R = 2 (F446-specific I2S/SAI output; 0 and 1 are invalid, reset value is 2).
 * Writing PLLCFGR as a whole means every field must be valid, including the
 * ones we don't use: an early version left R = 0, which Renode flagged. */
#define PLL_M 8u
#define PLL_N 180u
#define PLL_P 2u
#define PLL_Q 8u
#define PLL_R 2u

#define TIMEOUT_LOOPS 1000000u

static int wait_set(volatile uint32_t *reg, uint32_t mask)
{
    for (uint32_t i = 0; i < TIMEOUT_LOOPS; i++) {
        if ((*reg & mask) == mask) return 0;
    }
    return -1;
}

int clock_init(void)
{
    /* 1. Power: enable the PWR clock, select voltage scale 1, turn on over-drive. */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR; /* read back: the clock must be running before PWR is accessed */
    PWR->CR |= PWR_CR_VOS; /* VOS = 0b11, scale 1 */

    /* HSI is on after reset; make sure it's ready. */
    RCC->CR |= RCC_CR_HSION;
    if (wait_set(&RCC->CR, RCC_CR_HSIRDY)) return CLOCK_ERR_HSI;

    /* Configure and start the PLL from HSI. P is encoded as (P/2 - 1). */
    RCC->CR &= ~RCC_CR_PLLON;
    RCC->PLLCFGR = (PLL_M << RCC_PLLCFGR_PLLM_Pos) |
                   (PLL_N << RCC_PLLCFGR_PLLN_Pos) |
                   (((PLL_P / 2u) - 1u) << RCC_PLLCFGR_PLLP_Pos) |
                   (PLL_Q << RCC_PLLCFGR_PLLQ_Pos) |
                   (PLL_R << RCC_PLLCFGR_PLLR_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSI;
    RCC->CR |= RCC_CR_PLLON;
    if (wait_set(&RCC->CR, RCC_CR_PLLRDY)) return CLOCK_ERR_PLL;

    /* Over-drive is enabled after the PLL is locked, per RM0390's over-drive sequence. */
    PWR->CR |= PWR_CR_ODEN;
    if (wait_set(&PWR->CSR, PWR_CSR_ODRDY)) return CLOCK_ERR_OVERDRIVE;
    PWR->CR |= PWR_CR_ODSWEN;
    if (wait_set(&PWR->CSR, PWR_CSR_ODSWRDY)) return CLOCK_ERR_OVERDRIVE;

    /* 2. Flash: 5 wait states + prefetch and caches. */
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS) return CLOCK_ERR_FLASH;

    /* 3. AHB /1, APB1 /4 (45 MHz), APB2 /2 (90 MHz). */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    /* 4. Switch to the PLL. */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    for (uint32_t i = 0; (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL; i++) {
        if (i == TIMEOUT_LOOPS) return CLOCK_ERR_SWITCH;
    }

    SystemCoreClock = BOARD_SYSCLK_HZ; /* CMSIS global, used by some library code */
    return CLOCK_OK;
}
