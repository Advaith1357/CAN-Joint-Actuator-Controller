/* NUCLEO-F446RE board definitions: clocks, pins, interrupt priorities.
 *
 * Pin map (Nucleo Morpho/Arduino headers):
 *   PA5   LD2 green LED (heartbeat)
 *   PA2   USART2_TX -> ST-LINK virtual COM port (debug console, 115200 8N1)
 *   PA11  CAN1_RX  \ to an SN65HVD230 transceiver
 *   PA12  CAN1_TX  /
 *   PA0   TIM2_CH1 \ quadrature encoder A/B (TIM2 is 32-bit)
 *   PA1   TIM2_CH2 /
 *   PA8   TIM1_CH1 -> DRV8871 IN1
 *   PA9   TIM1_CH2 -> DRV8871 IN2
 *   PA4   ADC1_IN4 <- DRV8871 IPROPI (current sense)
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f446xx.h"

/* Clock tree after clock_init(): HSI 16 MHz -> PLL -> 180 MHz SYSCLK.
 * APB1 = 45 MHz (max 45), APB2 = 90 MHz (max 90). Timers on a prescaled APB
 * bus get 2x the bus clock. */
#define BOARD_SYSCLK_HZ    180000000u
#define BOARD_HCLK_HZ      BOARD_SYSCLK_HZ
#define BOARD_PCLK1_HZ     (BOARD_HCLK_HZ / 4u)   /* CAN1, USART2 */
#define BOARD_PCLK2_HZ     (BOARD_HCLK_HZ / 2u)   /* ADC1 */
#define BOARD_TIMCLK1_HZ   (BOARD_PCLK1_HZ * 2u)  /* TIM2: 90 MHz */
#define BOARD_TIMCLK2_HZ   (BOARD_PCLK2_HZ * 2u)  /* TIM1: 180 MHz */

/* NVIC priorities (0 = most urgent, 15 = least). Anything that calls a
 * FreeRTOS ...FromISR function must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5). */
#define BOARD_IRQ_PRIO_CAN_RX  6

/* CAN node ID of this actuator. Override with -DBOARD_NODE_ID=n. */
#ifndef BOARD_NODE_ID
#define BOARD_NODE_ID 1
#endif

#endif
