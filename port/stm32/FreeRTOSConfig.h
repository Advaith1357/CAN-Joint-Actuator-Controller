/* FreeRTOS configuration for the STM32F446 (Cortex-M4F @ 180 MHz). */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "board.h"

/* --- Scheduler -------------------------------------------------------- */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1  /* CLZ-based ready-list lookup */
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      BOARD_SYSCLK_HZ
#define configTICK_RATE_HZ                      1000  /* 1 tick == 1 ms; app/osal.h relies on this */
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                128   /* words */
#define configMAX_TASK_NAME_LEN                 12
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TIME_SLICING                  1

/* --- Features used by the OSAL ---------------------------------------- */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0

/* --- Memory ------------------------------------------------------------ */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configTOTAL_HEAP_SIZE                   (16 * 1024)  /* heap_4 */
#define configAPPLICATION_ALLOCATED_HEAP        0

/* --- Debug and safety hooks ------------------------------------------- */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2   /* canary check on every context switch */
#define configUSE_TRACE_FACILITY                0
#define configGENERATE_RUN_TIME_STATS           0

void board_assert_failed(const char *file, int line);
#define configASSERT(x) do { if (!(x)) board_assert_failed(__FILE__, __LINE__); } while (0)

/* --- Interrupt priorities ---------------------------------------------
 * The STM32F4 implements 4 priority bits (16 levels, 0 = most urgent), held
 * in the top 4 bits of each 8-bit priority register.
 *
 *   KERNEL priority (15):  SysTick and PendSV run at the lowest urgency, so a
 *                          context switch never preempts a real ISR.
 *   MAX_SYSCALL (5):       FreeRTOS masks interrupts with priority 5..15 in its
 *                          critical sections. ISRs that call ...FromISR APIs
 *                          (CAN RX) must be in 5..15. ISRs in 0..4 are never
 *                          masked (lowest latency) but may not touch the kernel. */
#define configPRIO_BITS                          __NVIC_PRIO_BITS
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY          (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* --- API functions to include ----------------------------------------- */
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetSchedulerState          1

/* --- Map the port's handlers onto the CMSIS vector table names --------- */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler  /* SysTick is only enabled by vTaskStartScheduler */

#endif
