/* Thin OS abstraction. The app is written against this so the same task code
 * runs on FreeRTOS (port/stm32) and on POSIX threads (port/host). Every call
 * maps 1:1 onto a FreeRTOS primitive:
 *
 *   osal_task_create  -> xTaskCreate
 *   osal_queue_*      -> xQueueCreate / xQueueSend / xQueueReceive
 *   osal_mutex_*      -> xSemaphoreCreateMutex / Take / Give
 *   osal_delay_until  -> vTaskDelayUntil   (drift-free periodic tasks)
 *   osal_now_ms       -> xTaskGetTickCount (1 kHz tick)
 */
#ifndef OSAL_H
#define OSAL_H

#include <stddef.h>
#include <stdint.h>

#define OSAL_WAIT_FOREVER UINT32_MAX

typedef struct osal_queue osal_queue_t;
typedef struct osal_mutex osal_mutex_t;
typedef void (*osal_task_fn)(void *arg);

/* Higher number = higher priority (FreeRTOS convention). */
int osal_task_create(osal_task_fn fn, void *arg, const char *name, int priority, size_t stack_words);

osal_queue_t *osal_queue_create(size_t length, size_t item_size);
/* 0 on success, -1 on timeout / full. timeout 0 = non-blocking. */
int osal_queue_send(osal_queue_t *q, const void *item, uint32_t timeout_ms);
int osal_queue_recv(osal_queue_t *q, void *item, uint32_t timeout_ms);

osal_mutex_t *osal_mutex_create(void);
void osal_mutex_lock(osal_mutex_t *m);
void osal_mutex_unlock(osal_mutex_t *m);

uint32_t osal_now_ms(void);
/* Sleep until *last_wake + period, then advance *last_wake by period. */
void osal_delay_until(uint32_t *last_wake_ms, uint32_t period_ms);

/* Start the scheduler. Doesn't return. */
void osal_start(void);

#endif
