/* FreeRTOS implementation of app/osal.h. Each call is a thin wrapper; the
 * only real work is converting between the OSAL's 0/-1 + milliseconds
 * convention and FreeRTOS's pdPASS + ticks. */
#include "osal.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

/* The OSAL speaks milliseconds and stores wake times in uint32_t. That only
 * works if one tick is one millisecond and ticks are 32 bits wide. */
_Static_assert(configTICK_RATE_HZ == 1000, "osal assumes a 1 kHz tick");
_Static_assert(sizeof(TickType_t) == sizeof(uint32_t), "osal assumes 32-bit ticks");

static TickType_t to_ticks(uint32_t timeout_ms)
{
    return timeout_ms == OSAL_WAIT_FOREVER ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
}

int osal_task_create(osal_task_fn fn, void *arg, const char *name, int priority, size_t stack_words)
{
    /* FreeRTOS stack depth is in words (4 bytes on Cortex-M), same as the OSAL. */
    return xTaskCreate(fn, name, (configSTACK_DEPTH_TYPE)stack_words, arg, (UBaseType_t)priority, NULL) == pdPASS ? 0 : -1;
}

osal_queue_t *osal_queue_create(size_t length, size_t item_size)
{
    /* osal_queue_t is an opaque handle; on this port it *is* the FreeRTOS handle. */
    return (osal_queue_t *)xQueueCreate((UBaseType_t)length, (UBaseType_t)item_size);
}

int osal_queue_send(osal_queue_t *q, const void *item, uint32_t timeout_ms)
{
    return xQueueSend((QueueHandle_t)q, item, to_ticks(timeout_ms)) == pdPASS ? 0 : -1;
}

int osal_queue_recv(osal_queue_t *q, void *item, uint32_t timeout_ms)
{
    return xQueueReceive((QueueHandle_t)q, item, to_ticks(timeout_ms)) == pdPASS ? 0 : -1;
}

osal_mutex_t *osal_mutex_create(void)
{
    /* A FreeRTOS mutex (not a binary semaphore) gives priority inheritance:
     * if the low-priority telemetry task holds it when the 1 kHz control task
     * wants it, telemetry is boosted so the control loop isn't delayed by
     * medium-priority work (priority inversion). */
    return (osal_mutex_t *)xSemaphoreCreateMutex();
}

void osal_mutex_lock(osal_mutex_t *m)
{
    (void)xSemaphoreTake((SemaphoreHandle_t)m, portMAX_DELAY);
}

void osal_mutex_unlock(osal_mutex_t *m)
{
    (void)xSemaphoreGive((SemaphoreHandle_t)m);
}

uint32_t osal_now_ms(void)
{
    return (uint32_t)xTaskGetTickCount();
}

void osal_delay_until(uint32_t *last_wake_ms, uint32_t period_ms)
{
    /* Unlike vTaskDelay (relative to "now"), this wakes at fixed multiples of
     * the period, so execution-time jitter doesn't accumulate as drift. */
    (void)xTaskDelayUntil((TickType_t *)last_wake_ms, pdMS_TO_TICKS(period_ms));
}

void osal_start(void)
{
    vTaskStartScheduler();
    /* Only reached if there wasn't enough heap for the idle task. */
    board_assert_failed(__FILE__, __LINE__);
}

/* --- Hooks enabled in FreeRTOSConfig.h -------------------------------- */

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    board_assert_failed("stack overflow", 0);
}

void vApplicationMallocFailedHook(void)
{
    board_assert_failed("malloc failed", 0);
}
