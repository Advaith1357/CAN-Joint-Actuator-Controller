/* POSIX implementation of osal.h for the host simulator. Priorities and stack
 * sizes are ignored; tasks are plain pthreads. */
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "osal.h"

struct osal_queue {
    pthread_mutex_t mtx;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
    size_t len, item_size, head, count;
    unsigned char *buf;
};

struct osal_mutex {
    pthread_mutex_t mtx;
};

typedef struct {
    osal_task_fn fn;
    void *arg;
} task_start_t;

static uint64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

uint32_t osal_now_ms(void)
{
    return (uint32_t)(mono_us() / 1000u);
}

static void sleep_us(uint64_t us)
{
    struct timespec ts = {.tv_sec = (time_t)(us / 1000000u), .tv_nsec = (long)(us % 1000000u) * 1000L};
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
    }
}

void osal_delay_until(uint32_t *last_wake_ms, uint32_t period_ms)
{
    *last_wake_ms += period_ms;
    const uint64_t now = mono_us();
    const int32_t remaining_ms = (int32_t)(*last_wake_ms - (uint32_t)(now / 1000u));
    if (remaining_ms > 0) {
        /* Sleep to the start of the target millisecond. */
        const uint64_t target_us = (now / 1000u + (uint64_t)remaining_ms) * 1000u;
        sleep_us(target_us - now);
    }
    /* If we're late, return immediately, like vTaskDelayUntil. */
}

static void *task_trampoline(void *p)
{
    task_start_t s = *(task_start_t *)p;
    free(p);
    s.fn(s.arg);
    return NULL;
}

int osal_task_create(osal_task_fn fn, void *arg, const char *name, int priority, size_t stack_words)
{
    (void)name;
    (void)priority;
    (void)stack_words;
    task_start_t *s = malloc(sizeof(*s));
    if (!s) return -1;
    s->fn = fn;
    s->arg = arg;
    pthread_t t;
    if (pthread_create(&t, NULL, task_trampoline, s) != 0) {
        free(s);
        return -1;
    }
    pthread_detach(t);
    return 0;
}

static void deadline_after(struct timespec *ts, uint32_t ms)
{
    clock_gettime(CLOCK_REALTIME, ts); /* pthread_cond_timedwait uses CLOCK_REALTIME */
    ts->tv_sec += ms / 1000u;
    ts->tv_nsec += (long)(ms % 1000u) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

/* Wait on cv until pred() holds or the timeout passes. Called with q->mtx held. */
static int wait_for(osal_queue_t *q, pthread_cond_t *cv, int (*pred)(const osal_queue_t *), uint32_t timeout_ms)
{
    if (pred(q)) return 0;
    if (timeout_ms == 0) return -1;
    if (timeout_ms == OSAL_WAIT_FOREVER) {
        while (!pred(q)) pthread_cond_wait(cv, &q->mtx);
        return 0;
    }
    struct timespec dl;
    deadline_after(&dl, timeout_ms);
    while (!pred(q)) {
        if (pthread_cond_timedwait(cv, &q->mtx, &dl) == ETIMEDOUT) return pred(q) ? 0 : -1;
    }
    return 0;
}

static int has_items(const osal_queue_t *q) { return q->count > 0; }
static int has_space(const osal_queue_t *q) { return q->count < q->len; }

osal_queue_t *osal_queue_create(size_t length, size_t item_size)
{
    osal_queue_t *q = calloc(1, sizeof(*q));
    if (!q) return NULL;
    q->buf = calloc(length, item_size);
    if (!q->buf) {
        free(q);
        return NULL;
    }
    q->len = length;
    q->item_size = item_size;
    pthread_mutex_init(&q->mtx, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
    return q;
}

int osal_queue_send(osal_queue_t *q, const void *item, uint32_t timeout_ms)
{
    pthread_mutex_lock(&q->mtx);
    int rc = wait_for(q, &q->not_full, has_space, timeout_ms);
    if (rc == 0) {
        const size_t tail = (q->head + q->count) % q->len;
        memcpy(q->buf + tail * q->item_size, item, q->item_size);
        q->count++;
        pthread_cond_signal(&q->not_empty);
    }
    pthread_mutex_unlock(&q->mtx);
    return rc;
}

int osal_queue_recv(osal_queue_t *q, void *item, uint32_t timeout_ms)
{
    pthread_mutex_lock(&q->mtx);
    int rc = wait_for(q, &q->not_empty, has_items, timeout_ms);
    if (rc == 0) {
        memcpy(item, q->buf + q->head * q->item_size, q->item_size);
        q->head = (q->head + 1) % q->len;
        q->count--;
        pthread_cond_signal(&q->not_full);
    }
    pthread_mutex_unlock(&q->mtx);
    return rc;
}

osal_mutex_t *osal_mutex_create(void)
{
    osal_mutex_t *m = malloc(sizeof(*m));
    if (m) pthread_mutex_init(&m->mtx, NULL);
    return m;
}

void osal_mutex_lock(osal_mutex_t *m) { pthread_mutex_lock(&m->mtx); }
void osal_mutex_unlock(osal_mutex_t *m) { pthread_mutex_unlock(&m->mtx); }

void osal_start(void)
{
    for (;;) pause(); /* tasks run on their own threads; main just waits for a signal */
}
