#ifndef APP_H
#define APP_H

#include <stdint.h>

#include "controller.h"

#define APP_CONTROL_PERIOD_MS 1u   /* 1 kHz control loop */
#define APP_TELEM_PERIOD_MS   10u  /* 100 Hz telemetry */
#define APP_STATUS_DIVIDER    10u  /* status every 10th telemetry frame -> 10 Hz */

typedef struct {
    uint8_t node_id;
    ctl_config_t ctl;
} app_config_t;

/* Create queues and tasks. Call before osal_start(). Returns 0 on success. */
int app_init(const app_config_t *cfg);

#endif
