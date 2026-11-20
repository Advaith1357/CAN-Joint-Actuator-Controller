/* Register-level driver for the STM32F4 bxCAN (CAN1), classic 11-bit frames. */
#ifndef BXCAN_H
#define BXCAN_H

#include <stdint.h>

#include "can_proto.h"

#define BXCAN_MAX_ACCEPT_IDS 8

typedef struct {
    uint32_t pclk_hz;          /* APB1 clock feeding CAN1 */
    uint32_t bitrate;          /* e.g. 500000 */
    int loopback;              /* 1 = internal loopback (self-test with no transceiver) */
    const uint16_t *accept_ids;/* standard IDs to accept in hardware; everything else is dropped */
    uint8_t n_accept_ids;      /* 1..BXCAN_MAX_ACCEPT_IDS */
} bxcan_config_t;

enum {
    BXCAN_OK = 0,
    BXCAN_ERR_BITRATE = -1,
    BXCAN_ERR_INIT_ENTER = -2,
    BXCAN_ERR_INIT_LEAVE = -3,
    BXCAN_ERR_ARGS = -4,
    BXCAN_ERR_NOMEM = -5,
};

/* Call before the scheduler starts. */
int bxcan_init(const bxcan_config_t *cfg);
/* Queue a frame in a free TX mailbox. 0 ok, -1 all three mailboxes busy.
 * Not reentrant: call from one task only. */
int bxcan_send(const can_frame_t *f);
/* Block up to timeout_ms for a received frame. 0 ok, 1 timeout. */
int bxcan_recv(can_frame_t *f, uint32_t timeout_ms);
/* Frames lost because the software queue or the hardware FIFO was full. */
uint32_t bxcan_rx_overruns(void);

#endif
