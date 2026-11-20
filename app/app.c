/* Three tasks:
 *
 *   control (prio 3, 1 kHz) owns the controller. Drains the command queue,
 *                           reads sensors, runs ctl_step, drives the motor and
 *                           publishes a snapshot.
 *   can_rx  (prio 2)        blocks on CAN, decodes, forwards commands to the
 *                           control task through a queue.
 *   telem   (prio 1, 100 Hz) copies the snapshot under a mutex and transmits
 *                           telemetry and status.
 *
 * The controller is only touched by the control task, so it needs no locking.
 * The only shared data is the snapshot (mutex) and the e-stop flag (atomic).
 */
#include "app.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

#include "hal.h"
#include "osal.h"

#define CMD_QUEUE_LEN 8

typedef struct {
    ctl_state_t state;
    uint8_t faults;
    uint8_t mode;
    uint8_t last_seq;
    float pos, vel, current;
} snapshot_t;

static struct {
    app_config_t cfg;
    controller_t ctl;
    osal_queue_t *cmd_q;
    osal_mutex_t *snap_mtx;
    snapshot_t snap;
    uint32_t start_ms;
    /* An e-stop must never be lost to a full queue, so it gets its own flag. */
    atomic_bool estop_req;
    atomic_uint rx_dropped;
} g;

static void control_task(void *arg)
{
    (void)arg;
    const float dt = (float)APP_CONTROL_PERIOD_MS * 1e-3f;
    uint32_t wake = osal_now_ms();

    for (;;) {
        osal_delay_until(&wake, APP_CONTROL_PERIOD_MS);
        const uint32_t now = osal_now_ms();

        if (atomic_exchange(&g.estop_req, false)) {
            ctl_on_estop(&g.ctl);
        }
        canp_cmd_t cmd;
        while (osal_queue_recv(g.cmd_q, &cmd, 0) == 0) {
            ctl_on_command(&g.ctl, &cmd, now);
        }

        const float v = ctl_step(&g.ctl, hal_encoder_count(), hal_current_a(), now, dt);
        hal_set_voltage(v);

        osal_mutex_lock(g.snap_mtx);
        g.snap.state = g.ctl.state;
        g.snap.faults = g.ctl.faults;
        g.snap.mode = g.ctl.mode;
        g.snap.last_seq = g.ctl.last_seq;
        g.snap.pos = g.ctl.pos;
        g.snap.vel = g.ctl.vel;
        g.snap.current = g.ctl.current;
        osal_mutex_unlock(g.snap_mtx);
    }
}

static void can_rx_task(void *arg)
{
    (void)arg;
    for (;;) {
        can_frame_t f;
        if (hal_can_recv(&f, 100) != 0) continue;

        uint8_t node;
        switch (canp_classify(&f, &node)) {
        case CANP_KIND_ESTOP:
            atomic_store(&g.estop_req, true);
            break;
        case CANP_KIND_CMD: {
            if (node != g.cfg.node_id) break;
            canp_cmd_t cmd;
            /* Malformed or dropped commands don't feed the watchdog, so a
             * flood of garbage still ends in a safe timeout. */
            if (canp_decode_cmd(&f, &node, &cmd) != CANP_OK || osal_queue_send(g.cmd_q, &cmd, 0) != 0) {
                atomic_fetch_add(&g.rx_dropped, 1u);
            }
            break;
        }
        case CANP_KIND_UNKNOWN:
            hal_on_unhandled_frame(&f);
            break;
        default: /* other nodes' status/telemetry */
            break;
        }
    }
}

static void telem_task(void *arg)
{
    (void)arg;
    uint32_t wake = osal_now_ms();
    uint32_t n = 0;

    for (;;) {
        osal_delay_until(&wake, APP_TELEM_PERIOD_MS);

        snapshot_t s;
        osal_mutex_lock(g.snap_mtx);
        s = g.snap;
        osal_mutex_unlock(g.snap_mtx);

        can_frame_t f;
        const canp_telem_t t = {
            .pos = canp_sat_i32(s.pos / CANP_POS_SCALE),
            .vel = canp_sat_i16(s.vel / CANP_VEL_TLM_SCALE),
            .current = canp_sat_i16(s.current / CANP_CUR_SCALE),
        };
        if (canp_encode_telem(g.cfg.node_id, &t, &f) == CANP_OK) hal_can_send(&f);

        if (++n % APP_STATUS_DIVIDER == 0) {
            const canp_status_t st = {
                .state = (uint8_t)s.state,
                .faults = s.faults,
                .mode = s.mode,
                .last_seq = s.last_seq,
                .uptime_ms = osal_now_ms() - g.start_ms,
            };
            if (canp_encode_status(g.cfg.node_id, &st, &f) == CANP_OK) hal_can_send(&f);
        }
    }
}

int app_init(const app_config_t *cfg)
{
    if (cfg->node_id > CANP_MAX_NODE) return -1;
    memset(&g.snap, 0, sizeof(g.snap));
    g.cfg = *cfg;
    ctl_init(&g.ctl, &cfg->ctl);
    atomic_init(&g.estop_req, false);
    atomic_init(&g.rx_dropped, 0u);
    g.start_ms = osal_now_ms();

    g.cmd_q = osal_queue_create(CMD_QUEUE_LEN, sizeof(canp_cmd_t));
    g.snap_mtx = osal_mutex_create();
    if (!g.cmd_q || !g.snap_mtx) return -1;

    if (osal_task_create(control_task, NULL, "control", 3, 512)) return -1;
    if (osal_task_create(can_rx_task, NULL, "can_rx", 2, 384)) return -1;
    if (osal_task_create(telem_task, NULL, "telem", 1, 384)) return -1;
    return 0;
}
