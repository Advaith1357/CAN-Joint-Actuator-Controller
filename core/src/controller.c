#include "controller.h"

#include <math.h>
#include <string.h>

#define TWO_PI_F 6.28318530718f

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

ctl_config_t ctl_default_config(void)
{
    ctl_config_t c = {
        /* Plant V -> w is ~first order: gain 1/ke = 20 rad/s/V, tau = J R / (kt ke) = 40 ms.
         * PI zero placed on the plant pole (ki/kp = 1/tau) for a ~100 rad/s
         * closed-loop bandwidth: kp = tau * wc / K = 0.2. */
        .vel_pid = {
            .kp = 0.2f,
            .ki = 5.0f,
            .kd = 0.0f,
            .out_min = -24.0f,
            .out_max = 24.0f,
            .d_filter_alpha = 1.0f,
        },
        .pos_kp = 20.0f,  /* ~5x below the velocity-loop bandwidth */
        .vel_limit = 300.0f,
        .vel_lpf_alpha = 0.3f, /* ~57 Hz cutoff at 1 kHz */
        .i_limit = 8.0f,
        .oc_fault_a = 12.0f,
        .oc_fault_ms = 20,
        .cmd_timeout_ms = 100,
        .motor_R = 1.0f,
        .motor_ke = 0.05f,
        .v_supply = 24.0f,
        .counts_per_rev = 4096,
    };
    return c;
}

void ctl_init(controller_t *c, const ctl_config_t *cfg)
{
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    pid_init(&c->vel_pid, &cfg->vel_pid);
    c->state = CTL_DISABLED;
    c->mode = CANP_MODE_IDLE;
}

static void enter_fault(controller_t *c, uint8_t flag)
{
    c->faults |= flag;
    c->state = CTL_FAULT;
    c->mode = CANP_MODE_IDLE;
    pid_reset(&c->vel_pid);
}

void ctl_on_estop(controller_t *c)
{
    enter_fault(c, CTL_FAULT_ESTOP);
}

void ctl_on_command(controller_t *c, const canp_cmd_t *cmd, uint32_t now_ms)
{
    c->last_seq = cmd->seq;

    if (c->state == CTL_FAULT) {
        /* Faults are latched. Clearing requires IDLE, so a single frame can't
         * both clear a fault and start the motor. */
        if ((cmd->flags & CANP_CMD_FLAG_CLEAR_FAULT) && cmd->mode == CANP_MODE_IDLE) {
            c->state = CTL_DISABLED;
            c->faults = 0;
            c->last_cmd_ms = now_ms;
        }
        return;
    }

    c->last_cmd_ms = now_ms;

    switch (cmd->mode) {
    case CANP_MODE_VELOCITY:
        c->sp_vel = clampf((float)cmd->setpoint * CANP_VEL_CMD_SCALE, -c->cfg.vel_limit, c->cfg.vel_limit);
        break;
    case CANP_MODE_POSITION:
        c->sp_pos = (float)cmd->setpoint * CANP_POS_SCALE;
        break;
    default: /* IDLE (decoder already rejects unknown modes) */
        c->state = CTL_DISABLED;
        c->mode = CANP_MODE_IDLE;
        pid_reset(&c->vel_pid);
        return;
    }

    if (c->state != CTL_ENABLED) {
        pid_reset(&c->vel_pid); /* start from a clean integrator */
        c->state = CTL_ENABLED;
    }
    c->mode = cmd->mode;
}

float ctl_step(controller_t *c, int32_t enc_count, float current_a, uint32_t now_ms, float dt)
{
    const ctl_config_t *cfg = &c->cfg;

    /* --- Sensing: position from the encoder, speed by differencing + low-pass. */
    if (!c->primed) {
        c->prev_count = enc_count;
        c->primed = 1;
    }
    /* Unsigned subtraction handles 32-bit counter wraparound. */
    const int32_t delta = (int32_t)((uint32_t)enc_count - (uint32_t)c->prev_count);
    c->prev_count = enc_count;
    const float rad_per_count = TWO_PI_F / (float)cfg->counts_per_rev;
    const float raw_vel = dt > 0.0f ? (float)delta * rad_per_count / dt : 0.0f;
    c->vel += cfg->vel_lpf_alpha * (raw_vel - c->vel);
    c->pos = (float)enc_count * rad_per_count;
    c->current = current_a;

    /* --- Safety checks run in every state. */
    if (fabsf(current_a) > cfg->oc_fault_a) {
        if (!c->oc_active) {
            c->oc_active = 1;
            c->oc_since_ms = now_ms;
        }
        if ((uint32_t)(now_ms - c->oc_since_ms) >= cfg->oc_fault_ms) {
            enter_fault(c, CTL_FAULT_OVERCURRENT);
        }
    } else {
        c->oc_active = 0;
    }

    if (c->state == CTL_ENABLED && (uint32_t)(now_ms - c->last_cmd_ms) > cfg->cmd_timeout_ms) {
        enter_fault(c, CTL_FAULT_TIMEOUT);
    }

    if (c->state != CTL_ENABLED) {
        c->target_vel = 0.0f;
        c->out_v = 0.0f;
        return 0.0f;
    }

    /* --- Outer loop: position error -> velocity target. */
    float target = c->sp_vel;
    if (c->mode == CANP_MODE_POSITION) {
        target = clampf(cfg->pos_kp * (c->sp_pos - c->pos), -cfg->vel_limit, cfg->vel_limit);
    }
    c->target_vel = target;

    /* --- Current limit without a current loop: the winding current is
     * (V - ke*w)/R in steady state, so keeping V within ke*w +/- R*i_limit
     * bounds it. Feeding these limits to the PID keeps its anti-windup honest. */
    const float bemf = cfg->motor_ke * c->vel;
    float hi = fminf(cfg->v_supply, bemf + cfg->motor_R * cfg->i_limit);
    float lo = fmaxf(-cfg->v_supply, bemf - cfg->motor_R * cfg->i_limit);
    if (lo > hi) {
        /* Only possible if the shaft is being back-driven faster than the
         * supply can follow; command the clamped back-EMF voltage. */
        lo = hi = clampf(bemf, -cfg->v_supply, cfg->v_supply);
    }
    pid_set_output_limits(&c->vel_pid, lo, hi);

    c->out_v = pid_update(&c->vel_pid, target, c->vel, dt);
    return c->out_v;
}
