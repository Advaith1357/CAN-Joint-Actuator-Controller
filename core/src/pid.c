#include "pid.h"

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

void pid_init(pid_ctrl_t *p, const pid_config_t *cfg)
{
    p->cfg = *cfg;
    pid_reset(p);
}

void pid_reset(pid_ctrl_t *p)
{
    p->integ = 0.0f;
    p->prev_meas = 0.0f;
    p->d_state = 0.0f;
    p->primed = 0;
}

void pid_set_output_limits(pid_ctrl_t *p, float out_min, float out_max)
{
    p->cfg.out_min = out_min;
    p->cfg.out_max = out_max;
}

float pid_update(pid_ctrl_t *p, float setpoint, float measurement, float dt)
{
    const pid_config_t *c = &p->cfg;
    const float err = setpoint - measurement;
    const float p_term = c->kp * err;

    /* Derivative on measurement: a setpoint step doesn't produce a D spike. */
    float d_term = 0.0f;
    if (p->primed && dt > 0.0f) {
        const float raw = -(measurement - p->prev_meas) / dt;
        p->d_state += c->d_filter_alpha * (raw - p->d_state);
        d_term = c->kd * p->d_state;
    }
    p->prev_meas = measurement;
    p->primed = 1;

    /* Conditional integration: if the output is already saturated and the
     * error would push it further, freeze the integrator (anti-windup). */
    float integ = p->integ + c->ki * err * dt;
    const float unsat = p_term + integ + d_term;
    if ((unsat > c->out_max && err > 0.0f) || (unsat < c->out_min && err < 0.0f)) {
        integ = p->integ;
    }
    p->integ = clampf(integ, c->out_min, c->out_max);

    return clampf(p_term + p->integ + d_term, c->out_min, c->out_max);
}
