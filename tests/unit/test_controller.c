/* Controller tests. The closed-loop ones run the real controller against the
 * motor model at 1 kHz, i.e. a software-in-the-loop test with no RTOS. */
#include <math.h>

#include "controller.h"
#include "motor_model.h"
#include "test.h"

#define DT 0.001f

typedef struct {
    controller_t ctl;
    motor_model_t motor;
    uint32_t now_ms;
    float peak_current;
} rig_t;

static void rig_init(rig_t *r)
{
    ctl_config_t cfg = ctl_default_config();
    motor_params_t mp = motor_default_params();
    ctl_init(&r->ctl, &cfg);
    motor_model_init(&r->motor, &mp);
    r->now_ms = 1000;
    r->peak_current = 0.0f;
}

static void rig_step(rig_t *r)
{
    r->now_ms++;
    float v = ctl_step(&r->ctl, motor_model_encoder(&r->motor), r->motor.i, r->now_ms, DT);
    motor_model_step(&r->motor, v, DT);
    if (fabsf(r->motor.i) > r->peak_current) r->peak_current = fabsf(r->motor.i);
}

/* Run for ms milliseconds, re-sending cmd every 20 ms like a real host. */
static void rig_run(rig_t *r, const canp_cmd_t *cmd, int ms)
{
    for (int k = 0; k < ms; k++) {
        if (cmd && k % 20 == 0) ctl_on_command(&r->ctl, cmd, r->now_ms);
        rig_step(r);
    }
}

static canp_cmd_t vel_cmd(float rad_s)
{
    canp_cmd_t c = {.mode = CANP_MODE_VELOCITY, .setpoint = (int32_t)lrintf(rad_s / CANP_VEL_CMD_SCALE)};
    return c;
}

static canp_cmd_t pos_cmd(float rad)
{
    canp_cmd_t c = {.mode = CANP_MODE_POSITION, .setpoint = (int32_t)lrintf(rad / CANP_POS_SCALE)};
    return c;
}

static void test_motor_model_steady_state(void)
{
    motor_params_t p = motor_default_params();
    motor_model_t m;
    motor_model_init(&m, &p);
    for (int k = 0; k < 2000; k++) motor_model_step(&m, 12.0f, DT);
    /* Steady state: w = V kt / (kt ke + R b) */
    const float w_ss = 12.0f * p.kt / (p.kt * p.ke + p.R * p.b);
    CHECK_NEAR(m.omega, w_ss, 0.01f * w_ss);

    motor_model_init(&m, &p);
    m.load_torque = 100.0f; /* hold the rotor; current settles at V/R */
    m.p.J = 1e3f;
    for (int k = 0; k < 20; k++) motor_model_step(&m, 12.0f, DT);
    CHECK_NEAR(m.i, 12.0f / p.R, 0.1f);
}

static void test_velocity_step_response(void)
{
    rig_t r;
    rig_init(&r);
    canp_cmd_t cmd = vel_cmd(100.0f);
    float peak = 0.0f;
    int settled_at = -1;
    for (int k = 0; k < 500; k++) {
        if (k % 20 == 0) ctl_on_command(&r.ctl, &cmd, r.now_ms);
        rig_step(&r);
        if (r.motor.omega > peak) peak = r.motor.omega;
        if (settled_at < 0 && fabsf(r.motor.omega - 100.0f) < 2.0f) settled_at = k;
    }
    CHECK(r.ctl.state == CTL_ENABLED);
    CHECK(settled_at >= 0 && settled_at < 150);       /* within 2% in < 150 ms */
    CHECK(peak < 105.0f);                             /* < 5% overshoot */
    CHECK_NEAR(r.motor.omega, 100.0f, 1.0f);
    CHECK(r.peak_current < r.ctl.cfg.i_limit * 1.15f); /* current limit holds */
}

static void test_velocity_reverse_and_current_limit(void)
{
    rig_t r;
    rig_init(&r);
    canp_cmd_t fwd = vel_cmd(250.0f), rev = vel_cmd(-250.0f);
    rig_run(&r, &fwd, 600);
    CHECK_NEAR(r.motor.omega, 250.0f, 3.0f);
    r.peak_current = 0.0f;
    rig_run(&r, &rev, 800);
    CHECK_NEAR(r.motor.omega, -250.0f, 3.0f);
    /* A full reversal would draw ~ (24 + 12.5)/1 = 36 A unclamped. */
    CHECK(r.peak_current < r.ctl.cfg.i_limit * 1.15f);
    CHECK(r.ctl.state == CTL_ENABLED);
}

static void test_position_step(void)
{
    rig_t r;
    rig_init(&r);
    canp_cmd_t cmd = pos_cmd(2.0f);
    float peak = 0.0f;
    for (int k = 0; k < 800; k++) {
        if (k % 20 == 0) ctl_on_command(&r.ctl, &cmd, r.now_ms);
        rig_step(&r);
        if (r.motor.theta > peak) peak = r.motor.theta;
    }
    CHECK_NEAR(r.motor.theta, 2.0f, 0.005f);
    CHECK(peak < 2.05f);
    CHECK(fabsf(r.motor.omega) < 0.5f);
}

static void test_disturbance_rejection(void)
{
    rig_t r;
    rig_init(&r);
    canp_cmd_t cmd = vel_cmd(100.0f);
    rig_run(&r, &cmd, 300);
    r.motor.load_torque = 0.2f; /* 4 A worth of load */
    rig_run(&r, &cmd, 400);
    CHECK_NEAR(r.motor.omega, 100.0f, 2.0f); /* integrator removes the droop */
}

static void test_command_timeout_faults(void)
{
    rig_t r;
    rig_init(&r);
    canp_cmd_t cmd = vel_cmd(50.0f);
    rig_run(&r, &cmd, 200);
    ctl_on_command(&r.ctl, &cmd, r.now_ms); /* last command the node will hear */
    rig_run(&r, NULL, 100); /* exactly the timeout: still OK */
    CHECK(r.ctl.state == CTL_ENABLED);
    rig_run(&r, NULL, 1);
    CHECK(r.ctl.state == CTL_FAULT);
    CHECK(r.ctl.faults & CTL_FAULT_TIMEOUT);
    CHECK(r.ctl.out_v == 0.0f);
}

static void test_overcurrent_faults_after_trip_time(void)
{
    controller_t c;
    ctl_config_t cfg = ctl_default_config();
    ctl_init(&c, &cfg);
    canp_cmd_t cmd = vel_cmd(10.0f);
    ctl_on_command(&c, &cmd, 0);
    /* A short spike is tolerated... */
    for (uint32_t t = 1; t <= 10; t++) ctl_step(&c, 0, 20.0f, t, DT);
    ctl_step(&c, 0, 1.0f, 11, DT);
    CHECK(c.state == CTL_ENABLED);
    /* ...a sustained one isn't. */
    uint32_t t = 12;
    for (; t < 12 + cfg.oc_fault_ms; t++) {
        ctl_on_command(&c, &cmd, t);
        ctl_step(&c, 0, -20.0f, t, DT);
    }
    CHECK(c.state == CTL_ENABLED);
    ctl_step(&c, 0, -20.0f, t, DT);
    CHECK(c.state == CTL_FAULT);
    CHECK(c.faults == CTL_FAULT_OVERCURRENT);
}

static void test_fault_is_latched_until_cleared_in_idle(void)
{
    rig_t r;
    rig_init(&r);
    canp_cmd_t cmd = vel_cmd(50.0f);
    rig_run(&r, &cmd, 100);
    ctl_on_estop(&r.ctl);
    CHECK(r.ctl.state == CTL_FAULT && (r.ctl.faults & CTL_FAULT_ESTOP));

    /* Motion commands are ignored while faulted. */
    rig_run(&r, &cmd, 100);
    CHECK(r.ctl.state == CTL_FAULT);
    CHECK(r.ctl.out_v == 0.0f);

    /* CLEAR_FAULT together with a motion mode is rejected. */
    canp_cmd_t bad = cmd;
    bad.flags = CANP_CMD_FLAG_CLEAR_FAULT;
    ctl_on_command(&r.ctl, &bad, r.now_ms);
    CHECK(r.ctl.state == CTL_FAULT);

    canp_cmd_t clear = {.mode = CANP_MODE_IDLE, .flags = CANP_CMD_FLAG_CLEAR_FAULT};
    ctl_on_command(&r.ctl, &clear, r.now_ms);
    CHECK(r.ctl.state == CTL_DISABLED && r.ctl.faults == 0);

    rig_run(&r, &cmd, 300);
    CHECK(r.ctl.state == CTL_ENABLED);
    CHECK_NEAR(r.motor.omega, 50.0f, 1.5f);
}

static void test_idle_disables(void)
{
    rig_t r;
    rig_init(&r);
    canp_cmd_t cmd = vel_cmd(50.0f), idle = {.mode = CANP_MODE_IDLE};
    rig_run(&r, &cmd, 100);
    ctl_on_command(&r.ctl, &idle, r.now_ms);
    rig_step(&r);
    CHECK(r.ctl.state == CTL_DISABLED);
    CHECK(r.ctl.out_v == 0.0f);
    rig_run(&r, NULL, 500); /* disabled nodes don't time out */
    CHECK(r.ctl.state == CTL_DISABLED);
}

static void test_encoder_wraparound(void)
{
    controller_t c;
    ctl_config_t cfg = ctl_default_config();
    cfg.vel_lpf_alpha = 1.0f;
    ctl_init(&c, &cfg);
    ctl_step(&c, INT32_MAX - 1, 0.0f, 1, DT);
    ctl_step(&c, INT32_MIN + 1, 0.0f, 2, DT); /* +3 counts across the wrap */
    const float expect = 3.0f * 6.28318530718f / 4096.0f / DT;
    CHECK_NEAR(c.vel, expect, 1e-3);
}

void run_controller_tests(void)
{
    RUN(test_motor_model_steady_state);
    RUN(test_velocity_step_response);
    RUN(test_velocity_reverse_and_current_limit);
    RUN(test_position_step);
    RUN(test_disturbance_rejection);
    RUN(test_command_timeout_faults);
    RUN(test_overcurrent_faults_after_trip_time);
    RUN(test_fault_is_latched_until_cleared_in_idle);
    RUN(test_idle_disables);
    RUN(test_encoder_wraparound);
}
