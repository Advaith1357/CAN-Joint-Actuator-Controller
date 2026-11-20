#include "pid.h"
#include "test.h"

static pid_config_t cfg(float kp, float ki, float kd)
{
    pid_config_t c = {.kp = kp, .ki = ki, .kd = kd, .out_min = -10.0f, .out_max = 10.0f, .d_filter_alpha = 1.0f};
    return c;
}

static void test_pid_proportional_only(void)
{
    pid_ctrl_t p;
    pid_config_t c = cfg(2.0f, 0.0f, 0.0f);
    pid_init(&p, &c);
    CHECK_NEAR(pid_update(&p, 3.0f, 1.0f, 0.001f), 4.0f, 1e-6);
    CHECK_NEAR(pid_update(&p, 1.0f, 3.0f, 0.001f), -4.0f, 1e-6);
}

static void test_pid_integral_accumulates(void)
{
    pid_ctrl_t p;
    pid_config_t c = cfg(0.0f, 10.0f, 0.0f);
    pid_init(&p, &c);
    float out = 0.0f;
    for (int i = 0; i < 100; i++) out = pid_update(&p, 1.0f, 0.0f, 0.001f);
    /* 10 * 1.0 * 0.1 s = 1.0 */
    CHECK_NEAR(out, 1.0f, 1e-4);
}

static void test_pid_output_clamped(void)
{
    pid_ctrl_t p;
    pid_config_t c = cfg(100.0f, 0.0f, 0.0f);
    pid_init(&p, &c);
    CHECK_NEAR(pid_update(&p, 1.0f, 0.0f, 0.001f), 10.0f, 1e-6);
    CHECK_NEAR(pid_update(&p, -1.0f, 0.0f, 0.001f), -10.0f, 1e-6);
}

static void test_pid_anti_windup(void)
{
    /* Saturate for 10 s with a big error. Without anti-windup the integrator
     * would hold ~1000 and need ages to unwind; with it, the output leaves
     * saturation as soon as the error changes sign. */
    pid_ctrl_t p;
    pid_config_t c = cfg(1.0f, 10.0f, 0.0f);
    pid_init(&p, &c);
    for (int i = 0; i < 10000; i++) pid_update(&p, 100.0f, 0.0f, 0.001f);
    CHECK(p.integ <= 10.0f);
    float out = pid_update(&p, 0.0f, 5.0f, 0.001f); /* error = -5 */
    CHECK(out < 10.0f);
    CHECK(out > -10.0f);
}

static void test_pid_no_derivative_kick_on_setpoint_step(void)
{
    pid_ctrl_t p;
    pid_config_t c = cfg(0.0f, 0.0f, 1.0f);
    pid_init(&p, &c);
    pid_update(&p, 0.0f, 2.0f, 0.001f);
    /* Setpoint jumps but the measurement is unchanged: D term stays 0. */
    CHECK_NEAR(pid_update(&p, 5.0f, 2.0f, 0.001f), 0.0f, 1e-6);
    /* Measurement rising at 1 unit/s -> D = -kd * 1 = -1. */
    CHECK_NEAR(pid_update(&p, 5.0f, 2.001f, 0.001f), -1.0f, 1e-3);
}

static void test_pid_first_sample_has_no_derivative(void)
{
    pid_ctrl_t p;
    pid_config_t c = cfg(0.0f, 0.0f, 1.0f);
    pid_init(&p, &c);
    CHECK_NEAR(pid_update(&p, 0.0f, 7.0f, 0.001f), 0.0f, 1e-6);
}

void run_pid_tests(void)
{
    RUN(test_pid_proportional_only);
    RUN(test_pid_integral_accumulates);
    RUN(test_pid_output_clamped);
    RUN(test_pid_anti_windup);
    RUN(test_pid_no_derivative_kick_on_setpoint_step);
    RUN(test_pid_first_sample_has_no_derivative);
}
