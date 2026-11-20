/* PID controller with derivative-on-measurement, a filtered D term and
 * conditional-integration anti-windup. No dynamic memory, no OS calls. */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float d_filter_alpha; /* 0..1: weight of a new sample in the D-term low-pass; 1 = unfiltered */
} pid_config_t;

typedef struct {
    pid_config_t cfg;
    float integ;     /* integral contribution, already multiplied by ki */
    float prev_meas;
    float d_state;   /* filtered d(measurement)/dt */
    int primed;      /* 0 until the first sample, so the D term doesn't kick on start-up */
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, const pid_config_t *cfg);
void pid_reset(pid_ctrl_t *p);
/* Limits can change every step (the controller uses this for current limiting). */
void pid_set_output_limits(pid_ctrl_t *p, float out_min, float out_max);
float pid_update(pid_ctrl_t *p, float setpoint, float measurement, float dt);

#endif
