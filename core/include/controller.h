/* Actuator controller: cascaded position -> velocity loops, current limiting,
 * and the safety state machine (command watchdog, overcurrent, e-stop).
 *
 * Pure logic: it takes sensor readings and a timestamp and returns a voltage
 * command, so the same code runs on the MCU, in the simulator and in unit
 * tests. It's owned by a single task; nothing here is thread-safe. */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>

#include "can_proto.h"
#include "pid.h"

typedef enum {
    CTL_DISABLED = 0,
    CTL_ENABLED = 1,
    CTL_FAULT = 2,
} ctl_state_t;

#define CTL_FAULT_TIMEOUT     0x01u
#define CTL_FAULT_OVERCURRENT 0x02u
#define CTL_FAULT_ESTOP       0x04u

typedef struct {
    pid_config_t vel_pid;    /* velocity loop: rad/s in, volts out */
    float pos_kp;            /* position loop: rad error -> rad/s target */
    float vel_limit;         /* |velocity target| clamp [rad/s] */
    float vel_lpf_alpha;     /* 0..1, low-pass on the encoder-differenced speed */
    float i_limit;           /* current limit enforced by the voltage clamp [A] */
    float oc_fault_a;        /* hard overcurrent threshold [A] ... */
    uint32_t oc_fault_ms;    /* ... sustained for this long -> FAULT */
    uint32_t cmd_timeout_ms; /* no command for this long while enabled -> FAULT */
    float motor_R;           /* motor model used for the current-limit clamp */
    float motor_ke;
    float v_supply;
    uint32_t counts_per_rev;
} ctl_config_t;

typedef struct {
    ctl_config_t cfg;
    pid_ctrl_t vel_pid;
    ctl_state_t state;
    uint8_t faults;
    uint8_t mode;       /* canp_mode_t */
    uint8_t last_seq;
    float sp_vel;       /* rad/s */
    float sp_pos;       /* rad */
    uint32_t last_cmd_ms;
    uint32_t oc_since_ms;
    int oc_active;
    int32_t prev_count;
    int primed;
    /* Latest estimates, exposed for telemetry. */
    float pos;
    float vel;
    float current;
    float target_vel;
    float out_v;
} controller_t;

/* Tuned defaults for motor_default_params(); see "Tuning" in README.md. */
ctl_config_t ctl_default_config(void);

void ctl_init(controller_t *c, const ctl_config_t *cfg);
void ctl_on_command(controller_t *c, const canp_cmd_t *cmd, uint32_t now_ms);
void ctl_on_estop(controller_t *c);
/* One control period. Returns the voltage to apply (0 unless ENABLED). */
float ctl_step(controller_t *c, int32_t enc_count, float current_a, uint32_t now_ms, float dt);

#endif
