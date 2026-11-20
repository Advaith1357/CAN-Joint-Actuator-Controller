/* Brushed DC motor + inertia plant model, used by the host simulator and the
 * closed-loop unit tests. Not compiled into the target firmware.
 *
 *   L di/dt = V - R i - ke w
 *   J dw/dt = kt i - b w - tau_load
 */
#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include <stdint.h>

typedef struct {
    float R;         /* winding resistance [ohm] */
    float L;         /* winding inductance [H] */
    float kt;        /* torque constant [N*m/A] */
    float ke;        /* back-EMF constant [V*s/rad] */
    float J;         /* rotor + load inertia [kg*m^2] */
    float b;         /* viscous friction [N*m*s/rad] */
    float v_supply;  /* bus voltage [V]; the commanded voltage is clamped to +/- this */
    uint32_t counts_per_rev; /* quadrature encoder counts per revolution (after x4) */
} motor_params_t;

typedef struct {
    motor_params_t p;
    float i;      /* winding current [A] */
    float omega;  /* shaft speed [rad/s] */
    float theta;  /* shaft angle [rad] */
    float load_torque; /* external torque opposing positive rotation [N*m] */
} motor_model_t;

/* Parameters for a small 24 V actuator; shared by tests and the simulator. */
motor_params_t motor_default_params(void);

void motor_model_init(motor_model_t *m, const motor_params_t *p);
/* Advance by dt seconds with voltage v applied (internally sub-stepped). */
void motor_model_step(motor_model_t *m, float v, float dt);
int32_t motor_model_encoder(const motor_model_t *m);

#endif
