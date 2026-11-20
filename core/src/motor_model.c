#include "motor_model.h"

#include <math.h>

/* The electrical time constant L/R is 0.5 ms, so a 1 ms control step is
 * sub-stepped with explicit Euler at 50 us (dt / tau = 0.1, stable). */
#define SUBSTEPS 20
#define TWO_PI_F 6.28318530718f

motor_params_t motor_default_params(void)
{
    motor_params_t p = {
        .R = 1.0f,
        .L = 0.5e-3f,
        .kt = 0.05f,
        .ke = 0.05f,
        .J = 1.0e-4f,
        .b = 1.0e-5f,
        .v_supply = 24.0f,
        .counts_per_rev = 4096,
    };
    return p;
}

void motor_model_init(motor_model_t *m, const motor_params_t *p)
{
    m->p = *p;
    m->i = 0.0f;
    m->omega = 0.0f;
    m->theta = 0.0f;
    m->load_torque = 0.0f;
}

void motor_model_step(motor_model_t *m, float v, float dt)
{
    const motor_params_t *p = &m->p;
    if (v > p->v_supply) v = p->v_supply;
    if (v < -p->v_supply) v = -p->v_supply;

    const float h = dt / SUBSTEPS;
    for (int k = 0; k < SUBSTEPS; k++) {
        const float di = (v - p->R * m->i - p->ke * m->omega) / p->L;
        const float dw = (p->kt * m->i - p->b * m->omega - m->load_torque) / p->J;
        m->i += di * h;
        m->omega += dw * h;
        m->theta += m->omega * h;
    }
}

int32_t motor_model_encoder(const motor_model_t *m)
{
    return (int32_t)floorf(m->theta / TWO_PI_F * (float)m->p.counts_per_rev);
}
