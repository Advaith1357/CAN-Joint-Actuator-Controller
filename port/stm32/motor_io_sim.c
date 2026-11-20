/* Simulated motor I/O: the same motor model as the host simulator, running on
 * the MCU. Used to run the real firmware image in Renode (or on a bare Nucleo)
 * without a motor attached. Everything else, including RTOS, CAN driver,
 * interrupts and clocks, is the production code. */
#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
#include "motor_io.h"
#include "motor_model.h"

#define SIM_ID_SET_LOAD 0x7F0u

static motor_model_t motor;

int motor_io_init(void)
{
    const motor_params_t p = motor_default_params();
    motor_model_init(&motor, &p);
    return 0;
}

int32_t motor_io_encoder(void)
{
    return motor_model_encoder(&motor);
}

float motor_io_current(void)
{
    return motor.i;
}

void motor_io_set_voltage(float volts)
{
    /* The plant advances exactly one control period per call, like hal_sim.c. */
    taskENTER_CRITICAL();
    motor_model_step(&motor, volts, (float)APP_CONTROL_PERIOD_MS * 1e-3f);
    taskEXIT_CRITICAL();
}

void motor_io_on_frame(const can_frame_t *f)
{
    if (f->id != SIM_ID_SET_LOAD || f->dlc < 4) return;
    const int32_t raw = (int32_t)((uint32_t)f->data[0] | ((uint32_t)f->data[1] << 8) |
                                  ((uint32_t)f->data[2] << 16) | ((uint32_t)f->data[3] << 24));
    /* Called from the can_rx task while the control task steps the model. */
    taskENTER_CRITICAL();
    motor.load_torque = (float)raw * 1e-4f;
    taskEXIT_CRITICAL();
}

uint8_t motor_io_extra_can_ids(const uint16_t **ids)
{
    static const uint16_t extra[] = {SIM_ID_SET_LOAD};
    *ids = extra;
    return 1;
}
