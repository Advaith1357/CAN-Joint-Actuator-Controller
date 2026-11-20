/* Motor sensing and drive. Two implementations, picked at link time:
 *   motor_io_hw.c   TIM2 quadrature encoder, TIM1 PWM -> DRV8871, ADC1 current sense
 *   motor_io_sim.c  the motor model running on the MCU itself (for Renode / bench tests) */
#ifndef MOTOR_IO_H
#define MOTOR_IO_H

#include <stdint.h>

#include "can_proto.h"

int motor_io_init(void);
int32_t motor_io_encoder(void);
float motor_io_current(void);
void motor_io_set_voltage(float volts);
/* Frames the app didn't handle; the simulator uses one to inject load torque. */
void motor_io_on_frame(const can_frame_t *f);
/* Extra CAN IDs this implementation wants the hardware filter to accept. */
uint8_t motor_io_extra_can_ids(const uint16_t **ids);

#endif
