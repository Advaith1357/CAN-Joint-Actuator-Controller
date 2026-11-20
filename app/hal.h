/* Board interface the app needs. Implemented by port/host (simulated motor,
 * CAN over UDP) and port/stm32 (bxCAN, TIM encoder mode, PWM H-bridge, ADC). */
#ifndef HAL_H
#define HAL_H

#include <stdint.h>

#include "can_proto.h"

/* CAN. send may be called from one task only; recv blocks up to timeout_ms.
 * Both return 0 on success, 1 on timeout, negative on error. */
int hal_can_send(const can_frame_t *f);
int hal_can_recv(can_frame_t *f, uint32_t timeout_ms);

/* Motor I/O, called only from the control task. */
int32_t hal_encoder_count(void);
float hal_current_a(void);
void hal_set_voltage(float volts); /* signed; the port turns this into PWM duty */

/* Frames the app doesn't understand (e.g. simulator controls). */
void hal_on_unhandled_frame(const can_frame_t *f);

#endif
