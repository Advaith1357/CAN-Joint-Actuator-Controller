/* app/hal.h on the STM32: CAN through bxCAN, motor I/O through motor_io_*. */
#include "hal.h"

#include "bxcan.h"
#include "motor_io.h"

int hal_can_send(const can_frame_t *f)
{
    return bxcan_send(f);
}

int hal_can_recv(can_frame_t *f, uint32_t timeout_ms)
{
    return bxcan_recv(f, timeout_ms);
}

int32_t hal_encoder_count(void)
{
    return motor_io_encoder();
}

float hal_current_a(void)
{
    return motor_io_current();
}

void hal_set_voltage(float volts)
{
    motor_io_set_voltage(volts);
}

void hal_on_unhandled_frame(const can_frame_t *f)
{
    motor_io_on_frame(f);
}
