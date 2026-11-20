#ifndef HAL_SIM_H
#define HAL_SIM_H

#include <stdint.h>

#include "motor_model.h"

/* Simulator-only CAN ID: set the external load torque (i32, 1e-4 N*m). */
#define SIM_ID_SET_LOAD 0x7F0u

/* CAN frames travel as 16-byte UDP datagrams on localhost:
 *   u32 id (LE) | u8 dlc | 3 bytes pad | 8 bytes data */
#define SIM_UDP_FRAME_LEN 16

int hal_sim_init(uint16_t rx_port, uint16_t tx_port, const motor_params_t *motor);

#endif
