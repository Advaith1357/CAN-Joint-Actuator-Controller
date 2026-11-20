# CAN protocol

CAN 2.0A (11-bit IDs), classic 8-byte frames, all multi-byte fields **little-endian**.
Each actuator has a node ID `N` in `0x0..0xF`.

| ID | Direction | Name | DLC | Payload |
|----|-----------|------|-----|---------|
| `0x001` | host → all | E-stop | any | ignored. Every node latches `FAULT` with the `ESTOP` flag |
| `0x080 + N` | node → host | Status (10 Hz) | 8 | `u8 state` · `u8 faults` · `u8 mode` · `u8 last_seq` · `u32 uptime_ms` |
| `0x180 + N` | node → host | Telemetry (100 Hz) | 8 | `i32 position [1e-4 rad]` · `i16 velocity [0.02 rad/s]` · `i16 current [mA]` |
| `0x200 + N` | host → node | Command | 8 | `u8 mode` · `u8 flags` · `u8 seq` · `u8 reserved` · `i32 setpoint` |
| `0x7F0` | host → sim | Sim: set load torque | 4 | `i32 torque [1e-4 N·m]`. **Host simulator only**, real firmware ignores it |

## Command

`mode`: `0` IDLE (motor off) · `1` VELOCITY (`setpoint` in 1e-3 rad/s) · `2` POSITION (`setpoint` in 1e-4 rad).
`flags` bit 0 `CLEAR_FAULT` is only honoured together with `mode = IDLE`, so a node can't clear a fault and start moving in the same frame.
`seq` is echoed back in `last_seq` on the status frame, which lets the host see which command was last applied.

Commands are a **heartbeat**: while enabled, a node that hears no command for 100 ms latches `FAULT/TIMEOUT` and stops driving the motor. The host must keep re-sending its setpoint (the harness sends at 50 Hz).

## Status

`state`: `0` DISABLED · `1` ENABLED · `2` FAULT.
`faults` bitmask: `0x01` TIMEOUT · `0x02` OVERCURRENT · `0x04` ESTOP.
Faults are latched. The only way out is a command with `mode = IDLE` and `CLEAR_FAULT`.

## Malformed frames

A command with the wrong DLC or an unknown mode is dropped and counted. It doesn't reset the command watchdog, so a node receiving only garbage still times out safely.
