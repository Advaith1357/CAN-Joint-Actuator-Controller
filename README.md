# joint-actuator-fw

Firmware for a CAN-controlled robot joint actuator on an STM32F446 (Cortex-M4F, FreeRTOS): cascaded position/velocity control of a DC motor, with current limiting and a latched safety state machine. The drivers are register-level, against CMSIS, with no vendor HAL.

The control and protocol code is hardware-independent, so the same code is tested at three levels:

- **Unit tests** cover the PID, the protocol codec and the controller, including closed-loop tests against a motor model.
- **Host HIL tests** drive a native build over CAN (UDP-encapsulated) from a Python harness, with a simulated motor.
- **Renode HIL tests** run the *same* Python tests against the real STM32 firmware image (FreeRTOS, bxCAN driver, interrupts, 180 MHz clock tree) on an emulated STM32F4. CAN reaches the emulator through a small UDP bridge written for Renode ([renode/UdpCanBridge.cs](renode/UdpCanBridge.cs)).
- **Sanitizers** (ASan, UBSan, ThreadSanitizer) run on every push.

Not yet tested on physical hardware. `firmware-hw.elf` (real encoder, PWM and ADC drivers) builds, but only `firmware-sim.elf`, which swaps in the motor model, has been run.

```
make test       # 22 unit tests
make hil        # 9 black-box tests against the simulator
make sanitize   # unit tests under ASan + UBSan
make tsan-hil   # black-box tests against a ThreadSanitizer build
make arm-check  # cross-compile for Cortex-M4F, fail on any double-precision math
make stm32      # firmware-hw.elf + firmware-sim.elf for the NUCLEO-F446RE
make renode-hil RENODE=/path/to/renode   # the 9 HIL tests against firmware-sim.elf in Renode
```

How each piece works, step by step: [docs/how-it-works.md](docs/how-it-works.md).

Toolchain and pinned FreeRTOS/CMSIS setup: [docs/setup.md](docs/setup.md).

## Architecture

```
            CAN bus
               │
     ┌─────────▼─────────┐      queue (cmds)     ┌──────────────────────┐
     │ can_rx task  (p2) │ ───────────────────▶ │ control task (p3)    │
     │ decode, filter    │  atomic flag (e-stop) │ 1 kHz, owns the      │
     └───────────────────┘ ───────────────────▶ │ controller           │
                                                 │ enc+current → V      │
     ┌───────────────────┐  mutex (snapshot)     └──────────┬───────────┘
     │ telem task   (p1) │ ◀───────────────────────────────┘
     │ 100 Hz telemetry  │                        hal_set_voltage()
     │ 10 Hz status      │                                  │
     └─────────┬─────────┘                                  ▼
               ▼ CAN bus                           H-bridge / motor model
```

| Layer | Files | Depends on |
|---|---|---|
| Core logic (pure C, no OS) | `core/`: `pid`, `controller`, `can_proto` | nothing |
| Application tasks | `app/app.c` | `osal.h`, `hal.h`, core |
| Host port | `port/host/`: pthreads OSAL, UDP CAN, `motor_model` | POSIX |
| STM32 port | `port/stm32/`: FreeRTOS OSAL, clock tree, bxCAN driver, linker script; motor I/O as `motor_io_hw.c` (TIM2 encoder, TIM1 PWM, ADC1) or `motor_io_sim.c` (on-chip motor model) | CMSIS, FreeRTOS. 12.6 KB flash, 19 KB RAM (16 KB is the FreeRTOS heap) |
| Renode platform | `renode/`: board description, UDP↔CAN bridge, launch script | Renode 1.17 |

### Design decisions
- **The controller has a single owner.** Only the control task touches `controller_t`, so it needs no locks. Other tasks talk to it through a queue (commands), an atomic flag (e-stop) and a mutex-guarded snapshot (telemetry).
- **An e-stop can't be dropped.** Commands go through a bounded queue, but the e-stop sets an atomic flag the control loop checks every cycle, so a full queue can't swallow it.
- **Commands act as a heartbeat.** If an enabled node hears nothing for 100 ms, it latches `FAULT/TIMEOUT` and stops driving the motor. Malformed frames don't feed the watchdog.
- **Faults are latched.** Clearing a fault requires `mode = IDLE` + `CLEAR_FAULT`, so a single frame can't both clear a fault and start motion.
- **Current is limited without a current sensor loop.** The velocity PID's output limits are set every cycle to `ke·ω ± R·I_limit`, which bounds the winding current. Because the limits go into the PID itself, its anti-windup stays correct. A separate overcurrent trip (12 A for 20 ms) catches hardware faults.
- **Floats are single precision only** (`-Wdouble-promotion -Werror`), because the Cortex-M4F FPU can't do doubles in hardware.

## Protocol
See [docs/protocol.md](docs/protocol.md). Summary: command `0x200+N`, telemetry `0x180+N` (100 Hz), status `0x080+N` (10 Hz), broadcast e-stop `0x001`.

## Tuning
Plant (`motor_default_params`): R = 1 Ω, L = 0.5 mH, kt = ke = 0.05, J = 1e-4 kg·m², 24 V, 4096-count encoder.
From voltage to speed, the motor behaves like a first-order system with gain 1/ke = 20 (rad/s)/V and time constant τ = J·R/(kt·ke) = 40 ms.

- **Velocity PI:** the PI zero cancels the plant pole (ki/kp = 1/τ), and kp = 0.2 sets the crossover near 100 rad/s.
- **Position P:** kp = 20 s⁻¹, about 5× below the velocity loop, so the loops don't fight.
- **Speed estimate:** encoder differencing followed by a first-order low-pass (α = 0.3, about 57 Hz).

Measured in the closed-loop model:

| Step | 10–90 % rise | 2 % settle | Overshoot | Peak current |
|---|---|---|---|---|
| Velocity 0 → 100 rad/s | 37 ms | 107 ms | 0.05 % | 7.7 A (limit 8 A) |
| Velocity 0 → 250 rad/s | 72 ms | 145 ms | 0.06 % | 7.7 A |
| Position 0 → 2 rad | 98 ms | 177 ms | 0.03 % | 7.7 A |
| 0.2 N·m load step at 100 rad/s | — | back within 2 % in ~100 ms (15 % dip) | — | — |

Without the current clamp, a full 250 → −250 rad/s reversal would draw about 36 A.

## Roadmap
- [x] `port/stm32`: NUCLEO-F446RE target. FreeRTOS OSAL, bxCAN driver with hardware filters, TIM2 quadrature encoder, TIM1 PWM to an H-bridge, ADC current sense.
- [x] Run the firmware image under Renode, driven by the same HIL tests (CI job included).
- [ ] Physical hardware: flash `firmware-hw.elf`, swap `UdpBus` for python-can + a USB-CAN adapter, and re-tune the controller for the real motor's R, ke and current-sense range.
- [ ] Inner current loop (PI on measured current) instead of the back-EMF voltage clamp.
- [ ] CAN bus-off / error-passive reporting in the status frame.
