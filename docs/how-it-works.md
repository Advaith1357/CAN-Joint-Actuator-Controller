# How it works, step by step

This follows the order things happen on the chip: power-on → boot → clocks → RTOS → tasks → control → CAN → motor I/O, and then how it's all tested. Each step ends with **Try it** (something to run or break) and **Check yourself** (questions an interviewer could ask).

Keep **RM0390** (STM32F446 reference manual) open while you read. Every register here is in it.

---

## 0. The big picture

```
 core/            pure C: PID, controller + safety, CAN protocol    ← no OS, no hardware
 app/             3 tasks, written against osal.h + hal.h            ← no hardware
 port/host/       osal = pthreads, hal = UDP + motor model           ← laptop simulator
 port/stm32/      osal = FreeRTOS, hal = bxCAN + timers/ADC          ← the real firmware
 renode/          emulated NUCLEO-F446RE + UDP↔CAN bridge            ← runs the real .elf
 tools/harness/   Python "robot main computer" that speaks CAN       ← drives all of the above
```

The key idea is **dependency direction**. `core/` knows nothing about FreeRTOS or STM32. `app/` knows only two small interfaces (`osal.h`, `hal.h`). Everything chip-specific lives in `port/stm32/`. That's why the controller can be unit-tested on a laptop, and why the same Python tests can drive the native simulator or the real firmware image.

There are two STM32 images. `firmware-hw.elf` drives a real motor. `firmware-sim.elf` is identical except `motor_io_sim.c` replaces `motor_io_hw.c` and runs the motor model on the chip. Everything else is the same code: RTOS, CAN driver, clocks, interrupts.

---

## 1. Boot: from reset to `main()`

**Files:** `port/stm32/stm32f446re.ld`, `third_party/cmsis_device_f4/Source/Templates/gcc/startup_stm32f446xx.s`

1. At reset the Cortex-M reads two words from address `0x08000000`, the start of flash:
   - word 0 → the initial stack pointer (`_estack`, the top of RAM),
   - word 1 → the address of `Reset_Handler`.

   That's why the linker script puts `.isr_vector` **first** in flash, inside `KEEP()`. Nothing in the code references the table, so without `KEEP()`, `--gc-sections` would throw it away.
2. `Reset_Handler` (startup file):
   - copies `.data` from flash to RAM, using `_sidata`, `_sdata` and `_edata` from the linker script,
   - zeroes `.bss` (`_sbss`.. `_ebss`),
   - calls `SystemInit()` (FPU access on), then `__libc_init_array()` (constructors), then `main()`.
3. **`.data` is "in RAM but stored in flash."** That's the `> RAM AT> FLASH` line. Its *VMA* (where the code expects it) is RAM; its *LMA* (where its initial bytes are stored) is flash.
4. We link with `-nostartfiles`. Newlib's default `crt0` would pull in `exit()` → stdio cleanup → `malloc` → syscall stubs we don't want. The map file showed exactly that chain. Because of that, `main.c` provides an empty `_init()`.

**Try it:**
- `arm-none-eabi-nm build/stm32/firmware-sim.elf | grep -E " (Reset_Handler|_estack|_sidata)"`
- Open `build/stm32/firmware-sim.map` and find `.isr_vector` at `0x08000000`.
- Remove `KEEP` from `.isr_vector`, rebuild, and run `make renode-hil`. What happens, and why?

**Check yourself:**
- Who copies `.data` into RAM, and how does it know where from?
- Why does the ISR stack (MSP) differ from task stacks (PSP)?
- What would happen if `.bss` weren't zeroed?

---

## 2. Clock tree: 16 MHz → 180 MHz

**File:** `port/stm32/clock.c`

The chip wakes up on HSI, the internal 16 MHz RC oscillator. `clock_init()` builds the PLL and switches to it.

1. **Power first:** voltage scale 1 + **over-drive**. Above 168 MHz the core needs over-drive.
2. **PLL math:**
   - VCO in = 16 MHz / M(8) = 2 MHz,
   - VCO out = 2 × N(180) = 360 MHz,
   - SYSCLK = 360 / P(2) = 180 MHz.
3. **Flash wait states *before* speeding up:** 5 WS at 180 MHz. Flash can't deliver instructions in one cycle at that speed, and switching first means fetching garbage.
4. **Bus prescalers:**
   - APB1 = 45 MHz (CAN, USART2). Timers on APB1 get 90 MHz.
   - APB2 = 90 MHz. Timers on APB2 get 180 MHz.
5. Switch SYSCLK to the PLL, then **wait for `SWS` to confirm** the switch.

Every wait loop has a timeout and returns an error code. Firmware that hangs silently in a `while(!ready)` is undebuggable on a real board.

**Bug Renode caught:** the first version wrote `PLLCFGR` as a whole and left the F446-only `PLLR` field at 0, which is an invalid value (reset value is 2). Renode warned about it, and it's now set explicitly. Lesson: when you write a whole register, every field matters, including the ones you don't use.

**Check yourself:**
- Why set flash latency before raising SYSCLK, but you could do the reverse when slowing down?
- Why is the TIM2 clock 90 MHz when APB1 is 45 MHz?
- Why read a register back right after enabling a peripheral clock?

---

## 3. FreeRTOS and the OS abstraction

**Files:** `port/stm32/FreeRTOSConfig.h`, `port/stm32/osal_freertos.c`, `app/osal.h`

- **Tick = 1 ms** (`configTICK_RATE_HZ 1000`). The OSAL speaks milliseconds, so `osal_freertos.c` has `_Static_assert`s that turn a wrong config into a compile error instead of a timing bug.
- **Three core interrupts** drive the scheduler:
  - **SVC** starts the first task.
  - **SysTick** is the 1 ms tick.
  - **PendSV** does the actual context switch, at the *lowest* priority, so it never preempts a real ISR.

  The startup file's vector table calls them `SVC_Handler`/`PendSV_Handler`/`SysTick_Handler`. FreeRTOS calls them `vPortSVCHandler`/... and three `#define`s bridge the two. If you forget them, the board hangs the moment the scheduler starts. `nm` confirms they point at FreeRTOS's handlers.
- **Interrupt priorities (the classic FreeRTOS bug).** The F446 has 4 priority bits (0 = most urgent).
  - `configMAX_SYSCALL_INTERRUPT_PRIORITY` is 5: FreeRTOS critical sections mask priorities 5–15.
  - The CAN RX ISR calls `xQueueSendFromISR`, so it **must** be at 5–15 (it's 6).
  - An ISR at priority 2 calling a FromISR API could interrupt the kernel mid-update and corrupt its lists.
- **`xTaskDelayUntil` vs `vTaskDelay`:** `DelayUntil` wakes at fixed multiples of the period, so the 1 kHz loop doesn't drift when one iteration runs long.
- **Mutex, not a binary semaphore,** for the telemetry snapshot. A mutex has **priority inheritance**: if the low-priority telemetry task holds it when the 1 kHz control task wants it, telemetry is temporarily boosted (this is how priority inversion is avoided).
- **Safety nets:**
  - `configCHECK_FOR_STACK_OVERFLOW 2` checks a canary on every context switch.
  - The malloc-failed hook catches heap exhaustion.
  - `configASSERT` prints the file and line over UART.
  - `HardFault_Handler` prints `CFSR`/`HFSR`.

**Check yourself:**
- Why does PendSV run at the lowest priority?
- What's priority inversion? Describe the Mars Pathfinder bug in two sentences.
- Why can't `osal_queue_send` be called from an ISR? What would you add to the OSAL to allow it?

---

## 4. Tasks and how they share data

**File:** `app/app.c`

| Task | Priority | Rate | Job |
|---|---|---|---|
| `control` | 3 (highest) | 1 kHz | drain commands → read sensors → `ctl_step` → set voltage → publish snapshot |
| `can_rx` | 2 | on demand | block on CAN, decode, forward commands |
| `telem` | 1 | 100 Hz | copy snapshot, send telemetry; status every 10th frame |
| `heartbeat` | 0 | 2 Hz | blink LD2. If it stops, something higher is hogging the CPU |

Rules that make this safe:
1. **One owner.** Only `control` touches `controller_t`. No lock is needed, because there's no sharing.
2. **Queue for commands.** `can_rx` → `control` is a bounded FIFO.
3. **Atomic flag for the e-stop.** If the queue were full, an e-stop sent through it could be dropped. A separate flag can't be.
4. **Mutex-protected snapshot** for telemetry. The copy is a handful of floats, so the lock is held for microseconds.

**Check yourself:** what goes wrong if `telem` reads `g.ctl.vel` directly without the snapshot? (Hint: a torn read across several fields.)

---

## 5. The controller: PID, cascade, current limit, safety

**Files:** `core/src/pid.c`, `core/src/controller.c`, and the Tuning section in `README.md`

1. **PID details that matter:**
   - **Derivative on measurement**, so a setpoint step doesn't spike the output.
   - **Filtered D term.**
   - **Conditional-integration anti-windup:** stop integrating while saturated *in the direction of the error*.
2. **Cascade:** position error × `pos_kp` → velocity target → velocity PI → volts. The outer loop is about 5× slower than the inner loop so they don't fight.
3. **Speed estimate:**
   - The encoder count difference per 1 ms, then a low-pass filter.
   - The difference uses unsigned subtraction, so a 32-bit counter wrap is harmless (there's a unit test for it).
4. **Current limit without a current loop:** in steady state, `i = (V − ke·ω)/R`. Clamping V to `ke·ω ± R·I_limit` bounds the current. The clamp is fed into the PID's output limits, so anti-windup sees the *real* limit.
   - Peak: 7.7 A against an 8 A limit.
   - An unclamped reversal would draw about 36 A.
5. **Safety state machine:** `DISABLED` → `ENABLED` → `FAULT`.
   - FAULT is **latched**, entered on:
     - a command timeout (100 ms without a command),
     - an e-stop,
     - overcurrent (12 A for 20 ms).
   - Clearing it requires `IDLE` + `CLEAR_FAULT`, so a single frame can't clear a fault *and* start moving.

**Try it:**
- Set `ki = 0` in `ctl_default_config()` and run `make test`. Which test fails, and why? (Steady-state error under load.)
- Set `i_limit = 50` and see which test fails.

---

## 6. CAN: protocol and the bxCAN driver

**Files:** `docs/protocol.md`, `core/src/can_proto.c`, `port/stm32/bxcan.c`

- **Protocol:** command `0x200+N`, telemetry `0x180+N`, status `0x080+N`, e-stop `0x001`.
  - Lower IDs win arbitration, which is why the e-stop is `0x001`.
  - Fields are packed little-endian byte by byte, never by `memcpy`-ing a struct, because struct padding and endianness aren't portable.
- **Bit timing:** 45 MHz / 500 kbit/s = 90 = prescaler 5 × 18 time quanta. 1 + BS1(15) + BS2(2) = 18, so the sample point is 88.9 %.
- **Hardware filters:** 16-bit *list* mode, 4 exact IDs per filter bank. The chip only accepts our command ID, the e-stop, and (in the sim build) the load-injection ID. Other nodes' traffic never costs an interrupt.
- **RX path:** the FIFO0-pending interrupt copies the frame out, releases the hardware slot (`RFOM0`) and does `xQueueSendFromISR`. Decoding happens in the task, never in the ISR. `portYIELD_FROM_ISR` switches straight to `can_rx` if it was woken.
- **TX path:** find an empty mailbox (`TME0..2`), write data, then write `TIR` with `TXRQ` *last*. That's the moment the hardware takes ownership.
- **Init handshake:** leave sleep → request init (`INRQ`) → wait `INAK` → configure → clear `INRQ` → wait `INAK = 0`. On real hardware that last step needs 11 recessive bits on the bus, so it times out without a transceiver unless you're in loopback mode.

**Check yourself:**
- What's the sample point, and why put it late in the bit?
- What does bus-off mean, and what does `ABOM` do?
- Why release the FIFO slot inside the ISR?

---

## 7. Motor I/O: real hardware vs simulation

**Files:** `port/stm32/motor_io_hw.c`, `port/stm32/motor_io_sim.c`

- **Encoder:** TIM2 in encoder mode 3 counts both edges of both channels (×4) in hardware. The CPU never misses a count. Input filters reject glitches.
- **PWM:**
  - TIM1, center-aligned at 20 kHz (ARR = 180 MHz / (2 × 20 kHz) = 4500).
  - Sign-magnitude drive into a DRV8871: PWM on one input, the other held low.
  - Preload registers mean a new duty starts at the next period, never mid-pulse.
  - `MOE` must be set, because advanced timers keep their outputs off otherwise.
- **Current sense:** ADC1 injected conversion triggered by TIM1 CH4 just before the counter valley, which is the *middle* of the on-pulse, where the ripple crosses its average. No CPU involvement.
- **Sim variant:** the same motor model as the laptop simulator, stepped once per control period on the chip, inside a critical section, since the load-injection frame writes it from another task.

**Honest status:** `motor_io_hw.c` compiles and links but hasn't run on a board. Expect to adjust current-sense scaling and controller gains for a real motor.

---

## 8. How it's tested (and what each level proves)

| Level | Command | Runs | Proves |
|---|---|---|---|
| Unit | `make test` | `core/` on the laptop | PID math, protocol codec, controller behavior incl. closed-loop step responses |
| Sanitizers | `make sanitize`, `make tsan-hil` | the same, instrumented | no memory errors, undefined behavior or data races |
| Host HIL | `make hil` | native build, pthreads | task structure + protocol end to end |
| Cross-compile | `make arm-check`, `make stm32` | ARM toolchain | target code builds with `-Werror`, no double math, fits in flash/RAM |
| **Renode HIL** | `make renode-hil` | **the real firmware-sim.elf** on an emulated STM32F4 | boot, clocks, FreeRTOS, bxCAN driver + filters + ISR, the whole firmware, against the same 9 tests |

**How Renode gets CAN to Python:** Renode has a `CANHub`, but its host bridge (SocketCAN) is Linux-only. `renode/UdpCanBridge.cs` is a small Renode plugin (modeled on that bridge) that forwards every frame on the emulated bus to a UDP socket and back, using the same 16-byte format as the laptop simulator. So `tools/harness` doesn't know or care which backend it's talking to.

**Bugs found while building it** (good interview stories):
1. **A test bug, not a firmware bug.** The timeout test measured silence from the wrong moment. The watchdog was correct.
2. **Dependency pinning.** `git ls-remote` returns *annotated tag objects*, not commits. The pin check caught the mismatch.
3. **Unexpected link dependencies.** `crt0` dragged in `exit` → stdio → `malloc`. Fixed with `-nostartfiles` (found by reading the linker map).
4. **Invalid register field.** `PLLR = 0` was invalid, caught by the emulator's register model.
5. **A silent hang.** Renode hung silently on a C# compile error in headless mode. Running it in console mode showed the real error.

---

## Exercises, in order of difficulty
1. Add a `PARAM` message (`0x300+N`) that sets `vel_limit` at runtime, with unit and HIL tests.
2. Report `bxcan_rx_overruns()` and the CAN error counters (`CAN1->ESR`) in a new diagnostics frame.
3. Replace the voltage clamp with a real inner current PI loop and compare step responses.
4. Make the e-stop also work if the `can_rx` task is starved. Hint: handle it in the ISR.
5. Flash `firmware-hw.elf` to a real Nucleo and get `make hil` running over a USB-CAN adapter with python-can.
