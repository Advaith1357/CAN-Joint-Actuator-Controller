# Setup

## Host build (unit tests, simulator, HIL tests)
You only need a C compiler, `make` and Python 3. No packages to install.

```sh
make test hil
```

## Cortex-M4F / STM32 build

### Toolchain
Use the Arm GNU Toolchain (`arm-none-eabi`) **15.3.rel2**.

- **macOS (Apple silicon):** use the official tarball from Arm, which needs no admin rights. This is how the dev machine is set up:

  ```sh
  B="https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/15.3.rel2"
  F=arm-gnu-toolchain-15.3.rel2-darwin-arm64-arm-none-eabi.tar.xz
  curl -fLO "$B/$F" && curl -fLO "$B/$F.sha256asc" && shasum -a 256 -c "$F.sha256asc"
  mkdir -p ~/Developer/tools && tar -xJf "$F" -C ~/Developer/tools
  export PATH="$HOME/Developer/tools/arm-gnu-toolchain-15.3.rel2-darwin-arm64-arm-none-eabi/bin:$PATH"
  ```

  To keep the toolchain on your PATH, add the `export` line to `~/.zshrc`. `brew install --cask gcc-arm-embedded` installs the same toolchain system-wide, but it needs `sudo`.

- **Ubuntu / CI:** `sudo apt-get install gcc-arm-none-eabi`. Any recent version works for `arm-check`.

### Third-party sources
`tools/fetch_deps.sh` clones the following into `third_party/` (which is gitignored), and checks that each checkout is at the exact pinned commit:

| Dependency | Version | Used for |
|---|---|---|
| FreeRTOS-Kernel | V11.1.0 | scheduler, ARM_CM4F port, `heap_4` |
| CMSIS_5 (`CMSIS/Core/Include` only) | 5.9.0 | `core_cm4.h`, NVIC/SysTick access |
| cmsis_device_f4 | v2.6.10 | `stm32f446xx.h` register definitions, startup code |

The port is written register-level against CMSIS, with no ST HAL. That keeps the drivers small and makes every peripheral setting explicit.

### Build
```sh
./tools/fetch_deps.sh
make arm-check   # portable code only, plus the no-double check
make stm32       # build/stm32/firmware-hw.elf and firmware-sim.elf
```

## Renode (emulated STM32)
Renode **1.17.0** from the GitHub release (no admin rights needed):

- **macOS (Apple silicon):** `renode-1.17.0.osx-arm64-portable.dmg`. Open it and copy `Renode.app` somewhere, e.g. `~/Developer/tools/`. The binary is `Renode.app/Contents/MacOS/renode`.
- **Linux / CI:** `renode-1.17.0.linux-portable.tar.gz` (see `.github/workflows/ci.yml`).

```sh
make renode-hil RENODE=~/Developer/tools/Renode.app/Contents/MacOS/renode
```

Each test boots a fresh emulator (about 5 s), so the suite takes about a minute. The firmware's UART output for each run goes to `build/stm32/uart-<port>.log`.

To poke at it interactively:
```sh
~/Developer/tools/Renode.app/Contents/MacOS/renode --console \
  -e '$elf=@'"$PWD"'/build/stm32/firmware-sim.elf' \
  -e '$uart_log=@'"$PWD"'/build/stm32/uart.log' \
  -e "include @$PWD/renode/actuator.resc"
```
Then try `sysbus.cpu PC`, `pause`, or `sysbus.can1` commands in the monitor.

`arm-check` fails if any `double` math sneaks into target code. The M4F FPU is single-precision only, so a double operation would be emulated in software.
