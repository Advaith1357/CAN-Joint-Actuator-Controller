# Host build: unit tests + simulator. The STM32 build lives in port/stm32 (coming next).
CC      ?= cc
BUILD   ?= build
OPT     ?= -O2 -g
CFLAGS  := $(OPT) -std=gnu11 -Wall -Wextra -Wshadow -Wdouble-promotion -Werror
CFLAGS  += -Icore/include -Iapp -Iport/host
LDLIBS  += -lm -lpthread

CORE_SRC := core/src/pid.c core/src/can_proto.c core/src/controller.c core/src/motor_model.c
TEST_SRC := $(wildcard tests/unit/*.c)
SIM_SRC  := app/app.c port/host/osal_posix.c port/host/hal_sim.c port/host/main.c

.PHONY: all test sim hil sanitize tsan-hil clean

all: $(BUILD)/unit_tests $(BUILD)/actuator_sim

$(BUILD)/unit_tests: $(CORE_SRC) $(TEST_SRC) $(wildcard core/include/*.h tests/unit/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -Itests/unit $(CORE_SRC) $(TEST_SRC) -o $@ $(LDLIBS)

$(BUILD)/actuator_sim: $(CORE_SRC) $(SIM_SRC) $(wildcard core/include/*.h app/*.h port/host/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CORE_SRC) $(SIM_SRC) -o $@ $(LDLIBS)

test: $(BUILD)/unit_tests
	./$(BUILD)/unit_tests

sim: $(BUILD)/actuator_sim

# Hardware-in-the-loop style tests against the simulator over UDP-encapsulated CAN.
hil: $(BUILD)/actuator_sim
	ACTUATOR_SIM=$(BUILD)/actuator_sim python3 -m unittest discover -s tools/harness -p 'test_*.py' -v

# Unit tests + simulator under AddressSanitizer / UBSan (separate build dir).
sanitize:
	$(MAKE) BUILD=build/san OPT="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all" test

# HIL tests against a ThreadSanitizer build of the simulator. Any data race
# aborts the sim, which fails the tests.
tsan-hil:
	$(MAKE) BUILD=build/tsan OPT="-O1 -g -fsanitize=thread" sim
	ACTUATOR_SIM=build/tsan/actuator_sim TSAN_OPTIONS="halt_on_error=1" python3 -m unittest discover -s tools/harness -p 'test_*.py'

clean:
	rm -rf build

# --- Cortex-M4F cross-compile check ------------------------------------------
# Compiles the target-portable code (core + app, not the motor model) for the
# STM32F446's Cortex-M4F and fails if any double-precision soft-float helper
# (__aeabi_d*) sneaks in. Needs arm-none-eabi-gcc on PATH (see docs/setup.md).
ARM_PREFIX ?= arm-none-eabi-
ARM_FLAGS  := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
              -Os -g -ffunction-sections -fdata-sections -fsingle-precision-constant \
              -std=gnu11 -Wall -Wextra -Wshadow -Wdouble-promotion -Werror \
              -Icore/include -Iapp
ARM_SRC    := core/src/pid.c core/src/can_proto.c core/src/controller.c app/app.c
ARM_OBJ    := $(patsubst %.c,build/arm/%.o,$(ARM_SRC))

build/arm/%.o: %.c
	@mkdir -p $(dir $@)
	$(ARM_PREFIX)gcc $(ARM_FLAGS) -c $< -o $@

.PHONY: arm-check
arm-check: $(ARM_OBJ)
	@$(ARM_PREFIX)size -t $(ARM_OBJ)
	@if $(ARM_PREFIX)nm -u $(ARM_OBJ) | grep -E '__aeabi_d|__aeabi_[fi]2d'; then \
		echo "error: double-precision soft-float in target code"; exit 1; fi
	@echo "arm-check: OK (no double-precision soft-float calls)"

# --- STM32F446 firmware images -----------------------------------------------
# firmware-hw.elf : real motor I/O (TIM2 encoder, TIM1 PWM, ADC1 current sense)
# firmware-sim.elf: identical except the motor is the on-chip motor model (runs in Renode)
# Needs arm-none-eabi-gcc on PATH and ./tools/fetch_deps.sh (see docs/setup.md).
TP         := third_party
RTOS       := $(TP)/FreeRTOS-Kernel
STM32_INC  := -Iport/stm32 -Icore/include -Iapp -I$(RTOS)/include -I$(RTOS)/portable/GCC/ARM_CM4F \
              -I$(TP)/CMSIS_5/CMSIS/Core/Include -I$(TP)/cmsis_device_f4/Include
STM32_CPU  := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
STM32_BASE := $(STM32_CPU) -Os -g3 -ffunction-sections -fdata-sections -fsingle-precision-constant \
              -std=gnu11 -DSTM32F446xx $(STM32_INC)
# Strict warnings for our code; vendor code is built with relaxed defaults.
STM32_OURS := $(STM32_BASE) -Wall -Wextra -Wshadow -Wdouble-promotion -Werror
STM32_LD   := $(STM32_CPU) -T port/stm32/stm32f446re.ld -nostartfiles --specs=nano.specs --specs=nosys.specs \
              -Wl,--gc-sections -Wl,--print-memory-usage
STM32_HDRS := $(wildcard port/stm32/*.h app/*.h core/include/*.h)

STM32_COMMON := core/src/pid.c core/src/can_proto.c core/src/controller.c app/app.c \
                port/stm32/main.c port/stm32/clock.c port/stm32/console.c port/stm32/bxcan.c \
                port/stm32/hal_stm32.c port/stm32/osal_freertos.c
STM32_VENDOR := $(RTOS)/tasks.c $(RTOS)/queue.c $(RTOS)/list.c $(RTOS)/portable/GCC/ARM_CM4F/port.c \
                $(RTOS)/portable/MemMang/heap_4.c $(TP)/cmsis_device_f4/Source/Templates/system_stm32f4xx.c
STM32_STARTUP := $(TP)/cmsis_device_f4/Source/Templates/gcc/startup_stm32f446xx.s

HW_OBJ     := $(patsubst %.c,build/stm32/hw/%.o,$(STM32_COMMON) port/stm32/motor_io_hw.c)
SIM_OBJ    := $(patsubst %.c,build/stm32/sim/%.o,$(STM32_COMMON) port/stm32/motor_io_sim.c core/src/motor_model.c)
VENDOR_OBJ := $(patsubst %.c,build/stm32/vendor/%.o,$(STM32_VENDOR)) build/stm32/vendor/startup.o

build/stm32/hw/%.o: %.c $(STM32_HDRS)
	@mkdir -p $(dir $@)
	$(ARM_PREFIX)gcc $(STM32_OURS) -DFW_VARIANT='"hw"' -c $< -o $@

build/stm32/sim/%.o: %.c $(STM32_HDRS)
	@mkdir -p $(dir $@)
	$(ARM_PREFIX)gcc $(STM32_OURS) -DFW_VARIANT='"sim"' -c $< -o $@

build/stm32/vendor/%.o: %.c port/stm32/FreeRTOSConfig.h port/stm32/board.h
	@mkdir -p $(dir $@)
	$(ARM_PREFIX)gcc $(STM32_BASE) -c $< -o $@

build/stm32/vendor/startup.o: $(STM32_STARTUP)
	@mkdir -p $(dir $@)
	$(ARM_PREFIX)gcc $(STM32_CPU) -x assembler-with-cpp -c $< -o $@

build/stm32/firmware-hw.elf: $(HW_OBJ) $(VENDOR_OBJ) port/stm32/stm32f446re.ld
	$(ARM_PREFIX)gcc $(STM32_LD) -Wl,-Map=$(@:.elf=.map) $(HW_OBJ) $(VENDOR_OBJ) -lm -o $@
	$(ARM_PREFIX)size $@

build/stm32/firmware-sim.elf: $(SIM_OBJ) $(VENDOR_OBJ) port/stm32/stm32f446re.ld
	$(ARM_PREFIX)gcc $(STM32_LD) -Wl,-Map=$(@:.elf=.map) $(SIM_OBJ) $(VENDOR_OBJ) -lm -o $@
	$(ARM_PREFIX)size $@

$(STM32_VENDOR) $(STM32_STARTUP):
	@echo "missing $@: run ./tools/fetch_deps.sh first"; exit 1

.PHONY: stm32
stm32: build/stm32/firmware-hw.elf build/stm32/firmware-sim.elf

# The same black-box tests as `make hil`, against the real firmware image
# running on a simulated STM32F446 in Renode. Set RENODE to the renode binary.
RENODE ?= renode
.PHONY: renode-hil
renode-hil: build/stm32/firmware-sim.elf
	ACTUATOR_BACKEND=renode RENODE=$(RENODE) python3 -m unittest discover -s tools/harness -p 'test_*.py' -v
