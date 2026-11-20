/* STM32F446 entry point: clocks -> console -> motor I/O -> CAN -> app tasks -> scheduler. */
#include <string.h>

#include "app.h"
#include "board.h"
#include "bxcan.h"
#include "clock.h"
#include "console.h"
#include "motor_io.h"
#include "osal.h"

#ifndef FW_VARIANT
#define FW_VARIANT "hw"
#endif

#define CAN_BITRATE 500000u

static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER5) | (1u << GPIO_MODER_MODER5_Pos);
}

/* Lowest-priority task: if the LED stops blinking, something higher is hogging the CPU. */
static void heartbeat_task(void *arg)
{
    (void)arg;
    uint32_t wake = osal_now_ms();
    for (;;) {
        osal_delay_until(&wake, 500);
        GPIOA->ODR ^= GPIO_ODR_OD5;
    }
}

static void fatal(const char *what, int code)
{
    console_puts("FATAL: ");
    console_puts(what);
    console_puts(" ");
    console_put_i32(code);
    console_puts("\n");
    __disable_irq();
    for (;;) {
    }
}

int main(void)
{
    const int clk = clock_init();
    led_init();
    console_init(clk == CLOCK_OK ? BOARD_PCLK1_HZ : 16000000u, 115200u);
    console_puts("\njoint-actuator-fw (" FW_VARIANT ") node ");
    console_put_u32(BOARD_NODE_ID);
    console_puts("\n");
    if (clk != CLOCK_OK) fatal("clock_init", clk);
    console_puts("clock: 180 MHz\n");

    if (motor_io_init() != 0) fatal("motor_io_init", -1);

    /* Hardware filter: our command ID, the broadcast e-stop, plus anything the
     * motor I/O layer needs (the simulator's load-injection frame). */
    uint16_t ids[BXCAN_MAX_ACCEPT_IDS] = {(uint16_t)(CANP_ID_CMD_BASE + BOARD_NODE_ID), CANP_ID_ESTOP};
    uint8_t n = 2;
    const uint16_t *extra;
    const uint8_t n_extra = motor_io_extra_can_ids(&extra);
    for (uint8_t i = 0; i < n_extra && n < BXCAN_MAX_ACCEPT_IDS; i++) ids[n++] = extra[i];

    const bxcan_config_t can_cfg = {
        .pclk_hz = BOARD_PCLK1_HZ,
        .bitrate = CAN_BITRATE,
        .loopback = 0,
        .accept_ids = ids,
        .n_accept_ids = n,
    };
    const int can_rc = bxcan_init(&can_cfg);
    if (can_rc != BXCAN_OK) fatal("bxcan_init", can_rc);
    console_puts("can: 500 kbit/s\n");

    app_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.node_id = BOARD_NODE_ID;
    cfg.ctl = ctl_default_config();
    if (app_init(&cfg) != 0) fatal("app_init", -1);
    if (osal_task_create(heartbeat_task, NULL, "heartbeat", 0, 128) != 0) fatal("heartbeat", -1);

    console_puts("starting scheduler\n");
    osal_start();
    return 0;
}

/* We link with -nostartfiles (our own Reset_Handler, no newlib crt0, which
 * would drag in exit() -> stdio -> malloc). __libc_init_array still calls
 * _init, normally provided by crti.o; there's nothing for it to do here. */
void _init(void);
void _init(void)
{
}

void board_assert_failed(const char *file, int line)
{
    __disable_irq();
    console_puts("ASSERT: ");
    console_puts(file);
    console_puts(":");
    console_put_i32(line);
    console_puts("\n");
    for (;;) {
    }
}

void HardFault_Handler(void)
{
    /* CFSR says *why* (bad address, unaligned, divide by zero, ...);
     * HFSR says whether it escalated from another fault. */
    console_puts("HardFault CFSR=");
    console_put_hex(SCB->CFSR);
    console_puts(" HFSR=");
    console_put_hex(SCB->HFSR);
    console_puts("\n");
    for (;;) {
    }
}
