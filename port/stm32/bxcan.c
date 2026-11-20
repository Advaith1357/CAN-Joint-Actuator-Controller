/* bxCAN driver.
 *
 * RX path: hardware filter -> 3-deep hardware FIFO0 -> CAN1_RX0 interrupt ->
 * FreeRTOS queue -> can_rx task. The ISR only copies the frame out and frees
 * the hardware slot; all decoding happens in task context.
 *
 * TX path: the telemetry task writes into one of three hardware mailboxes;
 * the peripheral arbitrates and retransmits on its own. */
#include "bxcan.h"

#include "board.h"
#include "FreeRTOS.h"
#include "queue.h"

#define RX_QUEUE_LEN    16
#define TIMEOUT_LOOPS   1000000u

/* Bit timing: one bit = 1 (sync) + BS1 + BS2 time quanta (tq).
 * At 45 MHz APB1 and 500 kbit/s: 45e6 / 500e3 = 90 = prescaler 5 x 18 tq.
 * Sample point = (1 + BS1) / 18 = 88.9 %, close to the CANopen-recommended 87.5 %. */
#define TQ_PER_BIT 18u
#define BS1        15u
#define BS2        2u
#define SJW        1u

static QueueHandle_t rx_queue;
static volatile uint32_t rx_overruns;

static int wait_msr(uint32_t mask, uint32_t want)
{
    for (uint32_t i = 0; i < TIMEOUT_LOOPS; i++) {
        if ((CAN1->MSR & mask) == want) return 0;
    }
    return -1;
}

static void setup_filters(const uint16_t *ids, uint8_t n)
{
    /* 16-bit identifier-list mode: each filter bank holds four exact 11-bit IDs.
     * 16-bit filter layout: STDID[10:0] in bits 15:5, then RTR, IDE, EXID[17:15]. */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R = 0;

    const uint8_t banks = (uint8_t)((n + 3u) / 4u);
    for (uint8_t b = 0; b < banks; b++) {
        uint32_t slot[4];
        for (uint8_t k = 0; k < 4; k++) {
            const uint8_t idx = (uint8_t)(b * 4u + k);
            /* Unused slots repeat the first ID: harmless, and never accepts extra IDs. */
            slot[k] = (uint32_t)(idx < n ? ids[idx] : ids[0]) << 5;
        }
        CAN1->sFilterRegister[b].FR1 = slot[0] | (slot[1] << 16);
        CAN1->sFilterRegister[b].FR2 = slot[2] | (slot[3] << 16);
        CAN1->FM1R |= (1u << b);   /* list mode */
        CAN1->FS1R &= ~(1u << b);  /* 16-bit scale */
        CAN1->FFA1R &= ~(1u << b); /* -> FIFO0 */
    }
    CAN1->FA1R = (1u << banks) - 1u;
    CAN1->FMR &= ~CAN_FMR_FINIT;
}

int bxcan_init(const bxcan_config_t *cfg)
{
    if (!cfg->accept_ids || cfg->n_accept_ids == 0 || cfg->n_accept_ids > BXCAN_MAX_ACCEPT_IDS) return BXCAN_ERR_ARGS;
    if (cfg->bitrate == 0 || cfg->pclk_hz % (cfg->bitrate * TQ_PER_BIT) != 0) return BXCAN_ERR_BITRATE;
    const uint32_t prescaler = cfg->pclk_hz / (cfg->bitrate * TQ_PER_BIT);
    if (prescaler == 0 || prescaler > 1024) return BXCAN_ERR_BITRATE;

    rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(can_frame_t));
    if (!rx_queue) return BXCAN_ERR_NOMEM;

    /* Clocks and pins: PA11 = CAN1_RX, PA12 = CAN1_TX, alternate function 9. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    (void)RCC->APB1ENR;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12)) |
                   (2u << GPIO_MODER_MODER11_Pos) | (2u << GPIO_MODER_MODER12_Pos);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12)) |
                    (9u << GPIO_AFRH_AFSEL11_Pos) | (9u << GPIO_AFRH_AFSEL12_Pos);
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED12; /* fast edges on TX */

    /* Leave sleep (the reset state) and enter initialization mode. */
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    CAN1->MCR |= CAN_MCR_INRQ;
    if (wait_msr(CAN_MSR_INAK | CAN_MSR_SLAK, CAN_MSR_INAK)) return BXCAN_ERR_INIT_ENTER;

    /* ABOM: recover from bus-off automatically after 128x11 recessive bits.
     * TXFP: send mailboxes in request order rather than by ID, so telemetry
     * and status go out in the order they were written. */
    CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_TXFP;
    CAN1->BTR = ((SJW - 1u) << CAN_BTR_SJW_Pos) |
                ((BS2 - 1u) << CAN_BTR_TS2_Pos) |
                ((BS1 - 1u) << CAN_BTR_TS1_Pos) |
                ((prescaler - 1u) << CAN_BTR_BRP_Pos) |
                (cfg->loopback ? CAN_BTR_LBKM : 0u);

    setup_filters(cfg->accept_ids, cfg->n_accept_ids);

    /* Interrupt on "FIFO0 message pending". Priority must be numerically >=
     * configMAX_SYSCALL_INTERRUPT_PRIORITY because the ISR uses a FromISR API. */
    CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_FOVIE0;
    NVIC_SetPriority(CAN1_RX0_IRQn, BOARD_IRQ_PRIO_CAN_RX);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    /* Leave init mode. On real hardware this completes after 11 recessive
     * bits on the bus, so it times out without a transceiver (unless loopback). */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    if (wait_msr(CAN_MSR_INAK, 0)) return BXCAN_ERR_INIT_LEAVE;
    return BXCAN_OK;
}

int bxcan_send(const can_frame_t *f)
{
    const uint32_t tsr = CAN1->TSR;
    uint32_t mb;
    if (tsr & CAN_TSR_TME0) mb = 0;
    else if (tsr & CAN_TSR_TME1) mb = 1;
    else if (tsr & CAN_TSR_TME2) mb = 2;
    else return -1;

    CAN_TxMailBox_TypeDef *m = &CAN1->sTxMailBox[mb];
    m->TDTR = f->dlc & 0xFu;
    m->TDLR = (uint32_t)f->data[0] | ((uint32_t)f->data[1] << 8) | ((uint32_t)f->data[2] << 16) | ((uint32_t)f->data[3] << 24);
    m->TDHR = (uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8) | ((uint32_t)f->data[6] << 16) | ((uint32_t)f->data[7] << 24);
    /* Writing TXRQ last hands the mailbox to the hardware. */
    m->TIR = ((f->id & 0x7FFu) << CAN_TI0R_STID_Pos) | CAN_TI0R_TXRQ;
    return 0;
}

int bxcan_recv(can_frame_t *f, uint32_t timeout_ms)
{
    return xQueueReceive(rx_queue, f, pdMS_TO_TICKS(timeout_ms)) == pdPASS ? 0 : 1;
}

uint32_t bxcan_rx_overruns(void)
{
    return rx_overruns;
}

void CAN1_RX0_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;

    while (CAN1->RF0R & CAN_RF0R_FMP0) {
        const CAN_FIFOMailBox_TypeDef *m = &CAN1->sFIFOMailBox[0];
        const uint32_t rir = m->RIR;
        can_frame_t f;
        f.id = (rir >> CAN_RI0R_STID_Pos) & 0x7FFu;
        f.dlc = (uint8_t)(m->RDTR & 0xFu);
        if (f.dlc > 8) f.dlc = 8;
        const uint32_t lo = m->RDLR, hi = m->RDHR;
        for (int i = 0; i < 4; i++) {
            f.data[i] = (uint8_t)(lo >> (8 * i));
            f.data[4 + i] = (uint8_t)(hi >> (8 * i));
        }
        CAN1->RF0R = CAN_RF0R_RFOM0; /* release the FIFO slot */

        /* The filters only pass standard IDs, but never trust the hardware blindly. */
        if (rir & CAN_RI0R_IDE) continue;
        if (xQueueSendFromISR(rx_queue, &f, &woken) != pdPASS) rx_overruns++;
    }
    if (CAN1->RF0R & CAN_RF0R_FOVR0) {
        CAN1->RF0R = CAN_RF0R_FOVR0; /* write-1-to-clear */
        rx_overruns++;
    }
    /* If the queue send woke the can_rx task, switch to it as soon as the ISR exits. */
    portYIELD_FROM_ISR(woken);
}
