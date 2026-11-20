#include "console.h"

#include "board.h"

void console_init(uint32_t pclk_hz, uint32_t baud)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    /* PA2 -> alternate function 7 (USART2_TX). */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER2) | (2u << GPIO_MODER_MODER2_Pos);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL2) | (7u << GPIO_AFRL_AFSEL2_Pos);

    /* Oversampling by 16: BRR = fPCLK / baud, rounded (mantissa.fraction in 12.4). */
    USART2->BRR = (pclk_hz + baud / 2u) / baud;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void console_puts(const char *s)
{
    for (; *s; s++) {
        if (*s == '\n') {
            while (!(USART2->SR & USART_SR_TXE)) {
            }
            USART2->DR = '\r';
        }
        while (!(USART2->SR & USART_SR_TXE)) {
        }
        USART2->DR = (uint8_t)*s;
    }
}

void console_put_u32(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v && i > 0);
    console_puts(&buf[i]);
}

void console_put_i32(int32_t v)
{
    if (v < 0) {
        console_puts("-");
        console_put_u32((uint32_t)(-(v + 1)) + 1u); /* safe for INT32_MIN */
    } else {
        console_put_u32((uint32_t)v);
    }
}

void console_put_hex(uint32_t v)
{
    static const char digits[] = "0123456789abcdef";
    char buf[11] = "0x";
    for (int i = 0; i < 8; i++) buf[2 + i] = digits[(v >> (28 - 4 * i)) & 0xFu];
    buf[10] = '\0';
    console_puts(buf);
}
