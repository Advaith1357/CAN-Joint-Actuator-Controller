/* Minimal blocking debug console on USART2 (ST-LINK virtual COM port).
 * Deliberately no printf: newlib's printf pulls in ~20 KB and isn't safe to
 * call from several tasks. Only call from init code or a single task. */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

void console_init(uint32_t pclk_hz, uint32_t baud);
void console_puts(const char *s);
void console_put_u32(uint32_t v);
void console_put_i32(int32_t v);
void console_put_hex(uint32_t v);

#endif
