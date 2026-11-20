#ifndef CLOCK_H
#define CLOCK_H

enum {
    CLOCK_OK = 0,
    CLOCK_ERR_HSI = -1,
    CLOCK_ERR_PLL = -2,
    CLOCK_ERR_OVERDRIVE = -3,
    CLOCK_ERR_FLASH = -4,
    CLOCK_ERR_SWITCH = -5,
};

/* Bring SYSCLK to 180 MHz. On error the chip is left running on HSI. */
int clock_init(void);

#endif
