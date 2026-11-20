/* Linker script for the STM32F446RE (512 KB flash, 128 KB SRAM).
 *
 * Flash holds the vector table, code, constants and the initial values of
 * .data. At reset, startup_stm32f446xx.s copies .data from flash (_sidata) to
 * RAM (_sdata.._edata) and zeroes .bss (_sbss.._ebss) before calling main. */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Initial main stack pointer: top of RAM (the stack grows down). This stack is
 * used by startup code, main() before the scheduler, and every ISR (MSP).
 * Each FreeRTOS task has its own stack from the FreeRTOS heap (PSP). */
_estack = ORIGIN(RAM) + LENGTH(RAM);
_min_stack_size = 0x800; /* 2 KB reserved for MSP; the link fails if RAM can't fit it */

SECTIONS
{
    /* The core reads word 0 (initial SP) and word 1 (reset vector) from
     * 0x08000000 at reset, so the vector table must be first. KEEP stops
     * --gc-sections from discarding it, since nothing references it. */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    /* Constructor tables walked by __libc_init_array (called from startup). */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH
    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH
    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /* Initialized data: runs from RAM (VMA) but is stored in flash (LMA). */
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT> FLASH

    /* Zero-initialized data, cleared by the startup code. The FreeRTOS heap
     * (ucHeap, configTOTAL_HEAP_SIZE) lives here. */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    /* Fail the link if .data + .bss leave less than _min_stack_size for MSP. */
    ._stack_check (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _min_stack_size;
        . = ALIGN(8);
    } > RAM

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
