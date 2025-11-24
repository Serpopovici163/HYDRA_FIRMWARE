/*
******************************************************************************
**
** STM32G473RETx RAM Linker Script (Corrected)
** RAM execution mode (debug)
**
** SRAM1 = 80 KB  @ 0x20000000 – 0x20013FFF
** SRAM2 = 16 KB  @ 0x20014000 – 0x20017FFF   <-- MSP stack
** FLASH  = 512 KB @ 0x08000000 – 0x0807FFFF
**
******************************************************************************
*/

ENTRY(Reset_Handler)

/* ============================================================
   Stack in SRAM2 (16 KB)
   ============================================================ */
_estack = ORIGIN(SRAM2) + LENGTH(SRAM2);

_Min_Heap_Size  = 0x200;   /* 512 bytes heap */
_Min_Stack_Size = 0x4000;  /* 16 KB MSP stack */

/* ============================================================
   Memory Regions
   ============================================================ */
MEMORY
{
    SRAM1 (xrw) : ORIGIN = 0x20000000, LENGTH = 80K     /* program + data */
    SRAM2 (xrw) : ORIGIN = 0x20014000, LENGTH = 16K     /* main stack */
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K    /* load image only */
}

/* ============================================================
   Sections
   ============================================================ */
SECTIONS
{
  /* Vector table ---------------------------------------------*/
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >SRAM1

  /* Code ------------------------------------------------------*/
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    *(.RamFunc)
    *(.RamFunc*)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >SRAM1

  /* Read-only data -------------------------------------------*/
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >SRAM1

  .ARM.extab (READONLY) :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >SRAM1

  .ARM (READONLY) :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >SRAM1

  /* Init arrays ----------------------------------------------*/
  .preinit_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } >SRAM1

  .init_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } >SRAM1

  .fini_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } >SRAM1

  /* Data ------------------------------------------------------*/
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >SRAM1 AT> FLASH

  /* BSS -------------------------------------------------------*/
  .bss :
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
  } >SRAM1

  /* Heap + PSP region ----------------------------------------*/
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);

    . = . + _Min_Heap_Size;   /* heap */
    . = . + _Min_Stack_Size;  /* PSP safety */
    . = ALIGN(8);
  } >SRAM1

  /* Remove unused libs ----------------------------------------*/
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
