/*
******************************************************************************
**
** STM32G473RETx FLASH Linker Script (Corrected)
** FLASH = 512 KB @ 0x08000000
** SRAM1 = 80 KB  @ 0x20000000 – 0x20013FFF
** SRAM2 = 16 KB  @ 0x20014000 – 0x20017FFF   <-- MSP lives here
** CCM SRAM = 32 KB @ 0x10000000 – 0x10007FFF (unused)
**
******************************************************************************
*/

ENTRY(Reset_Handler)

/* ============================================================
   Stack in SRAM2 (16 KB)
   ============================================================ */
_estack = ORIGIN(SRAM2) + LENGTH(SRAM2);  /* Top of SRAM2 for MSP */

_Min_Heap_Size = 0x200;   /* 512 bytes heap  */
_Min_Stack_Size = 0x8000;  /* 16 KB MSP stack (fits in SRAM2) */

/* ============================================================
   Memory Regions
   ============================================================ */
MEMORY
{
    SRAM1 (xrw)  : ORIGIN = 0x20000000, LENGTH = 80K     /* .data, .bss, PSP stacks */
    SRAM2 (xrw)  : ORIGIN = 0x20014000, LENGTH = 16K     /* MSP */
    FLASH (rx)   : ORIGIN = 0x08000000, LENGTH = 512K
}

/* ============================================================
   Sections
   ============================================================ */
SECTIONS
{
  /* Interrupt vector table ----------------------------------*/
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Code + constants ----------------------------------------*/
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM (READONLY) :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  /* Init-array sections -------------------------------------*/
  .preinit_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* ============================================================
     .data → SRAM1
     ============================================================ */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >SRAM1 AT> FLASH

  /* ============================================================
     .bss → SRAM1
     ============================================================ */
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

  /* ============================================================
     Heap + PSP safety region → SRAM1
     (FreeRTOS tasks allocate from SRAM1)
     ============================================================ */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);

    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;   /* PSP stack safety margin */
    . = ALIGN(8);
  } >SRAM1

  /* ============================================================
     Discard unused library sections
     ============================================================ */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

