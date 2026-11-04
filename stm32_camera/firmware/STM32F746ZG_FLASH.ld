/*
 * Linker script for the STM32F746ZG camera node.
 *
 * RAM plan (320 KB total):
 *   DTCM  0x20000000  64 KB  code variables, lwIP heap and pools, C heap, stack
 *   SRAM1 0x20010000 240 KB  JPEG frame buffers (section .frame_buffers, from step 4),
 *                            then the WS2812B DMA buffer (section .led_dma, ~7.5 KB)
 *   SRAM2 0x2004C000  16 KB  Ethernet DMA descriptors + receive buffers (section .eth_dma)
 * The F7's DMA controllers and Ethernet DMA can all reach DTCM, SRAM1 and SRAM2.
 */
ENTRY(Reset_Handler)

_estack = ORIGIN(DTCM) + LENGTH(DTCM);  /* stack grows down from the top of DTCM */
_Min_Heap_Size  = 0x800;                 /* 2 KB C heap (printf) */
_Min_Stack_Size = 0x1000;                /* 4 KB stack */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  DTCM  (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
  SRAM1 (xrw) : ORIGIN = 0x20010000, LENGTH = 240K
  SRAM2 (xrw) : ORIGIN = 0x2004C000, LENGTH = 16K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

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

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCM AT> FLASH

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
  } >DTCM

  /* Reserve heap + stack so the link fails if DTCM is too full. */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCM

  /* Not zeroed at startup (NOLOAD): the code that uses these fills them itself. */
  .frame_buffers (NOLOAD) :
  {
    . = ALIGN(32);
    *(.frame_buffers)
    *(.frame_buffers*)
  } >SRAM1

  .led_dma (NOLOAD) :
  {
    . = ALIGN(32);
    *(.led_dma)
    *(.led_dma*)
  } >SRAM1

  .eth_dma (NOLOAD) :
  {
    . = ALIGN(32);
    *(.eth_dma)
    *(.eth_dma*)
  } >SRAM2

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
