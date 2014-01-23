/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * ST32F407xG memory setup.
 */

/*
__main_stack_size__ is the size of INTERRUPTS stack
__process_stack_size__ is the stack of the C-runtime
*/
__main_stack_size__     = 0x01000;
__process_stack_size__  = 0x02000;

MEMORY
{
   flash : org = 0x08000000, len = 1M-128k
   eemul :  org = 0x080E0000, len = 128k
    /* ram : org = 0x20000000, len = 112k */
    /* ethram : org = 0x2001C000, len = 16k */
   ram : org = 0x20000000, len = 128k
   ccmram : org = 0x10000000, len = 64k
   bkpsram : org = 0x40024000, len = 4k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = LENGTH(ram);
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    . = 0;
    _text = .;

    startup : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(vectors))
    } > flash

    constructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > flash

    destructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end = .);
    } > flash

    .text : ALIGN(16) SUBALIGN(16)
    {
        *(.text.startup.*)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash

    .textalign : ONLY_IF_RO
    {
        . = ALIGN(8);
    } > flash

    . = ALIGN(4);
    _etext = .;
    _textdata = _etext;

    .stacks :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > ram


    .ccm (NOLOAD):
    {
        . = ALIGN(4);
        *(.ccmram)
        . = ALIGN(4);
    } > ccmram

    .bkp (NOLOAD):
    {
        . = ALIGN(4);
        *(.bkpsram)
        . = ALIGN(4);
    } > bkpsram


   /* .ccm : */
   /*  { */
   /*      PROVIDE(_cmm_start = .); */
   /*      . = ALIGN(4); */
   /*      *(.bss.mainthread.*) */
   /*      . = ALIGN(4); */
   /*      *(.bss._idle_thread_wa) */
   /*      . = ALIGN(4); */
   /*      *(.bss.rlist) */
   /*      . = ALIGN(4); */
   /*      *(.bss.vtlist) */
   /*      . = ALIGN(4); */
   /*      *(.bss.endmem) */
   /*      . = ALIGN(4); */
   /*      *(.bss.nextmem) */
   /*      . = ALIGN(4); */
   /*      *(.bss.default_heap) */
   /*      . = ALIGN(4); */
   /*      PROVIDE(_cmmend = .); */
   /*  } > ccmram */


    .data :
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        . = ALIGN(4);
        *(.data.*)
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > ram AT > flash

    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
    } > ram
}

PROVIDE(end = .);
_end            = .;

__heap_base__   = _end;
__heap_end__    = __ram_end__;
