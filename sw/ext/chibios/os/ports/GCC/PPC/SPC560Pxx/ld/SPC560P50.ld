/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

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

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

/*
 * SPC560P44 memory setup.
 */
__irq_stack_size__      = 0x0000; /* Not yet used.*/
__process_stack_size__  = 0x0800;

MEMORY
{
    flash     : org = 0x00000000, len = 512k
    dataflash : org = 0x00800000, len = 64k
    ram       : org = 0x40000000, len = 40k
}

ENTRY(_reset_address)

/*
 * Derived constants.
 */
__flash_size__          = LENGTH(flash);
__flash_start__         = ORIGIN(flash);
__flash_end__           = ORIGIN(flash) + LENGTH(flash);

__ram_size__            = LENGTH(ram);
__ram_start__           = ORIGIN(ram);
__ram_end__             = ORIGIN(ram) + LENGTH(ram);

SECTIONS
{
    . = ORIGIN(flash);
    .boot : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.bam))
        KEEP(*(.coreinit))
        KEEP(*(.handlers))
        KEEP(*(.crt0))
        . = ALIGN(0x00000800);
        KEEP(*(.vectors))
        /* Note, have to waste the first 64KB because the IVPR register
           requires an alignment of 64KB and the first 64KB cannot be used,
           IVOR0 would conflict with the BAM word. Applications could
           allocate code or data in the first 64KB by using special sections.*/
        . = ALIGN(0x00010000);
        __ivpr_base__ = .;
        KEEP(*(.ivors))
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

    .text_vle : ALIGN(16) SUBALIGN(16)
    {
        *(.text_vle)
        *(.text_vle.*)
        *(.gnu.linkonce.t_vle.*)
    } > flash

    .text : ALIGN(16) SUBALIGN(16)
    {
        *(.text)
        *(.text.*)
        *(.gnu.linkonce.t.*)
    } > flash

    .rodata : ALIGN(16) SUBALIGN(16)
    {
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
        *(.rodata)
        *(.rodata.*)
        *(.rodata1)
    } > flash

    .sdata2 : ALIGN(16) SUBALIGN(16)
    {
        __sdata2_start__ = . + 0x8000;
        *(.sdata2)
        *(.sdata2.*)
        *(.gnu.linkonce.s2.*)
        *(.sbss2)
        *(.sbss2.*)
        *(.gnu.linkonce.sb2.*)
    } > flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash

    .romdata : ALIGN(16) SUBALIGN(16)
    {
        __romdata_start__ = .;
    } > flash

    .stacks :
    {
        . = ALIGN(8);
        __irq_stack_base__ = .;
        . += __irq_stack_size__;
        . = ALIGN(8);
        __irq_stack_end__ = .;
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > ram

    .data : AT(__romdata_start__)
    {
        . = ALIGN(4);
        __data_start__ = .;
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        __sdata_start__ = . + 0x8000;
        *(.sdata)
        *(.sdata.*)
        *(.gnu.linkonce.s.*)
        __data_end__ = .;
    } > ram

    .sbss :
    {
        __bss_start__ = .;
        *(.sbss)
        *(.sbss.*)
        *(.gnu.linkonce.sb.*)
        *(.scommon)
    } > ram

    .bss :
    {
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        __bss_end__ = .;
    } > ram

    __heap_base__   = __bss_end__;
    __heap_end__    = __ram_end__;
}
