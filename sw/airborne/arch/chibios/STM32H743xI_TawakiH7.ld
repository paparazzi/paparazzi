/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * STM32H743xI generic setup.
 * 
 * AXI SRAM     - BSS, Data, Heap.
 * SRAM1+SRAM2  - None.
 * SRAM3        - NOCACHE, ETH.
 * SRAM4        - None.
 * DTCM-RAM     - Main Stack, Process Stack.
 * ITCM-RAM     - None.
 * BCKP SRAM    - None.
 */
MEMORY
{
    flash0  (rx) : org = 0x08000000, len = 2M        /* Flash bank1+bank2 */
    flash1  (rx) : org = 0x08000000, len = 1M        /* Flash bank 1 */
    flash2  (rx) : org = 0x08100000, len = 1M        /* Flash bank 2 */
    flash3  (rx) : org = 0x00000000, len = 0
    flash4  (rx) : org = 0x00000000, len = 0
    flash5  (rx) : org = 0x00000000, len = 0
    flash6  (rx) : org = 0x00000000, len = 0
    flash7  (rx) : org = 0x00000000, len = 0
    ram0nc  (wx) : org = 0x24000000, len = 128k       /* AXI SRAM NON CACHED */
    ram0    (wx) : org = 0x24020000, len = 384k      /* AXI SRAM */
    ram1    (wx) : org = 0x30000000, len = 256k      /* AHB SRAM1+SRAM2 */
    ram2    (wx) : org = 0x30000000, len = 288k      /* AHB SRAM1+SRAM2+SRAM3 */
    ram3    (wx) : org = 0x30040000, len = 32k       /* AHB SRAM3 NON CACHED*/
    ram4    (wx) : org = 0x38000000, len = 64k       /* AHB SRAM4 NON CACHED*/
    ram5    (wx) : org = 0x20000000, len = 128k      /* DTCM-RAM */
    ram6    (wx) : org = 0x00000000, len = 64k       /* ITCM-RAM */
    ram7    (wx) : org = 0x38800000, len = 4k        /* BCKP SRAM */
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", flash0);
REGION_ALIAS("XTORS_FLASH_LMA", flash0);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", flash0);
REGION_ALIAS("TEXT_FLASH_LMA", flash0);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", flash0);
REGION_ALIAS("RODATA_FLASH_LMA", flash0);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash0);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", ram5);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram5);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

/* Stack rules inclusion.*/
INCLUDE rules_stacks.ld

/*===========================================================================*/
/* Custom sections for STM32H7xx.                                            */
/* SRAM3 is assumed to be marked non-cacheable using MPU.                    */
/*===========================================================================*/

/* RAM region to be used for ram0nc (SDMMC1) nocache segment.*/
/* named nocache because SDIO driver is hardwired to declare buffer nocache */
REGION_ALIAS("NOCACHE_RAM", ram0nc);

/* RAM region to be used for ram3 nocache segment.*/
REGION_ALIAS("NOCACHE3_RAM", ram3);

/* RAM region to be used for ram4 (BDMA) nocache segment.*/
REGION_ALIAS("NOCACHE4_RAM", ram4);

/* RAM region to be used for eth segment.*/
REGION_ALIAS("ETH_RAM", ram3);

SECTIONS
{

/* Special section for non cache-able areas.*/
    .nocache (NOLOAD) : ALIGN(4)
    {
        __nocache_base__ = .;
        *(.nocache)
        *(.nocache.*)
        *(.bss.__nocache_*)
        . = ALIGN(4);
        __nocache_end__ = .;
    	} > NOCACHE_RAM

/* Special section for non cache-able areas.*/
    .nocache3 (NOLOAD) : ALIGN(4)
    {
        __nocache3_base__ = .;
        *(.nocache3)
        *(.nocache3.*)
        *(.bss.__nocache3_*)
        . = ALIGN(4);
        __nocache3_end__ = .;
    } > NOCACHE3_RAM

/* Special section for non cache4-able areas.*/
    .nocache4 (NOLOAD) : ALIGN(4)
    {
        __nocache4_base__ = .;
        *(.nocache4)
        *(.nocache4.*)
        *(.bss.__nocache4_*)
        . = ALIGN(4);
        __nocache4_end__ = .;
    } > NOCACHE4_RAM

       
/* Special section for Ethernet DMA non cache-able areas.*/
    .eth (NOLOAD) : ALIGN(4)
    {
        __eth_base__ = .;
        *(.eth)
        *(.eth.*)
        *(.bss.__eth_*)
        . = ALIGN(4);
        __eth_end__ = .;
    } > ETH_RAM
}


ENTRY(Reset_Handler)

SECTIONS
{
    .vectors : ALIGN(1024)
    {
        __textvectors_base__ = LOADADDR(.vectors);
        __vectors_base__ = .;
        KEEP(*(.vectors))
        __vectors_end__ = .;
    } > VECTORS_FLASH AT > VECTORS_FLASH_LMA

    .xtors : ALIGN(4)
    {
        __init_array_base__ = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end__ = .;
        __fini_array_base__ = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end__ = .;
    } > XTORS_FLASH AT > XTORS_FLASH_LMA

    .text : ALIGN_WITH_INPUT
    {
        __text_base__ = .;
        *(.text)
        *(.text.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
        __text_end__ = .;
    } > TEXT_FLASH AT > TEXT_FLASH_LMA

    .rodata : ALIGN(4)
    {
        __rodata_base__ = .;
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
        __rodata_end__ = .;
    } > RODATA_FLASH AT > RODATA_FLASH_LMA

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .ARM.exidx : {
        __exidx_base__ = .;
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end__ = .;
        __exidx_end = .;
     } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA
}



SECTIONS
{
    .data : ALIGN(4)
    {
        PROVIDE(_textdata = LOADADDR(.data));
        PROVIDE(_data = .);
        __textdata_base__ = LOADADDR(.data);
        __data_base__ = .;
        *(.data)
        *(.data.*)
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
        __data_end__ = .;
    } > DATA_RAM AT > DATA_RAM_LMA

    .bss (NOLOAD) : ALIGN(4)
    {
        __bss_base__ = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
        PROVIDE(end = .);
    } > BSS_RAM
}


__ram0nc_base__         = ORIGIN(ram0nc);
__ram0nc_size__         = LENGTH(ram0nc);
__ram0nc_end__          = __ram0nc_base__ + __ram0nc_size__;
__ram0_base__           = ORIGIN(ram0);
__ram0_size__           = LENGTH(ram0);
__ram0_end__            = __ram0_base__ + __ram0_size__;
__ram1_base__           = ORIGIN(ram1);
__ram1_size__           = LENGTH(ram1);
__ram1_end__            = __ram1_base__ + __ram1_size__;
__ram2_base__           = ORIGIN(ram2);
__ram2_size__           = LENGTH(ram2);
__ram2_end__            = __ram2_base__ + __ram2_size__;
__ram3_base__           = ORIGIN(ram3);
__ram3_size__           = LENGTH(ram3);
__ram3_end__            = __ram3_base__ + __ram3_size__;
__ram4_base__           = ORIGIN(ram4);
__ram4_size__           = LENGTH(ram4);
__ram4_end__            = __ram4_base__ + __ram4_size__;
__ram5_base__           = ORIGIN(ram5);
__ram5_size__           = LENGTH(ram5);
__ram5_end__            = __ram5_base__ + __ram5_size__;
__ram6_base__           = ORIGIN(ram6);
__ram6_size__           = LENGTH(ram6);
__ram6_end__            = __ram6_base__ + __ram6_size__;
__ram7_base__           = ORIGIN(ram7);
__ram7_size__           = LENGTH(ram7);
__ram7_end__            = __ram7_base__ + __ram7_size__;

__flash0_base__         = ORIGIN(flash0);
__flash0_size__         = LENGTH(flash0);
__flash0_end__          = __flash0_base__ + __flash0_size__;
__flash1_base__         = ORIGIN(flash1);
__flash1_size__         = LENGTH(flash1);
__flash1_end__          = __flash1_base__ + __flash1_size__;
__flash2_base__         = ORIGIN(flash2);
__flash2_size__         = LENGTH(flash2);
__flash2_end__          = __flash2_base__ + __flash2_size__;
__flash3_base__         = ORIGIN(flash3);
__flash3_size__         = LENGTH(flash3);
__flash3_end__          = __flash3_base__ + __flash3_size__;
__flash4_base__         = ORIGIN(flash4);
__flash4_size__         = LENGTH(flash4);
__flash4_end__          = __flash4_base__ + __flash4_size__;
__flash5_base__         = ORIGIN(flash5);
__flash5_size__         = LENGTH(flash5);
__flash5_end__          = __flash5_base__ + __flash5_size__;
__flash6_base__         = ORIGIN(flash6);
__flash6_size__         = LENGTH(flash6);
__flash6_end__          = __flash6_base__ + __flash6_size__;
__flash7_base__         = ORIGIN(flash7);
__flash7_size__         = LENGTH(flash7);
__flash7_end__          = __flash7_base__ + __flash7_size__;

SECTIONS
{
     .ram0nc_init : ALIGN(4)
    {
        __ram0nc_init_text__ = LOADADDR(.ram0nc_init);
        __ram0nc_init__ = .;
        KEEP(*(.ram0nc_init))
        KEEP(*(.ram0nc_init.*))
        . = ALIGN(4);
    } > ram0nc AT > RAM_INIT_FLASH_LMA

    .ram0nc (NOLOAD) : ALIGN(4)
    {
        __ram0nc_clear__ = .;
        *(.ram0nc_clear)
        *(.ram0nc_clear.*)
        . = ALIGN(4);
        __ram0nc_noinit__ = .;
        *(.ram0nc)
        *(.ram0nc.*)
        . = ALIGN(4);
        __ram0nc_free__ = .;
    } > ram0nc

    .ram0_init : ALIGN(4)
    {
        __ram0_init_text__ = LOADADDR(.ram0_init);
        __ram0_init__ = .;
        KEEP(*(.ram0_init))
        KEEP(*(.ram0_init.*))
        . = ALIGN(4);
    } > ram0 AT > RAM_INIT_FLASH_LMA

    .ram0 (NOLOAD) : ALIGN(4)
    {
        __ram0_clear__ = .;
        *(.ram0_clear)
        *(.ram0_clear.*)
        . = ALIGN(4);
        __ram0_noinit__ = .;
        *(.ram0)
        *(.ram0.*)
        . = ALIGN(4);
        __ram0_free__ = .;
    } > ram0

 

     .ram1_init : ALIGN(4)
    {
        __ram1_init_text__ = LOADADDR(.ram1_init);
        __ram1_init__ = .;
        KEEP(*(.ram1_init))
  
         KEEP(*(.ram1_init.*))
        . = ALIGN(4);
    } > ram1 AT > RAM_INIT_FLASH_LMA

    .ram1 (NOLOAD) : ALIGN(4)
    {
        __ram1_clear__ = .;
        *(.ram1_clear)
        *(.ram1_clear.*)
        . = ALIGN(4);
        __ram1_noinit__ = .;
        *(.ram1)
        *(.ram1.*)
        . = ALIGN(4);
        __ram1_free__ = .;
    } > ram1

    .ram2_init : ALIGN(4)
    {
        __ram2_init_text__ = LOADADDR(.ram2_init);
        __ram2_init__ = .;
        KEEP(*(.ram2_init))
        KEEP(*(.ram2_init.*))
        . = ALIGN(4);
    } > ram2 AT > RAM_INIT_FLASH_LMA

    .ram2 (NOLOAD) : ALIGN(4)
    {
        __ram2_clear__ = .;
        *(.ram2_clear)
        *(.ram2_clear.*)
        . = ALIGN(4);
        __ram2_noinit__ = .;
        *(.ram2)
        *(.ram2.*)
        . = ALIGN(4);
        __ram2_free__ = .;
    } > ram2

    .ram3_init : ALIGN(4)
    {
        __ram3_init_text__ = LOADADDR(.ram3_init);
        __ram3_init__ = .;
        KEEP(*(.ram3_init))
        KEEP(*(.ram3_init.*))
        . = ALIGN(4);
    } > ram3 AT > RAM_INIT_FLASH_LMA

    .ram3 (NOLOAD) : ALIGN(4)
    {
        __ram3_clear__ = .;
        *(.ram3_clear)
        *(.ram3_clear.*)
        . = ALIGN(4);
        __ram3_noinit__ = .;
        *(.ram3)
        *(.ram3.*)
        . = ALIGN(4);
        __ram3_free__ = .;
    } > ram3

    .ram4_init : ALIGN(4)
    {
        __ram4_init_text__ = LOADADDR(.ram4_init);
        __ram4_init__ = .;
        KEEP(*(.ram4_init))
        KEEP(*(.ram4_init.*))
        . = ALIGN(4);
    } > ram4 AT > RAM_INIT_FLASH_LMA

    .ram4 (NOLOAD) : ALIGN(4)
    {
        __ram4_clear__ = .;
        *(.ram4_clear)
        *(.ram4_clear.*)
        . = ALIGN(4);
        __ram4_noinit__ = .;
        *(.ram4)
        *(.ram4.*)
        . = ALIGN(4);
        __ram4_free__ = .;
    } > ram4

    .ram5_init : ALIGN(4)
    {
        __ram5_init_text__ = LOADADDR(.ram5_init);
        __ram5_init__ = .;
        KEEP(*(.ram5_init))
        KEEP(*(.ram5_init.*))
        . = ALIGN(4);
    } > ram5 AT > RAM_INIT_FLASH_LMA

    .ram5 (NOLOAD) : ALIGN(4)
    {
        __ram5_clear__ = .;
        *(.ram5_clear)
        *(.ram5_clear.*)
        . = ALIGN(4);
        __ram5_noinit__ = .;
        *(.ram5)
        *(.ram5.*)
        . = ALIGN(4);
        __ram5_free__ = .;
    } > ram5

    .ram6_init : ALIGN(4)
    {
        __ram6_init_text__ = LOADADDR(.ram6_init);
        __ram6_init__ = .;
        KEEP(*(.ram6_init))
        KEEP(*(.ram6_init.*))
        . = ALIGN(4);
    } > ram6 AT > RAM_INIT_FLASH_LMA

    .ram6 (NOLOAD) : ALIGN(4)
    {
        __ram6_clear__ = .;
        *(.ram6_clear)
        *(.ram6_clear.*)
        . = ALIGN(4);
        __ram6_noinit__ = .;
        *(.ram6)
        *(.ram6.*)
        . = ALIGN(4);
        __ram6_free__ = .;
    } > ram6

    .ram7_init : ALIGN(4)
    {
        __ram7_init_text__ = LOADADDR(.ram7_init);
        __ram7_init__ = .;
        KEEP(*(.ram7_init))
        KEEP(*(.ram7_init.*))
        . = ALIGN(4);
    } > ram7 AT > RAM_INIT_FLASH_LMA

    .ram7 (NOLOAD) : ALIGN(4)
    {
        __ram7_clear__ = .;
        *(.ram7_clear)
        *(.ram7_clear.*)
        . = ALIGN(4);
        __ram7_noinit__ = .;
        *(.ram7)
        *(.ram7.*)
        . = ALIGN(4);
        __ram7_free__ = .;
    } > ram7

    .flash0 : ALIGN(4)
    {
        __flash0_init__ = .;
        KEEP(*(.flash0_init))
        KEEP(*(.flash0_init.*))
        __flash0_free__ = .;
    } > flash0

    .flash1 : ALIGN(4)
    {
        __flash1_init__ = .;
        KEEP(*(.flash1_init))
        KEEP(*(.flash1_init.*))
        __flash1_free__ = .;
    } > flash1

    .flash2 : ALIGN(4)
    {
        __flash2_init__ = .;
        KEEP(*(.flash2_init))
        KEEP(*(.flash2_init.*))
        __flash2_free__ = .;
    } > flash2

    .flash3 : ALIGN(4)
    {
        __flash3_init__ = .;
        KEEP(*(.flash3_init))
        KEEP(*(.flash3_init.*))
        __flash3_free__ = .;
    } > flash3

    .flash4 : ALIGN(4)
    {
        __flash4_init__ = .;
        KEEP(*(.flash4_init))
        KEEP(*(.flash4_init.*))
        __flash4_free__ = .;
    } > flash4

    .flash5 : ALIGN(4)
    {
        __flash5_init__ = .;
        KEEP(*(.flash5_init))
        KEEP(*(.flash5_init.*))
        __flash5_free__ = .;
    } > flash5

    .flash6 : ALIGN(4)
    {
        __flash6_init__ = .;
        KEEP(*(.flash6_init))
        KEEP(*(.flash6_init.*))
        __flash6_free__ = .;
    } > flash6

    .flash7 : ALIGN(4)
    {
        __flash7_init__ = .;
        KEEP(*(.flash7_init))
        KEEP(*(.flash7_init.*))
        __flash7_free__ = .;
    } > flash7

    /* The default heap uses the (statically) unused part of a RAM section.*/
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __heap_base__ = .;
        . = ORIGIN(HEAP_RAM) + LENGTH(HEAP_RAM);
        __heap_end__ = .;
    } > HEAP_RAM
}
