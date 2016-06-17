/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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
 * STM32F407xG memory setup.
 * Note: Use of ram1 and ram2 is mutually exclusive with use of ram0.
 */
MEMORY
{
    flash : org = 0x08000000, len = 1M
    ram0  : org = 0x20000000, len = 128k    /* SRAM1 + SRAM2 */
    ram1  : org = 0x20000000, len = 112k    /* SRAM1 */
    ram2  : org = 0x2001C000, len = 16k     /* SRAM2 */
    ram3  : org = 0x00000000, len = 0
    ram4  : org = 0x10000000, len = 64k     /* CCM SRAM */
    ccmram  : org = 0x10000000, len = 64k     /* CCM SRAM */
    ram5  : org = 0x40024000, len = 4k      /* BCKP SRAM */
    ram6  : org = 0x00000000, len = 0
    ram7  : org = 0x00000000, len = 0
}


/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

__ccmram_start__          = ORIGIN(ccmram);
__ccmram_size__           = LENGTH(ccmram);
__ccmram_end__            = __ccmram_start__ + __ccmram_size__;

SECTIONS
{
  .ccmram_init : ALIGN(4)
    {
        . = ALIGN(4);
        __ccmram_init_text__ = LOADADDR(.ccmram_init);
        __ccmram_init__ = .;
        *(.ccmram_init)
        *(.ccmram_init.*)
        . = ALIGN(4);
    } > ccmram AT > flash

    .ccmram (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __ccmram_clear__ = .;
        *(.ccmram_clear)
        *(.ccmram_clear.*)
        . = ALIGN(4);
        __ccmram_noinit__ = .;
        *(.ccmram)
        *(.ccmram.*)
        . = ALIGN(4);
        __ccmram_free__ = .;
    } > ccmram 

}

INCLUDE rules.ld
