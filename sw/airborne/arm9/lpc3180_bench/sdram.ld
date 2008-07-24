/*****************************************************************************
 * Phytec LPC3180 linker script
 * Linker directive "sdram.ld" for putting code into board's external SDRAM
 * Last update  29 June 2008
 * Copyright(C) 2008, Dirk Behme, dirk.behme@gmail.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 *****************************************************************************/

ENTRY(_start)
OUTPUT_FORMAT("elf32-littlearm")

SECTIONS
{
  . = 0x80000000;
  _text_start = .;
  .text : { *(.text)}
  _text_end = .;

  _data_start = .;
  .data : { *(.rodata)
            *(.data)
            *(.sdata)}
  _data_end = .;

  _bss_start = .;
  .bss : { *(.bss)
           *(COMMON)}

  .sbss  : { *(.sbss)
             *(.scommon)}

   _bss_end = .;
   _end = .;
   end = .;
}
