/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/*
 *\brief ARM7 low level hardware initialisation 
 * PLL, IOPORT, MAM, VIC
 *
 */

#ifndef LOW_LEVEL_HW_H
#define LOW_LEVEL_HW_H

#include "LPC21xx.h"
#include "config.h"

static inline void low_level_init(void) {
  /* set PLL multiplier & divisor. */
  /* values computed from config.h */
  PLLCFG = PLLCFG_MSEL | PLLCFG_PSEL;
  /* enable PLL                    */
  PLLCON = PLLCON_PLLE;
  /* commit changes                */
  PLLFEED = 0xAA;
  PLLFEED = 0x55;

  /* setup port pins */
  IO0CLR = PIO0_ZERO_BITS;                // clear the ZEROs output
  IO0SET = PIO0_ONE_BITS;                 // set the ONEs output
  IO0DIR = PIO0_OUTPUT_BITS;              // set the output bit direction
  
  IO1CLR = PIO1_ZERO_BITS;                // clear the ZEROs output
  IO1SET = PIO1_ONE_BITS;                 // set the ONEs output
  IO1DIR = PIO1_OUTPUT_BITS;              // set the output bit direction
  
  /* wait for PLL lock */
  while (!(PLLSTAT & PLLSTAT_LOCK))
    continue;
 
  /* enable & connect PLL */
  PLLCON = PLLCON_PLLE | PLLCON_PLLC;
  /* commit changes                */
  PLLFEED = 0xAA;
  PLLFEED = 0x55;

  /* setup & enable the MAM */
  MAMTIM = MAMTIM_CYCLES;
  MAMCR = MAMCR_FULL;
  
  /* setup & enable the MAM */
  MAMTIM = MAMTIM_CYCLES;
  MAMCR = MAMCR_FULL;
  
  /* set the peripheral bus speed */
  /* value computed from config.h */
  VPBDIV = VPBDIV_VALUE;
 
  /* set the interrupt controller defaults  */
#if defined(RAM_RUN)
  /* map interrupt vectors space into SRAM  */
  MEMMAP = MEMMAP_SRAM;
#elif defined(ROM_RUN)
  /* map interrupt vectors space into FLASH */
  MEMMAP = MEMMAP_FLASH;
#else
#error RUN_MODE not defined!
#endif

  /* clear all interrupts     */
  VICIntEnClear = 0xFFFFFFFF;
  /* clear all FIQ selections */
  VICIntSelect = 0x00000000;
  /* point unvectored IRQs to reset() */
  VICDefVectAddr = (uint32_t)reset;

}

#endif /* LOW_LEVEL_HW_H */
