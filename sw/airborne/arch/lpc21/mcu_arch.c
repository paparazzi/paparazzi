/*
 * Copyright (C) 2010 The Paparazzi team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/lpc21/mcu_arch.c
 * @brief lpc21 arch dependant microcontroller initialisation functions.
 * @ingroup lpc21_arch
 */

#include "mcu.h"


#include <inttypes.h>
#include BOARD_CONFIG
#include "LPC21xx.h"


/* declare functions and values from crt0.S & the linker control file */
extern void reset(void);

void mcu_arch_init(void)
{

  /* set PLL multiplier & divisor. */
  /* values computed from config.h */
  PLLCFG = PLLCFG_MSEL | PLLCFG_PSEL;
  /* enable PLL                    */
  PLLCON = PLLCON_PLLE;
  /* commit changes                */
  PLLFEED = 0xAA;
  PLLFEED = 0x55;

  /* wait for PLL lock */
  while (!(PLLSTAT & PLLSTAT_LOCK)) {
    continue;
  }

  /* enable & connect PLL */
  PLLCON = PLLCON_PLLE | PLLCON_PLLC;
  /* commit changes                */
  PLLFEED = 0xAA;
  PLLFEED = 0x55;

  /* setup & enable the MAM */
  MAMTIM = MAMTIM_CYCLES;
  MAMCR = MAMCR_FULL;

  /* set the peripheral bus speed */
  /* value computed from config.h */
  VPBDIV = VPBDIV_VALUE;

  /* set the interrupt controller to flash */
  MEMMAP = MEMMAP_FLASH;

  /* clear all interrupts     */
  VICIntEnClear = 0xFFFFFFFF;
  /* clear all FIQ selections */
  VICIntSelect = 0x00000000;
  /* point unvectored IRQs to reset() */
  VICDefVectAddr = (uint32_t)reset;

}
