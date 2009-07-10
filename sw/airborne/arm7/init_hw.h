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
 * PLL, MAM, VIC
 *
 */

#ifndef INIT_HW_H
#define INIT_HW_H

#include <inttypes.h>
#include CONFIG
#include "LPC21xx.h"


#ifdef PERIPHERALS_AUTO_INIT
#ifdef LED
#include "led.h"
#endif
#if defined USE_UART0 || defined USE_UART1
#include "uart.h"
#endif
#ifdef USE_USB_SERIAL
#include "usb_serial.h"
#endif
#if defined USE_I2C0 || defined USE_I2C1
#include "i2c.h"
#endif
#endif /* PERIPHERALS_AUTO_INIT */


/* declare functions and values from crt0.S & the linker control file */
extern void reset(void);
/* extern void exit(void); */
extern void abort(void);


static inline void hw_init(void) {
  /* set PLL multiplier & divisor. */
  /* values computed from config.h */
  PLLCFG = PLLCFG_MSEL | PLLCFG_PSEL;
  /* enable PLL                    */
  PLLCON = PLLCON_PLLE;
  /* commit changes                */
  PLLFEED = 0xAA;
  PLLFEED = 0x55;

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
  //#if defined(RAM_RUN)
  /* map interrupt vectors space into SRAM  */
  //  MEMMAP = MEMMAP_SRAM;
  //#elif defined(ROM_RUN)
  /* map interrupt vectors space into FLASH */
  MEMMAP = MEMMAP_FLASH;
  //#else
  //#error RUN_MODE not defined!
  //#endif

  /* clear all interrupts     */
  VICIntEnClear = 0xFFFFFFFF;
  /* clear all FIQ selections */
  VICIntSelect = 0x00000000;
  /* point unvectored IRQs to reset() */
  VICDefVectAddr = (uint32_t)reset;


#ifdef PERIPHERALS_AUTO_INIT
#ifdef LED
  led_init();
#endif
#ifdef USE_UART0
  uart0_init_tx();
#endif
#ifdef USE_UART1
  uart1_init_tx();
#endif
#ifdef USE_USB_SERIAL
  VCOM_init();
#endif
#ifdef USE_I2C0
  i2c0_init();
#endif
#ifdef USE_I2C1
  i2c1_init();
#endif
#endif /* PERIPHERALS_AUTO_INIT */


}

#endif /* INIT_HW_H */
