/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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
 */

/**
 * @file arch/lpc21/peripherals/ms2100_arch.c
 *
 * LPC21xx specific functions for the ms2100 magnetic sensor from PNI.
 */

#include "peripherals/ms2100.h"

#include "LPC21xx.h"
#include "armVIC.h"
#include "mcu_periph/sys_time.h"
#include BOARD_CONFIG

static void EXTINT_ISR(void) __attribute__((naked));

void ms2100_arch_init(void)
{

  /* configure RESET pin */
  Ms2100Reset();                                /* pin idles low  */
  SetBit(MS2100_RESET_IODIR, MS2100_RESET_PIN); /* pin is output  */

  /* configure DRDY pin */
  /* connected pin to EXINT */
  MS2100_DRDY_PINSEL |= MS2100_DRDY_PINSEL_VAL << MS2100_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, MS2100_DRDY_EINT);  /* EINT is edge trigered */
  SetBit(EXTPOLAR, MS2100_DRDY_EINT); /* EINT is trigered on rising edge */
  SetBit(EXTINT, MS2100_DRDY_EINT);   /* clear pending EINT */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(MS2100_DRDY_VIC_IT);   /* select EINT as IRQ source */
  VICIntEnable = VIC_BIT(MS2100_DRDY_VIC_IT);     /* enable it                 */
  _VIC_CNTL(MS2100_DRDY_VIC_SLOT) = VIC_ENABLE | MS2100_DRDY_VIC_IT;
  _VIC_ADDR(MS2100_DRDY_VIC_SLOT) = (uint32_t)EXTINT_ISR;         // address of the ISR

}

void EXTINT_ISR(void)
{
  ISR_ENTRY();
  /* no, we won't do anything asynchronously, so just notify */
  ms2100.status = MS2100_GOT_EOC;
  /* clear EINT */
  EXTINT = (1 << MS2100_DRDY_EINT);
  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void ms2100_reset_cb(struct spi_transaction *t __attribute__((unused)))
{
  // set RESET pin high for at least 100 nsec
  // busy wait should not harm
  // storing start and dt is probably long enough...
  Ms2100Set();
  uint32_t start = T0TC;
  uint32_t dt = cpu_ticks_of_nsec(110);
  while ((uint32_t)(T0TC - start) < dt);
  Ms2100Reset();
}

