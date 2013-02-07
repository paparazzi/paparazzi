/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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
 * @file arch/stm32/peripherals/ms2100_arch.c
 *
 * STM32 specific functions for the ms2100 magnetic sensor from PNI.
 */

#include "peripherals/ms2100.h"

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/exti.h>

void ms2100_arch_init( void ) {

  /* set mag reset as output (reset on PC13) ----*/
  Ms2100Reset();

  /* configure data ready on PB5 */
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
	        GPIO_CNF_INPUT_FLOAT, GPIO5);

  // TODO configure IRQ for drdy pin
}

void ms2100_reset_cb( struct spi_transaction * t __attribute__ ((unused)) ) {
  // set RESET pin high for at least 100 nsec
  // busy wait should not harm
  // storing start and dt is probably long enough...
  Ms2100Set();
  // TODO wait loop so the reset toggle is long enough
  Ms2100Reset();
}
