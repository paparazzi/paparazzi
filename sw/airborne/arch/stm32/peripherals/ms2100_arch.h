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

#ifndef MS2100_ARCH_H
#define MS2100_ARCH_H

#include <libopencm3/stm32/f1/gpio.h>
#include "mcu_periph/spi.h"

#define Ms2100Reset() GPIOC_BSRR = GPIO13;
#define Ms2100Set()   GPIOC_BRR = GPIO13

#define Ms2100HasEOC() (gpio_get(GPIOB, GPIO5) != 0)

/** Reset callback.
 * called before spi transaction and after slave select
 */
extern void ms2100_reset_cb( struct spi_transaction * t );

#endif /* MS2100_ARCH_H */
