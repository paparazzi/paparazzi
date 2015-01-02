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
 * @file arch/lpc21/peripherals/ms2100_arch.h
 *
 * LPC21xx specific functions for the ms2100 magnetic sensor from PNI.
 */

#ifndef MS2100_ARCH_H
#define MS2100_ARCH_H

#include "mcu_periph/spi.h"

#define Ms2100Reset() SetBit(MS2100_RESET_IOCLR,MS2100_RESET_PIN)
#define Ms2100Set()   SetBit(MS2100_RESET_IOSET,MS2100_RESET_PIN)

/** Reset callback.
 * called before spi transaction and after slave select
 */
extern void ms2100_reset_cb(struct spi_transaction *t);

#endif /* MS2100_ARCH_H */
