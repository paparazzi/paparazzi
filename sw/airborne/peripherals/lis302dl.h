/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/lis302dl.h
 *
 * ST LIS302DL 3-axis accelerometer driver common interface (I2C and SPI).
 */

#ifndef LIS302DL_H
#define LIS302DL_H

/* Include address and register definition */
#include "peripherals/lis302dl_regs.h"

enum Lis302dlConfStatus {
  LIS_CONF_UNINIT = 0,
  LIS_CONF_WHO_AM_I = 1,
  LIS_CONF_WHO_AM_I_OK = 2,
  LIS_CONF_REG2   = 3,
  LIS_CONF_REG3   = 4,
  LIS_CONF_ENABLE = 5,
  LIS_CONF_DONE   = 6
};

struct Lis302dlConfig {
  bool_t int_invert;        ///< Invert Interrupt FALSE: active high, TRUE: active low
  bool_t spi_3_wire;        ///< Set 3-wire SPI mode, if FALSE: 4-wire SPI mode

  /** Filtered Data Selection.
   * FALSE: internal filter bypassed;
   * TRUE: data from internal filter sent to output register */
  bool_t filt_data;
  enum Lis302dlRanges range; ///< g Range
  enum Lis302dlRates rate;   ///< Data Output Rate
};

static inline void lis302dl_set_default_config(struct Lis302dlConfig *c)
{
  c->int_invert = TRUE;
  c->filt_data = FALSE;
  c->spi_3_wire = FALSE;

  c->rate = LIS302DL_RATE_100HZ;
  c->range = LIS302DL_RANGE_2G;
}

#endif /* LIS302DL_H */
