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
 * @file peripherals/l3gd20.h
 *
 * ST L3GD20 3-axis gyroscope driver common interface (I2C and SPI).
 */

#ifndef L3GD20_H
#define L3GD20_H

/* Include address and register definition */
#include "peripherals/l3gd20_regs.h"


/** default gyro sensitivy and neutral from the datasheet
 * L3GD20 has 70e-3 LSB/(deg/s) at 2000deg/s range
 * sens = 70e-3 * pi/180 * 2^INT32_RATE_FRAC
 * sens = (70e-3 / 180.0f) * pi * 4096
  */
#define L3GD20_SENS_2000_NUM 5004
#define L3GD20_SENS_2000_DEN 1000

enum L3gd20ConfStatus {
  L3G_CONF_UNINIT = 0,
  L3G_CONF_WHO_AM_I = 1,
  L3G_CONF_WHO_AM_I_OK = 2,
  L3G_CONF_REG4   = 3,
  L3G_CONF_ENABLE = 4,
  L3G_CONF_DONE   = 5
};

struct L3gd20Config {
  bool spi_3_wire;        ///< Set 3-wire SPI mode, if FALSE: 4-wire SPI mode

  enum L3gd20FullScale full_scale; ///< gyro full scale
  enum L3gd20DRBW drbw;   ///< Data rate and bandwidth
};

static inline void l3gd20_set_default_config(struct L3gd20Config *c)
{
  c->spi_3_wire = false;

  c->drbw = L3GD20_DRBW_760Hz_100BW;
  c->full_scale = L3GD20_FS_2000dps2;
}

#endif /* L3GD20_H */
