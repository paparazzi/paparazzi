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
 * @file peripherals/adxl345.h
 *
 * Analog Devices ADXL345 accelerometer driver common interface (I2C and SPI).
 */

#ifndef ADXL345_H
#define ADXL345_H

#include "std.h"

/* Include address and register definition */
#include "peripherals/adxl345_regs.h"

enum Adxl345ConfStatus {
  ADXL_CONF_UNINIT = 0,
  ADXL_CONF_RATE   = 1,
  ADXL_CONF_INT    = 2,
  ADXL_CONF_FORMAT = 3,
  ADXL_CONF_ENABLE = 4,
  ADXL_CONF_DONE   = 5
};

struct Adxl345Config {
  bool drdy_int_enable;   ///< Enable Data Ready Interrupt
  bool int_invert;        ///< Invert Interrupt FALSE: active high, TRUE: active low
  bool full_res;          ///< Full Resolution: FALSE: 10bit, TRUE: full with 4mg/LSB
  bool justify_msb;       ///< justify: FALSE: right with sign-extension, TRUE: MSB (left)
  bool self_test;         ///< Enable self-test-force causing shift in output data.
  bool spi_3_wire;        ///< Set 3-wire SPI mode, if FALSE: 4-wire SPI mode
  enum Adxl345Ranges range; ///< g Range
  enum Adxl345Rates rate;   ///< Data Output Rate
};

static inline void adxl345_set_default_config(struct Adxl345Config *c)
{
  c->drdy_int_enable = false;
  c->int_invert = true;
  c->full_res = true;
  c->justify_msb = false;
  c->self_test = false;
  c->spi_3_wire = false;

  c->rate = ADXL345_RATE_100HZ;
  c->range = ADXL345_RANGE_16G;
}

static inline uint8_t adxl345_data_format(struct Adxl345Config *c)
{
  return ((c->self_test << 7) | (c->spi_3_wire << 6) | (c->int_invert << 5) |
          (c->full_res << 3) | (c->justify_msb << 2) | (c->range));
}

#endif // ADXL345_H
