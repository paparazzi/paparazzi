/*
 * Copyright (C) 2019 Alexis Cornard <alexiscornard@gmail.com>
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
 * @file peripherals/lps25h_i2c.h
 *
 * I2C interface for LPS25H barometer.
 */

#ifndef LPS25H_I2C_H
#define LPS25H_I2C_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include common LPS25H options and definitions */
#include "peripherals/lps25h.h"

struct Lps25h_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Lps25hConfStatus init_status; ///< init status
  bool initialized;                  ///< config done flag
  volatile bool data_available;      ///< data ready flag
  int32_t data;
  struct Lps25hConfig config;
};

// Functions
extern void lps25h_i2c_init(struct Lps25h_I2c *lps, struct i2c_periph *i2c_p, uint8_t addr);
extern void lps25h_i2c_start_configure(struct Lps25h_I2c *lps);
extern void lps25h_i2c_read(struct Lps25h_I2c *lps);
extern void lps25h_i2c_event(struct Lps25h_I2c *lps);
extern float lps25h_readPressureMillibars(int32_t press);
extern float pressureToAltMeters(float pressure_mbar, float altimeter_setting_mbar);

// convenience function: read or start configuration if not already initialized
static inline void lps25h_i2c_periodic(struct Lps25h_I2c *lps)
{
  if (lps->initialized) {
    lps25h_i2c_read(lps);
  } else {
    lps25h_i2c_start_configure(lps);
  }
}

#endif // LPS25H_I2C_H
