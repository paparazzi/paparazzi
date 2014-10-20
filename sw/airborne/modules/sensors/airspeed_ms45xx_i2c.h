/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2014 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file modules/sensors/airspeed_ms45xx_i2c.h
 *  Airspeed driver for the MS45xx via I2C.
 */

#ifndef AIRSPEED_MS45XX_I2C_H
#define AIRSPEED_MS45XX_I2C_H

#include "std.h"

struct AirspeedMs45xx {
  float diff_pressure;   ///< differential pressure in Pascal
  int16_t temperature;   ///< temperature in 0.1 deg Celcius
  float airspeed;        ///< Airspeed in m/s estimated from differential pressure.
  float airspeed_scale;  ///< quadratic scale factor to convert differential pressure to airspeed
  float pressure_scale;  ///< scaling factor from raw measurement to Pascal
  float pressure_offset; ///< offset in Pascal
  bool_t sync_send;      ///< flag to enable sending every new measurement via telemetry
};

extern struct AirspeedMs45xx ms45xx;

extern void ms45xx_i2c_init(void);
extern void ms45xx_i2c_periodic(void);
extern void ms45xx_i2c_event(void);

#endif
