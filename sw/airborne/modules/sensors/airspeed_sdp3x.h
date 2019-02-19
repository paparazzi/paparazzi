/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file modules/sensors/airspeed_sdp3x.h
 *  Airspeed driver for the SDP3X pressure sensor via I2C.
 */

#ifndef AIRSPEED_SDP3X_H
#define AIRSPEED_SDP3X_H

#include "std.h"

struct AirspeedSdp3x {
  float pressure;              ///< (differential) pressure in Pascal
  float temperature;           ///< Temperature in deg Celcius
  float airspeed;              ///< Airspeed in m/s estimated from (differential) pressure.
  float airspeed_scale;        ///< Quadratic scale factor to convert (differential) pressure to airspeed
  float pressure_scale;        ///< Scaling factor from raw measurement to Pascal
  float pressure_offset;       ///< Offset in Pascal
  bool sync_send;              ///< Flag to enable sending every new measurement via telemetry for debugging purpose
  bool initialized;            ///< init flag
};

extern struct AirspeedSdp3x sdp3x;

extern void sdp3x_init(void);
extern void sdp3x_periodic(void);
extern void sdp3x_event(void);

#endif
