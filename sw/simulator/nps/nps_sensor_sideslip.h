/*
 * Copyright (C) 2016 Johan Maurin, Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file nps_sensor_sideslip.h
 *
 * Simulated Angle Of Attack of the Wind for NPS simulator.
 *
 */

#ifndef NPS_SENSOR_SIDESLIP_H
#define NPS_SENSOR_SIDESLIP_H

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_float.h"
#include "std.h"

struct NpsSensorSideSlip{
  double value;          ///< sideslip reading in radian
  double offset;         ///< offset in meters/second
  double noise_std_dev;  ///< noise standard deviation
  double next_update;
  bool data_available;
};


extern void nps_sensor_sideslip_init(struct NpsSensorSideSlip *sideslip, double time);
extern void nps_sensor_sideslip_run_step(struct NpsSensorSideSlip *sideslip, double time);

#endif /* NPS_SENSOR_SIDESLIP_H */

