/*
 * Copyright (C) 2025 MVLab <microuav@gmail.com>
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

/** @file "modules/gps/gps_propagate.h"
 * @author MVLab <microuav@gmail.com>
 * Continue flying on baro and magnetometer when GPS fails
 */

#ifndef GPS_PROPAGATE_H
#define GPS_PROPAGATE_H

#include "std.h"

extern void gps_propagate_init(void);
extern void gps_propagate_periodic(void);

extern void gps_propagate_vision_callback(int32_t lat, int32_t lon, int32_t amsl_mm);
extern void gps_propagate_reset_rls(void);

enum ExtrapolationMethod {
  HEADING_BIAS, // Use heading bias to extrapolate position
  WIND_VECTOR, // Use wind vector and airspeed to extrapolate position
  RLS, // Use Recursive Least Squares to estimate wind and airspeed
};
extern enum ExtrapolationMethod extrapolation_method; // Default extrapolation method
extern float lambda; // Forgetting factor
extern bool wind_estimation_enabled;

#endif  // GPS_PROPAGATE_H
