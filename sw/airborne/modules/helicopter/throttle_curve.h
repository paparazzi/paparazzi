/*
 * Copyright (C) 2015 C. De Wagter
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
/**
 * @file "modules/helicopter/throttle_curve.h"
 * @author C. De Wagter and Freek van Tienen
 * Throttle Curve Mixers
 */

#ifndef THROTTLE_CURVE_H
#define THROTTLE_CURVE_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

/* Throttle and collective curve */
struct curve_t {
  uint8_t nb_points;                          ///< The number of points in the curve
  uint16_t throttle[THROTTLE_POINTS_NB];      ///< Throttle points in the curve
  int16_t collective[THROTTLE_POINTS_NB];     ///< The collective points in the curve
};

/* Main throttle curve structure */
struct throttle_curve_t {
  uint8_t mode;                               ///< Flight mode
  uint8_t nb_curves;                          ///< The number of throttle/pitch curves
  struct curve_t curves[THROTTLE_CURVES_NB];  ///< Throttle/pitch curves

  uint16_t throttle;                          ///< Output thrust(throttle) of the throttle curve
  int16_t collective;                         ///< Output collective of the throttle curve
};
extern struct throttle_curve_t throttle_curve;

/* External functions */
extern void throttle_curve_init(void);
void throttle_curve_run(bool motors_on, pprz_t in_cmd[]);
void nav_throttle_curve_set(uint8_t mode);

#endif
