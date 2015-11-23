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
 * @file "modules/helicopter/throttle_curve.c"
 * @author C. De Wagter and Freek van Tienen
 * Throttle Curve Mixers
 */

#include "throttle_curve.h"

/* The switching values for the Throttle Curve Mode switch */
#define THROTTLE_CURVE_SWITCH_VAL (MAX_PPRZ*2/THROTTLE_CURVES_NB)

/* Initialize the throttle curves from the airframe file */
struct throttle_curve_t throttle_curve = {
  .nb_curves = THROTTLE_CURVES_NB,
  .curves = THROTTLE_CURVES
};

/**
 * Initialize the default throttle curve values
 */
void throttle_curve_init(void)
{
  throttle_curve.mode       = THROTTLE_CURVE_MODE_INIT;
  throttle_curve.throttle   = throttle_curve.curves[THROTTLE_CURVE_MODE_INIT].throttle[0];
  throttle_curve.collective = throttle_curve.curves[THROTTLE_CURVE_MODE_INIT].collective[0];
}

/**
 * Run the throttle curve and generate the output throttle and pitch
 * This depends on the FMODE(flight mode) and TRHUST command
 */
void throttle_curve_run(bool_t motors_on, pprz_t in_cmd[])
{
  // Calculate the mode value from the switch
  int8_t mode = ((float)(in_cmd[COMMAND_FMODE] + MAX_PPRZ) / THROTTLE_CURVE_SWITCH_VAL);
  Bound(mode, 0, THROTTLE_CURVES_NB - 1);
  throttle_curve.mode = mode;

  // Check if we have multiple points or a single point
  struct curve_t curve = throttle_curve.curves[mode];
  if (curve.nb_points == 1) {
    throttle_curve.throttle = curve.throttle[0];
    throttle_curve.collective = curve.collective[0];
  } else {
    // Calculate the left point on the curve we need to use
    uint16_t curve_range = (MAX_PPRZ / (curve.nb_points - 1));
    int8_t curve_p = ((float)in_cmd[COMMAND_THRUST] / curve_range);
    Bound(curve_p, 0, curve.nb_points - 1);

    // Calculate the throttle and pitch value
    uint16_t x = in_cmd[COMMAND_THRUST] - curve_p * curve_range;
    throttle_curve.throttle = curve.throttle[curve_p]
                              + ((curve.throttle[curve_p + 1] - curve.throttle[curve_p]) * x / curve_range);
    throttle_curve.collective = curve.collective[curve_p]
                                + ((curve.collective[curve_p + 1] - curve.collective[curve_p]) * x / curve_range);
  }

  // Only set throttle if motors are on
  if (!motors_on) {
    throttle_curve.throttle = 0;
  }
}
