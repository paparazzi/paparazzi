/*
 * Copyright (C) 2021 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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
 * @file "modules/ctrl/approach_moving_target.h"
 * @author Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * Approach a moving target (e.g. ship)
 */

#ifndef APPROACH_MOVING_TARGET_H
#define APPROACH_MOVING_TARGET_H

#include "std.h"
#include "math/pprz_algebra_float.h"

// Angle of the approach in degrees
extern float approach_moving_target_angle_deg;

struct Amt {
  struct FloatVect3 rel_unit_vec;
  float distance;
  float speed;
  float pos_gain;
  float psi_ref;
  float slope_ref;
  float speed_gain;
  float relvel_gain;
  int32_t enabled_time;
  uint8_t wp_id;
};

extern struct Amt amt;
extern float amt_err_slowdown_gain;

extern void approach_moving_target_init(void);
extern void follow_diagonal_approach(void);
extern void approach_moving_target_enable(uint8_t wp_id);

#endif // APPROACH_MOVING_TARGET_H
