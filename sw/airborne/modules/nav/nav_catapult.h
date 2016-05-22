/*
 * Copyright (C) 2012, Christophe De Wagter
 * Copyright (C) 2016, Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/**
 * @file modules/nav/nav_catapult.h
 * @brief catapult launch timing system
 *
 *
 * - Phase 1: Zero Roll, Climb Pitch, Zero Throttle
 * - Phase 2: After detecting the Start Acceleration\n
 *            Zero Roll, Climb Pitch, Full Throttle
 * - Phase 3: After getting the GPS heading (time based)\n
 *            Place climb in front of us\n
 *            GoTo(climb)
 */

#ifndef NAV_CATAPULT_H
#define NAV_CATAPULT_H

#include "std.h"
#include "math/pprz_algebra_float.h"

// Take off state machine
enum nav_catapult_state {
  NAV_CATAPULT_UNINIT,
  NAV_CATAPULT_ARMED,
  NAV_CATAPULT_WAIT_ACCEL,
  NAV_CATAPULT_MOTOR_ON,
  NAV_CATAPULT_MOTOR_CLIMB,
  NAV_CATAPULT_DISARM,
};

// Internal structure
struct nav_catapult_struct {
  enum nav_catapult_state status; ///< current procedure state
  uint32_t timer;                 ///< internal timer
  struct FloatVect3 pos;          ///< catapult position
  float accel_threshold;          ///< acceleration threshold for launch detection (in g)
  float motor_delay;              ///< delay to start motor after launch detection (in seconds)
  float heading_delay;            ///< delay to estimate initial heading after launch (in seconds)
  float initial_pitch;            ///< pitch angle during first take-off phase (in radian)
  float initial_throttle;         ///< throttle during first take-off phase (in radian)
};

extern struct nav_catapult_struct nav_catapult;

// Init
extern void nav_catapult_init(void);

// Module Code
extern void nav_catapult_highrate_module(void);

// Flightplan Code
extern bool nav_catapult_run(uint8_t _climb);

#endif

