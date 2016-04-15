/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/nav/nav_heli_spinup.h"
 * @author Freek van Tienen
 * This module controls the spinup of the main rotor from a Helicopter
 */

#ifndef NAV_HELI_SPINUP_H
#define NAV_HELI_SPINUP_H

#include "std.h"

struct nav_heli_spinup_t {
  uint16_t duration;        ///< The duration in seconds to reach the final throttle
  uint32_t throttle;        ///< The final throttle level
};

/** Initialization function
 *
 * Called in the flight plan before the 'run' function
 *
 * @param[in] duration The duration in seconds before reaching final throttle level
 * @param[in] throttle The throttle value to go to
 */
extern void nav_heli_spinup_setup(uint16_t duration, float throttle);

/** Heli spinup run function
 *
 * Controls the spinup of the throttle without stabilization
 *
 * @return true until the takeoff procedure ends
 */
extern bool nav_heli_spinup_run(void);

#endif
