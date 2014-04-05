/*
 *
 * Copyright (C) 2012, Christophe De Wagter
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
 *
 */

/**
 * @file modules/nav/nav_catapult.h
 * @brief catapult launch timing system
 */

#ifndef NAV_CATAPULT_H
#define NAV_CATAPULT_H

#include "std.h"
#include "paparazzi.h"

extern float nav_catapult_motor_delay;
extern float nav_catapult_acceleration_threshold;
extern float nav_catapult_heading_delay;
extern float nav_catapult_initial_pitch;
extern float nav_catapult_initial_throttle;

// Module Code
void nav_catapult_highrate_module(void);

// Flightplan Code
extern bool_t nav_catapult_setup(void);

extern bool_t nav_catapult_arm(void);
extern bool_t nav_catapult_run(uint8_t _to, uint8_t _climb);
extern bool_t nav_catapult_disarm(void);

extern bool_t nav_select_touch_down(uint8_t _td);

#endif
