/*
 * Copyright (C) 2013 Sergey Krukowski
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
 */

#ifndef AUTOPILOT_FP_BLOCK_SWITCH_H
#define AUTOPILOT_FP_BLOCK_SWITCH_H

#include "autopilot_rc_helpers.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "led.h"


#define AUTOPILOT_BLOCK_SWITCH_TIME     40
#define AUTOPILOT_STICK_STATE_UNDEF     AUTOPILOT_BLOCK_SWITCH_TIME + 1

#define AUTOPILOT_BLOCK_HOME 						5
#define AUTOPILOT_BLOCK_LANDING					9
#define AUTOPILOT_BLOCK_CIRCLE					7
#define AUTOPILOT_BLOCK_ROUTE						6


uint32_t autopilot_block_switch_counter;
uint8_t autopilot_block_switch_state;


static inline void autopilot_block_switch_init(void) {
  autopilot_block_switch_counter = 0;
  autopilot_block_switch_state = nav_block;
}

/**
 * Nav block machine to check block switching from RC.
 */
static inline void autopilot_block_switch_check( bool_t in_flight ) {
  /* only allow switching blocks if we are in NAV mode and if already in flight*/
  if (autopilot_mode == AP_MODE_NAV && in_flight) {
		if (THROTTLE_STICK_CENTERED() && YAW_STICK_CENTERED()) {
			if(ROLL_STICK_CENTERED() && PITCH_STICK_CENTERED()) {
				autopilot_block_switch_counter = 0;
			}
			else {
				if(autopilot_block_switch_counter < AUTOPILOT_BLOCK_SWITCH_TIME) {
					if(PITCH_STICK_PUSHED_DOWN() && ROLL_STICK_PUSHED_LEFT()) {								// Home
						autopilot_block_switch_counter++;
						autopilot_block_switch_state = RC_HOME;
					} else if(PITCH_STICK_PUSHED_DOWN() && ROLL_STICK_PUSHED_RIGHT()) { 			// Landing
						autopilot_block_switch_counter++;
						autopilot_block_switch_state = RC_LAND;
					} else if(PITCH_STICK_PUSHED_UP() && ROLL_STICK_PUSHED_LEFT()) {					// Circle
						autopilot_block_switch_counter++;
						autopilot_block_switch_state = RC_CIRCLE;
					} else if(PITCH_STICK_PUSHED_UP() && ROLL_STICK_PUSHED_RIGHT()) {					// Route
						autopilot_block_switch_counter++;
						autopilot_block_switch_state = RC_ROUTE;
					} else if(autopilot_block_switch_counter) {
						autopilot_block_switch_counter = AUTOPILOT_STICK_STATE_UNDEF;
					}
				} else if(rc_command != autopilot_block_switch_state && autopilot_block_switch_counter == AUTOPILOT_BLOCK_SWITCH_TIME) {
					rc_command = autopilot_block_switch_state;
					autopilot_block_switch_counter = AUTOPILOT_STICK_STATE_UNDEF;
				}
			}
		}
  }
}

#endif /* AUTOPILOT_FP_BLOCK_SWITCH_H */
