/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rotwing/rotwing_state.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module keeps track of the current state of a rotating wing drone and desired state set by the RC or flightplan. Paramters are being scheduled in each change of a current state and desired state. Functions are defined in this module to call the actual state and desired state and set a desired state.
 */

#ifndef ROTWING_STATE_H
#define ROTWING_STATE_H

#include "std.h"

/** Rotwing States
 */
#define ROTWING_STATE_HOVER               0 // Wing is skewed in 0 degrees (quad)
#define ROTWING_STATE_SKEWING             1 // WIng is skewing
#define ROTWING_STATE_FW                  2 // Wing is skewed at 90 degrees (fixed wing), hover motors have full authority
#define ROTWING_STATE_FW_HOV_MOT_IDLE     3 // Wing is skewed at 90 degrees (fixed wing), hover motors are forced to idle
#define ROTWING_STATE_FW_HOV_MOT_OFF      4 // Wing is skewed at 90 degrees (fixed wubg), hover motors are switched off
#define ROTWING_STATE_FREE                5 // This is a desired state for which the controller has to decide the desired state itself

struct RotwingState {
  uint8_t current_state;
  uint8_t desired_state;
};

#define ROTWING_STATE_WING_QUAD_SETTING         0 // Wing skew at 0 degrees
#define ROTWING_STATE_WING_SCHEDULING_SETTING   1 // Wing skew handled by airspeed scheduler
#define ROTWING_STATE_WING_FW_SETTING           2 // Wing skew at 90 degrees

#define ROTWING_STATE_PITCH_QUAD_SETTING        0 // Pitch at prefered hover
#define ROTWING_STATE_PITCH_TRANSITION_SETTING  1 // Pitch scheduled
#define ROTWING_STATE_PITCH_FW_SETTING          2 // Pitch at prefered forward

struct RotWingStateSettings {
  uint8_t wing_scheduler;
  bool hover_motors_active;
  bool hover_motors_disable;
  bool force_forward;
  uint8_t preferred_pitch;
  bool stall_protection;
  float max_v_climb;
  float max_v_descend;
};

extern struct RotwingState rotwing_state;
extern struct RotWingStateSettings rotwing_state_settings;

extern bool rotwing_state_force_quad;

extern bool hover_motors_active;
extern bool bool_disable_hover_motors;

extern void init_rotwing_state(void);
extern void periodic_rotwing_state(void);
extern void request_rotwing_state(uint8_t state);

#endif  // ROTWING_STATE_H
