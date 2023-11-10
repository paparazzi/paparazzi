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

/** @file "modules/rot_wing_drone/rot_wing_automation.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Fucntions to automate the navigation and guidance of the rotating wing drone
 */

#ifndef ROT_WING_AUTOMATION_H
#define ROT_WING_AUTOMATION_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "modules/rot_wing_drone/rotwing_state.h"

extern void init_rot_wing_automation(void);
extern void periodic_rot_wing_automation(void);
extern void rot_wing_vis_transition(uint8_t wp_transition_id, uint8_t wp_decel_id, uint8_t wp_end_id);

struct rot_wing_automation {
  float trans_accel;
  float trans_decel;
  float trans_length;
  float trans_airspeed;
  bool transitioned;
};

extern struct rot_wing_automation rot_wing_a;

#define RotWingAutomationReadyForForward()  (rot_wing_a.transitioned && (rotwing_state_skewing.wing_angle_deg > 75.0))


#endif  // ROT_WING_AUTOMATION_H
