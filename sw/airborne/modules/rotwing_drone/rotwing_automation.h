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

/** @file "modules/rotwing_drone/rotwing_automation.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Fucntions to automate the navigation and guidance of the rotating wing drone
 */

#ifndef ROTWING_AUTOMATION_H
#define ROTWING_AUTOMATION_H

#include "std.h"
#include "math/pprz_algebra_float.h"

#ifdef RW_USE_MODULES_V2
#include "modules/rotwing_drone/rotwing_state_V2.h"
#else
#include "modules/rotwing_drone/rotwing_state.h"
#endif

extern void init_rotwing_automation(void);
extern void periodic_rotwing_automation(void);
extern void rotwing_vis_transition(uint8_t wp_transition_id, uint8_t wp_decel_id, uint8_t wp_end_id);

struct rotwing_automation {
  // Constants
  float trans_accel;            // Acceleration during transition
  float trans_decel;            // Deceleration during transition
  float trans_length;           // Transition length
  float trans_airspeed;         // Transition airspeed

  // Variables
  bool transitioned;            // Boolean that indicates if drone flies faster than transion airspeed
  struct FloatVect2 windvect;   // Wind vector
  struct FloatVect2 windvect_f; // Filtered wind vector
};

extern struct rotwing_automation rotwing_a;

#define RotWingAutomationReadyForForward()  (rotwing_a.transitioned && (rotwing_state_skewing.wing_angle_deg > 75.0))


#endif  // ROTWING_AUTOMATION_H
