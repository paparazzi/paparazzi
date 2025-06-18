/*
 * Copyright (C) 2025 Noah Wechtler <noahwechtler@tudelft.nl>
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

/** @file "modules/rotwing/rotwing_demo.h"
 * @author Noah Wechtler <noahwechtler@tudelft.nl>
 * This module implements changes to the ground checks an RC for a rotwing demo.
 */

#include "modules/rotwing_drone/rotwing_demo.h"
#include "modules/rotwing_drone/rotwing_state.h"
#include "modules/radio_control/radio_control.h"
#include "modules/checks/preflight_checks.h"
#include "modules/checks/pfc_actuators.h"
#include "autopilot_rc_helpers.h"

#ifndef RADIO_CONTROL_ROTWING_DEMO
#warning "No RC switch defined for skew demo, please define a 3-way switch in the airframe file, under the ap target, with name RADIO_CONTROL_ROTWING_DEMO. E.g. <define name="RADIO_CONTROL_ROTWING_DEMO"   value="RADIO_AUX5"/>"
#endif

#ifndef ROTWING_DEMO_SKEW_MIDPOINT
#define ROTWING_DEMO_SKEW_MIDPOINT 45.0
#endif

#ifndef ROTWING_DEMO_SKEW_ENDPOINT
#define ROTWING_DEMO_SKEW_ENDPOINT 70.0
#endif

void rotwing_demo_periodic(void) {
  
  // If actuator checks are not yet completed and drone is armed, start the actuator checks.
  if (!kill_switch_is_on() && pfc_actuators_get_state() != PFC_ACTUATORS_STATE_SUCCESS && pfc_actuators_get_state() != PFC_ACTUATORS_STATE_RUNNING) {
    pfc_actuators_start(true);
  }

  // If actuator checks are running, and drone is disarmed again, stop the actuator checks.
  if (kill_switch_is_on() && pfc_actuators_get_state() == PFC_ACTUATORS_STATE_RUNNING) {
    pfc_actuators_start(false);
  }

  // If actuator checks are completed, and drone is disarmed again, bypass the preflight checks.
  if (kill_switch_is_on() && preflight_bypass == false && pfc_actuators_get_state() == PFC_ACTUATORS_STATE_SUCCESS) {
    preflight_bypass = true;
    preflight_checks_log_bypass(preflight_bypass);
  }

#ifdef RADIO_CONTROL_ROTWING_DEMO
  if (preflight_bypass == true) {
    rotwing_state.force_skew = true;
  } 

  // Set the skew angle based on the RC switch position
  if (rotwing_state.force_skew == true) {
    if (radio_control.values[RADIO_CONTROL_ROTWING_DEMO] < (MAX_PPRZ / 3)) {
      rotwing_state.sp_skew_angle_deg = 0.f;
    } else if (radio_control.values[RADIO_CONTROL_ROTWING_DEMO] > (2 * MAX_PPRZ / 3)) {
      rotwing_state.sp_skew_angle_deg = ROTWING_DEMO_SKEW_ENDPOINT;
    } else {
      rotwing_state.sp_skew_angle_deg = ROTWING_DEMO_SKEW_MIDPOINT;
    }
  }
#endif
}
