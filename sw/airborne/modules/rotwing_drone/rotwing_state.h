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

/* Magnitude skew angle jump away from quad */
#ifndef ROTWING_SKEW_ANGLE_STEP
#define ROTWING_SKEW_ANGLE_STEP 55.0
#endif

/* Preferred pitch angle for the quad mode (deg) */
#ifndef ROTWING_QUAD_PREF_PITCH
#define ROTWING_QUAD_PREF_PITCH -5.0
#endif

enum rotwing_states_t {
  ROTWING_STATE_FORCE_HOVER,
  ROTWING_STATE_REQUEST_HOVER,
  ROTWING_STATE_FORCE_FW,
  ROTWING_STATE_REQUEST_FW,
  ROTWING_STATE_FREE,
};

union rotwing_bitmask_t {
  uint16_t value;
  struct {
    bool skew_angle_valid : 1;      // Skew angle didn't timeout
    bool hover_motors_enabled : 1;  // Hover motors command is enabled
    bool hover_motors_idle : 1;     // Hover motors are idling (throttle < IDLE_THROTTLE)
    bool hover_motors_running : 1;  // Hover motors are running (RPM >= MIN_RPM)
    bool pusher_motor_running : 1;  // Pusher motor is running (RPM >= MIN_RPM)
    bool skew_forced : 1;           // Skew angle is forced
  };
};

struct rotwing_state_t {
  /* Control */
  enum rotwing_states_t state;      // Current state
  enum rotwing_states_t nav_state;  // Desired navigation state (requested only by NAV and can be overruled by RC)
  bool hover_motors_enabled;        // Hover motors enabled (> idle throttle)

  /* Skew */
  float sp_skew_angle_deg;        // Setpoint skew angle in degrees
  float ref_model_skew_angle_deg; // Reference model skew angle in degrees
  float meas_skew_angle_deg;      // Measured skew angle in degrees
  float meas_skew_angle_time;     // Time of the last skew angle measurement
  bool force_skew;                // Force skew angle to a certain value by the GCS
  int16_t skew_cmd;               // Skewing command in pprz values

  /* Airspeeds */
  float fw_min_airspeed;      // Minimum airspeed (stall+margin)
  float cruise_airspeed;      // Airspeed for cruising
  float min_airspeed;         // Minimum airspeed for bounding
  float max_airspeed;         // Maximum airspeed for bounding

  /* RPM measurements*/
  int32_t meas_rpm[5];        // Measured RPM of the hover and pusher motors
  float meas_rpm_time[5];     // Time of the last RPM measurement

  /* Sim failures */
  bool fail_skew_angle;       // Skew angle sensor failure
  bool fail_hover_motor;      // Hover motor failure
  bool fail_pusher_motor;     // Pusher motor failure
};
extern struct rotwing_state_t rotwing_state;

void rotwing_state_init(void);
void rotwing_state_periodic(void);
bool rotwing_state_hover_motors_running(void);
bool rotwing_state_pusher_motor_running(void);
bool rotwing_state_skew_angle_valid(void);
bool rotwing_state_hover_motors_idling(void);

void rotwing_state_set(enum rotwing_states_t state);
bool rotwing_state_choose_circle_direction(uint8_t wp_id);
void rotwing_state_set_transition_wp(uint8_t wp_id);
void rotwing_state_update_WP_height(uint8_t wp_id, float height);

#endif  // ROTWING_STATE_H
