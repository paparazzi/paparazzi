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

/** @file "modules/rotwing_drone/rotwing_automation.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Fucntions to automate the navigation and guidance of the rotating wing drone
 */

#include "modules/rotwing_drone/rotwing_automation.h"
#include "state.h"
#include "modules/datalink/telemetry.h"
#include "filters/low_pass_filter.h"
#include "state.h"

#include "math/pprz_algebra_float.h"
#include "modules/nav/waypoints.h"
#include "generated/modules.h"

/*** Longitudinal maximum acceleration during a transition */
#ifndef ROTWING_AUTOMATION_TRANS_ACCEL
#define ROTWING_AUTOMATION_TRANS_ACCEL 1.0
#endif

/*** Longitudinal maximum deceleration during a transition */
#ifndef ROTWING_AUTOMATION_TRANS_DECEL
#define ROTWING_AUTOMATION_TRANS_DECEL 0.5
#endif

/*** Maximum transition distance (at which to draw waypoints) */
#ifndef ROTWING_AUTOMATION_TRANS_LENGTH
#define ROTWING_AUTOMATION_TRANS_LENGTH 200.0
#endif

/*** Airspeed threshold above which the  transiton is considered complete */
#ifndef ROTWING_AUTOMATION_TRANS_AIRSPEED
#define ROTWING_AUTOMATION_TRANS_AIRSPEED 15.0
#endif

/*** Wind low-pass filtering cutoff frequency */
#ifndef ROTWING_AUTOMATION_WIND_FILT_CUTOFF
#define ROTWING_AUTOMATION_WIND_FILT_CUTOFF 0.001
#endif

struct rotwing_automation rotwing_a;

// Declare filters (for windspeed estimation)
Butterworth2LowPass rotwing_automation_wind_filter[2]; // Wind filter

// declare function
inline void update_waypoint_rotwing_automation(uint8_t wp_id, struct FloatVect3 *target_ned);
inline void update_wind_vector(void);

void init_rotwing_automation(void)
{
  rotwing_a.trans_accel = ROTWING_AUTOMATION_TRANS_ACCEL;
  rotwing_a.trans_decel = ROTWING_AUTOMATION_TRANS_DECEL;
  rotwing_a.trans_length = ROTWING_AUTOMATION_TRANS_LENGTH;
  rotwing_a.trans_airspeed = ROTWING_AUTOMATION_TRANS_AIRSPEED;

  rotwing_a.transitioned = false;
  rotwing_a.windvect.x = 0.0;
  rotwing_a.windvect.y = 0.0;
  rotwing_a.windvect_f.x = 0.0;
  rotwing_a.windvect_f.y = 0.0;

  // Init windvector low pass filter
  float tau = 1.0 / (2.0 * M_PI * ROTWING_AUTOMATION_WIND_FILT_CUTOFF);
  float sample_time = 1.0 / PERIODIC_ROTWING_AUTOMATION_FREQ;
  init_butterworth_2_low_pass(&rotwing_automation_wind_filter[0], tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&rotwing_automation_wind_filter[1], tau, sample_time, 0.0);
}

// periodic function
void periodic_rotwing_automation(void)
{
  update_wind_vector();
  float airspeed = stateGetAirspeed_f();
  if (airspeed > rotwing_a.trans_airspeed) {
    rotwing_a.transitioned = true;
  } else {
    rotwing_a.transitioned = false;
  }
}

// Update a waypoint such that you can see on the GCS where the drone wants to go
void update_waypoint_rotwing_automation(uint8_t wp_id, struct FloatVect3 *target_ned)
{

  // Update the waypoint
  struct EnuCoor_f target_enu;
  ENU_OF_TO_NED(target_enu, *target_ned);
  waypoint_set_enu(wp_id, &target_enu);

  // Send waypoint update roughly every half second
  RunOnceEvery(100 / 2, {
    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                               &waypoints[wp_id].enu_i.x,
                               &waypoints[wp_id].enu_i.y,
                               &waypoints[wp_id].enu_i.z);
  });
}

void update_wind_vector(void)
{
  float psi = stateGetNedToBodyEulers_f()->psi;

  float cpsi = cosf(psi);
  float spsi = sinf(psi);

  float airspeed = stateGetAirspeed_f();
  struct NedCoor_f *groundspeed = stateGetSpeedNed_f();
  struct FloatVect2 airspeed_v = { cpsi * airspeed, spsi * airspeed };
  VECT2_DIFF(rotwing_a.windvect, *groundspeed, airspeed_v);

  // Filter the wind
  rotwing_a.windvect_f.x = update_butterworth_2_low_pass(&rotwing_automation_wind_filter[0], rotwing_a.windvect.x);
  rotwing_a.windvect_f.x = update_butterworth_2_low_pass(&rotwing_automation_wind_filter[1], rotwing_a.windvect.y);
}

// Function to visualize from flightplan, call repeadely
void rotwing_vis_transition(uint8_t wp_transition_id, uint8_t wp_decel_id, uint8_t wp_end_id)
{
  //state eulers in zxy order
  struct FloatEulers eulers_zxy;

  // get heading and cos and sin of heading
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  float psi = eulers_zxy.psi;
  float cpsi = cosf(psi);
  float spsi = sinf(psi);

  // get airspeed
  float airspeed = stateGetAirspeed_f();
  // get groundspeed
  float ground_speed = stateGetHorizontalSpeedNorm_f();

  // get drone position
  struct NedCoor_f *drone_pos = stateGetPositionNed_f();

  // Move reference waypoints
  // Move end transition waypoint at the correct length
  struct FloatVect3 end_transition_rel_pos;
  VECT3_COPY(end_transition_rel_pos, *drone_pos);
  end_transition_rel_pos.x = cpsi * rotwing_a.trans_length;
  end_transition_rel_pos.y = spsi * rotwing_a.trans_length;
  struct FloatVect3 end_transition_pos;
  VECT3_SUM(end_transition_pos, end_transition_rel_pos, *drone_pos);
  end_transition_pos.z = drone_pos->z;
  update_waypoint_rotwing_automation(wp_end_id, &end_transition_pos);

  // Move transition waypoint
  float airspeed_error = rotwing_a.trans_airspeed - airspeed;
  float transition_time = airspeed_error / rotwing_a.trans_accel;
  float average_ground_speed = ground_speed + airspeed_error / 2.;
  float transition_distance = average_ground_speed * transition_time;

  struct FloatVect3 transition_rel_pos;
  VECT3_COPY(transition_rel_pos, *drone_pos);
  transition_rel_pos.x = cpsi * transition_distance;
  transition_rel_pos.y = spsi * transition_distance;
  struct FloatVect3 transition_pos;
  VECT3_SUM(transition_pos, transition_rel_pos, *drone_pos);
  transition_pos.z = drone_pos->z;
  update_waypoint_rotwing_automation(wp_transition_id, &transition_pos);

  // Move decel point
  float final_groundspeed = ground_speed + airspeed_error;
  float decel_time = final_groundspeed / rotwing_a.trans_decel;
  float decel_distance = (final_groundspeed / 2.) * decel_time;
  float decel_distance_from_drone = rotwing_a.trans_length - decel_distance;

  struct FloatVect3 decel_rel_pos;
  VECT3_COPY(decel_rel_pos, *drone_pos);
  decel_rel_pos.x = cpsi * decel_distance_from_drone;
  decel_rel_pos.y = spsi * decel_distance_from_drone;
  struct FloatVect3 decel_pos;
  VECT3_SUM(decel_pos, decel_rel_pos, *drone_pos);
  decel_pos.z = drone_pos->z;
  update_waypoint_rotwing_automation(wp_decel_id, &decel_pos);
}
