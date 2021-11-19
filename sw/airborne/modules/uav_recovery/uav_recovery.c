/*
 * Copyright (C) 2013-2020 Chris Efstathiou hendrixgr@gmail.com
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

/**
 * @file modules/uav_recovery/uav_recovery.c
 *
 */


#include "autopilot.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "modules/nav/common_nav.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "modules/intermcu/inter_mcu.h"
#include "modules/datalink/datalink.h"
#include "modules/multi/traffic_info.h"
#include "uav_recovery.h"

#if defined(USE_PARACHUTE) && USE_PARACHUTE == 1

#ifndef GLUE_DEFINITIONS_STEP2
#define GLUE_DEFINITIONS_STEP2(a, b)  (a##b)
#endif
#ifndef GLUE_DEFINITIONS
#define GLUE_DEFINITIONS(a, b)  GLUE_DEFINITIONS_STEP2(a, b)
#endif

#if defined(PARACHUTE_SERVO_CHANNEL) && PARACHUTE_SERVO_CHANNEL != -1
#define PARACHUTE_OUTPUT_COMMAND    GLUE_DEFINITIONS(COMMAND_, PARACHUTE_SERVO_CHANNEL)
#else
#warning YOU HAVE NOT DEFINED A PARACHUTE RELEASE AUTOPILOT SERVO
#endif

#endif

/** TERMINAL VELOCITY OF THE PARACHUTE PLUS PAYLOAD */
#ifndef PARACHUTE_TRIGGER_DELAY
#define PARACHUTE_TRIGGER_DELAY 2.
#endif

#ifndef PARACHUTE_DESCENT_RATE
#define PARACHUTE_DESCENT_RATE    3.0
#endif
#ifndef PARACHUTE_WIND_CORRECTION
#define PARACHUTE_WIND_CORRECTION 1.0
#endif
#ifndef PARACHUTE_LINE_LENGTH
#define PARACHUTE_LINE_LENGTH   3.0
#endif

float  parachute_start_qdr;
float  parachute_z;
float  airborne_wind_dir = 0;
float  airborne_wind_speed = 0;
float  calculated_wind_dir = 0;
bool   wind_measurements_valid = true;
bool   wind_info_valid = false;
bool   deploy_parachute_var = 0;
bool   land_direction = 0;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_wind_info(struct transport_tx *trans, struct link_device *dev)
{

  float foo1 = 0, foo2 = 0, foo3 = 0;
  pprz_msg_send_WEATHER(trans, dev, AC_ID, &foo1, &foo2, &airborne_wind_speed, &airborne_wind_dir, &foo3);

  return;
}
#endif

void uav_recovery_init(void)
{

  deploy_parachute_var = 0;
  airborne_wind_dir = 0;
  airborne_wind_speed = 0;
  wind_measurements_valid = true;
  wind_info_valid = true;
#if defined(PARACHUTE_OUTPUT_COMMAND)
  ap_state->commands[PARACHUTE_OUTPUT_COMMAND] = MIN_PPRZ;
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WEATHER, send_wind_info);
#endif


  return;
}


void uav_recovery_periodic(void)
{
#if defined(PARACHUTE_OUTPUT_COMMAND)
  if (autopilot.mode != AP_MODE_MANUAL) {
    if (deploy_parachute_var == 1) {
      ap_state->commands[PARACHUTE_OUTPUT_COMMAND] = MAX_PPRZ;

    } else { ap_state->commands[PARACHUTE_OUTPUT_COMMAND] = MIN_PPRZ; }
  }
#else
#warning PARACHUTE COMMAND NOT FOUND
#endif

  return;
}

uint8_t LockParachute(void)
{

  deploy_parachute_var = 0;

  return (0);
}

uint8_t DeployParachute(void)
{

  deploy_parachute_var = 1;

  return (0);
}


uint8_t calculate_wind_no_airspeed(uint8_t wp, float radius, float height)
{

//struct EnuCoor_f* pos = stateGetPositionEnu_f();
//struct UtmCoor_f * pos = stateGetPositionUtm_f();

  static bool init = false;
  static float circle_count = 0;
  static bool wind_measurement_started = false;

//Legacy estimator values
  float estimator_hspeed_dir = 0;
  float estimator_hspeed_mod = 0;
  estimator_hspeed_dir = stateGetHorizontalSpeedDir_f();
  estimator_hspeed_mod = stateGetHorizontalSpeedNorm_f();

  float speed_max_buf = 0;
  float speed_min_buf = 1000.0;
  float heading_angle_buf = 0;

  if (init == false) {
    airborne_wind_dir = 0;
    airborne_wind_speed = 0;
    wind_measurements_valid = false;
    speed_max_buf = 0;
    speed_min_buf = 1000.;
    heading_angle_buf = 0;
    wind_measurement_started = false;
    init = true;
  }

  NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
  NavVerticalAltitudeMode(Height(height), 0.);
  NavCircleWaypoint(wp, radius);
  if (nav_in_circle == false || GetPosAlt() < Height(height - 10)) {
    return (true);
  }
  NavVerticalThrottleMode(MAX_PPRZ * (float)V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE);
  // Wait until we are in altitude and in circle.
  if (wind_measurement_started == false) {
    circle_count = NavCircleCount();
    wind_measurement_started = true;
  }
  if (estimator_hspeed_mod > speed_max_buf) {
    speed_max_buf = estimator_hspeed_mod;
  }
  // WHEN THE GROUND SPEED IS LOWEST THIS IS WHERE THE WIND IS BLOWING FROM.
  if (estimator_hspeed_mod < speed_min_buf) {
    speed_min_buf = estimator_hspeed_mod;
    heading_angle_buf = estimator_hspeed_dir; //180 to -180 in radians, 0 is to the NORTH!
  }

  if ((NavCircleCount() - circle_count) >= 1.1) {
    //Update the wind direction and speed in the WEATHER message.
    airborne_wind_speed = ((speed_max_buf - speed_min_buf) / 2) * AIRBORNE_WIND_CORRECTION;
#if defined(FIXED_WIND_SPEED_FOR_TESTING) && defined(SITL)
#pragma message "Wind SPEED for UAV recovery is fixed for the simulation"
#pragma message "You can change the value by editing 'uav_recovery.xml' file."
    airborne_wind_speed = FIXED_WIND_SPEED_FOR_TESTING; // FOR TESTING IN SIMULATION ONLY!
#endif
#if defined(FIXED_WIND_DIRECTION_FOR_TESTING) && defined(SITL)
#pragma message "Wind direction for UAV recovery is fixed for the simulation"
#pragma message "You can change the value by editing 'uav_recovery.xml' file."
    if (FIXED_WIND_DIRECTION_FOR_TESTING > 180.) {
      heading_angle_buf = RadOfDeg(FIXED_WIND_DIRECTION_FOR_TESTING - 360.);
    } else {
      heading_angle_buf = RadOfDeg(FIXED_WIND_DIRECTION_FOR_TESTING);
    }
#endif
    airborne_wind_dir = DegOfRad(heading_angle_buf); //For use with the WEATHER message
    //Convert from 180, -180 to 0, 360 in Degrees. 0=NORTH, 90=EAST
    if (airborne_wind_dir < 0) { airborne_wind_dir += 360; }
    calculated_wind_dir = heading_angle_buf;
    wind_measurements_valid = true;
    wind_info_valid = true;

    //NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    // EXIT and continue the flight plan.
    init = false;
    return (false);
  }

  return (true);
}


// Compute an approximation for the RELEASE waypoint from expected descent and altitude
unit_t parachute_compute_approach(uint8_t baseleg, uint8_t release, uint8_t wp_target)
{

  float x = 0, y = 0;
  float approach_dir = 0;
  float calculated_wind_east = 0;
  float calculated_wind_north = 0;

  struct {
    float x;
    float y;
    float a;
  } start_wp;

  // Calculate a starting waypoint opposite to the wind.
  //180 TO -180 in Radians. Convert from 0 = NORTH to 0 = EAST
  approach_dir = RadOfDeg(90.) - calculated_wind_dir;
  if (approach_dir > M_PI) { approach_dir -= 2 * M_PI; }
  approach_dir -= M_PI; // reverse the direction as we must release the parachute against the wind!

  start_wp.x = (cos(approach_dir) * nav_radius) + waypoints[wp_target].x;
  start_wp.y = (sin(approach_dir) * nav_radius) + waypoints[wp_target].y;
  start_wp.a = waypoints[release].a;

  // We approximate vx and vy, taking into account that RELEASE is close to the TARGET
  float x_0 = waypoints[wp_target].x - start_wp.x;
  float y_0 = waypoints[wp_target].y - start_wp.y;

  // Unit vector from the calculated starting waypoint to TARGET
  float d = sqrt(x_0 * x_0 + y_0 * y_0);
  float x1 = x_0 / d;
  float y_1 = y_0 / d;

  waypoints[baseleg].x = start_wp.x + y_1 * nav_radius;
  waypoints[baseleg].y = start_wp.y - x1 * nav_radius;
  waypoints[baseleg].a = start_wp.a;
  parachute_start_qdr = M_PI - atan2(-y_1, -x1);
  if (nav_radius < 0) {
    parachute_start_qdr += M_PI;
  }

  parachute_z = waypoints[release].a - ground_alt - PARACHUTE_LINE_LENGTH;

  calculated_wind_north = cosf(calculated_wind_dir) * airborne_wind_speed;
  calculated_wind_east = sinf(calculated_wind_dir) * airborne_wind_speed;
  x = (parachute_z / PARACHUTE_DESCENT_RATE) * calculated_wind_east * PARACHUTE_WIND_CORRECTION;
  y = (parachute_z / PARACHUTE_DESCENT_RATE) * calculated_wind_north * PARACHUTE_WIND_CORRECTION;

  waypoints[release].x = (waypoints[wp_target].x + x);
  waypoints[release].y = waypoints[wp_target].y + y;


  return 0;
}

