/*
 * Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file firmwares/rotorcraft/guidance/guidance_plane.c
 *  Guidance controller for planes with PID
 *  compatible with the rotorcraft firmware
 *  no airspeed control
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_plane.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/nav/waypoints.h"
#include "state.h"

#ifndef GUIDANCE_PLANE_MAX_BANK
#define GUIDANCE_PLANE_MAX_BANK RadOfDeg(45.f)
#endif

#ifndef GUIDANCE_PLANE_MAX_PITCH
#define GUIDANCE_PLANE_MAX_PITCH RadOfDeg(30.f)
#endif

#ifndef GUIDANCE_PLANE_MIN_PITCH
#define GUIDANCE_PLANE_MIN_PITCH RadOfDeg(-20.f)
#endif

#ifndef GUIDANCE_PLANCE_COURSE_PRE_BANK
#define GUIDANCE_PLANCE_COURSE_PRE_BANK 1.f
#endif

#ifndef GUIDANCE_PLANE_MAX_CLIMB
#define GUIDANCE_PLANE_MAX_CLIMB 2.f
#endif

#ifndef GUIDANCE_PLANE_PITCH_OF_VZ
#define GUIDANCE_PLANE_PITCH_OF_VZ RadOfDeg(5.f)
#endif

#ifndef GUIDANCE_PLANE_PITCH_TRIM
#define GUIDANCE_PLANE_PITCH_TRIM RadOfDeg(0.f)
#endif

#ifndef GUIDANCE_PLANE_CLIMB_THROTTLE_INCREMENT
#define GUIDANCE_PLANE_CLIMB_THROTTLE_INCREMENT 0.1f
#endif

/*
 * external variables
 */

struct GuidancePlane guidance_plane;

/*
 * internal variables
 */

static float pitch_sum_err;
#define GUIDANCE_PLANE_PITCH_MAX_SUM_ERR (RadOfDeg(10.))

static float throttle_sum_err;
#define GUIDANCE_PLANE_THROTTLE_MAX_SUM_ERR 0.4

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

#endif

void guidance_plane_init(void)
{
  guidance_plane.roll_max_setpoint = GUIDANCE_PLANE_MAX_BANK;
  guidance_plane.course_setpoint = 0.f;
  guidance_plane.course_kp = GUIDANCE_PLANE_COURSE_KP;
  guidance_plane.course_kd = GUIDANCE_PLANE_COURSE_KD;
  guidance_plane.course_pre_bank_correction = GUIDANCE_PLANCE_COURSE_PRE_BANK;
  guidance_plane.roll_cmd = 0.f;

  guidance_plane.pitch_max_setpoint = GUIDANCE_PLANE_MAX_PITCH;
  guidance_plane.pitch_min_setpoint = GUIDANCE_PLANE_MIN_PITCH;
  guidance_plane.climb_max_setpoint = GUIDANCE_PLANE_MAX_CLIMB;
  guidance_plane.altitude_setpoint = 0.f;
  guidance_plane.climb_setpoint = 0.f;
  guidance_plane.climb_kp = GUIDANCE_PLANE_CLIMB_KP;
  guidance_plane.pitch_trim = GUIDANCE_PLANE_PITCH_TRIM;
  guidance_plane.pitch_of_vz = GUIDANCE_PLANE_PITCH_OF_VZ;
  guidance_plane.climb_throttle_increment = GUIDANCE_PLANE_CLIMB_THROTTLE_INCREMENT;
  guidance_plane.p_kp = GUIDANCE_PLANE_PITCH_KP;
  guidance_plane.p_kd = GUIDANCE_PLANE_PITCH_KD;
  guidance_plane.p_ki = GUIDANCE_PLANE_PITCH_KI;
  guidance_plane.t_kp = GUIDANCE_PLANE_THROTTLE_KP;
  guidance_plane.t_kd = GUIDANCE_PLANE_THROTTLE_KD;
  guidance_plane.t_ki = GUIDANCE_PLANE_THROTTLE_KI;
  guidance_plane.cruise_throttle = GUIDANCE_PLANE_NOMINAL_THROTTLE;
  guidance_plane.pitch_cmd = 0.f;
  guidance_plane.throttle_cmd = 0;

  pitch_sum_err = 0.f;
  throttle_sum_err = 0.f;

#if PERIODIC_TELEMETRY
#endif
}

/**
 * run horizontal control loop for position and speed control
 */
struct StabilizationSetpoint guidance_plane_attitude_from_nav(bool in_flight UNUSED)
{
  struct FloatEulers att_sp;

  // course control loop
  if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_NONE || nav.horizontal_mode == NAV_HORIZONTAL_MODE_GUIDED) {
    guidance_plane.roll_cmd = 0.f;
    guidance_plane.pitch_cmd = 0.f;
  } else if (nav.horizontal_mode == NAV_HORIZONTAL_MODE_ATTITUDE) {
    if (nav.setpoint_mode == NAV_SETPOINT_MODE_QUAT) {
      struct FloatEulers e;
      float_eulers_of_quat(&e, &nav.quat);
      guidance_plane.roll_cmd = e.phi;
      guidance_plane.pitch_cmd = e.theta;
    }
    else {
      guidance_plane.roll_cmd = nav.roll;
      guidance_plane.pitch_cmd = nav.pitch;
    }
  } else {
    // update carrot for GCS display and convert ENU float -> NED int
    // even if sp is changed later
    // FIXME really needed here ?
    guidance_h.sp.pos.x = POS_BFP_OF_REAL(nav.carrot.y);
    guidance_h.sp.pos.y = POS_BFP_OF_REAL(nav.carrot.x);

    switch (nav.setpoint_mode) {
      case NAV_SETPOINT_MODE_POS:
        guidance_plane.course_setpoint = atan2f(nav.carrot.x - stateGetPositionEnu_f()->x, nav.carrot.y - stateGetPositionNed_f()->y);
        break;
      case NAV_SETPOINT_MODE_SPEED:
        guidance_plane.course_setpoint = atan2f(nav.speed.x, nav.speed.y);
        break;
      case NAV_SETPOINT_MODE_ACCEL:
        // not supported, flying towards HOME waypoint
        guidance_plane.course_setpoint = atan2f(waypoint_get_x(WP_HOME) - stateGetPositionEnu_f()->x,
            waypoint_get_y(WP_HOME) - stateGetPositionEnu_f()->y);
        break;
      default:
        // nothing to do for other cases at the moment
        break;
    }
    /* final attitude setpoint */

    // Ground path error
    static float last_err = 0.f;
    float err = guidance_plane.course_setpoint - stateGetHorizontalSpeedDir_f();
    NormRadAngle(err);
    float d_err = err - last_err;
    last_err = err;
    NormRadAngle(d_err);
    //float course_pre_bank = 0.f; // TODO: compute if flying a circle -> pb is that it is coming from nav
    // guidance_plane.course_pre_bank_correction * course_pre_bank

    guidance_plane.roll_cmd = guidance_plane.course_kp * err + guidance_plane.course_kd * d_err;
    BoundAbs(guidance_plane.roll_cmd, guidance_plane.roll_max_setpoint);
  }

  att_sp.phi = guidance_plane.roll_cmd;
  att_sp.theta = guidance_plane.pitch_cmd;
  att_sp.psi = stateGetNedToBodyEulers_f()->psi;
  return stab_sp_from_eulers_f(&att_sp);
}


static void guidance_plane_set_pitch(bool in_flight)
{
  static float last_err = 0.f;

  if (!in_flight) {
    pitch_sum_err = 0.f;
  }

  // Compute errors
  float err = guidance_plane.climb_setpoint - stateGetSpeedEnu_f()->z;
  float d_err = err - last_err;
  last_err = err;

  if (guidance_plane.p_ki > 0.) {
    pitch_sum_err += err * (1. / PERIODIC_FREQUENCY);
    BoundAbs(pitch_sum_err, GUIDANCE_PLANE_PITCH_MAX_SUM_ERR / guidance_plane.p_ki);
  }

  // PID loop + feedforward ctl
  guidance_plane.pitch_cmd = nav.pitch
    + guidance_plane.pitch_trim
    + guidance_plane.pitch_of_vz * guidance_plane.climb_setpoint
    + guidance_plane.p_kp * err
    + guidance_plane.p_kd * d_err
    + guidance_plane.p_ki * pitch_sum_err;

}

static void guidance_plane_set_throttle(bool in_flight)
{
  static float last_err = 0.;

  if (!in_flight) {
    throttle_sum_err = 0;
  }

  // Compute errors
  float err =  guidance_plane.climb_setpoint - stateGetSpeedEnu_f()->z;
  float d_err = err - last_err;
  last_err = err;

  if (guidance_plane.t_ki > 0.) {
    throttle_sum_err += err * (1. / PERIODIC_FREQUENCY);
    BoundAbs(throttle_sum_err, GUIDANCE_PLANE_THROTTLE_MAX_SUM_ERR / guidance_plane.t_ki);
  }

  // PID loop + feedforward ctl
  float th_cmd = guidance_plane.cruise_throttle
    + guidance_plane.climb_throttle_increment * guidance_plane.climb_setpoint
    + guidance_plane.t_kp * err
    + guidance_plane.t_kd * d_err
    + guidance_plane.t_ki * throttle_sum_err;

  guidance_plane.throttle_cmd = TRIM_UPPRZ(MAX_PPRZ * th_cmd);

}

/**
 * run vertical control loop for position and speed control
 */
struct ThrustSetpoint guidance_plane_thrust_from_nav(bool in_flight)
{
  if (nav.vertical_mode == NAV_VERTICAL_MODE_MANUAL) {
    guidance_plane.throttle_cmd = (int32_t) nav.throttle;
  } else {
    if (nav.vertical_mode == NAV_VERTICAL_MODE_ALT) {
      guidance_plane.altitude_setpoint = nav.nav_altitude;
      float alt_err = guidance_plane.altitude_setpoint - stateGetPositionEnu_f()->z;
      guidance_plane.climb_setpoint = guidance_plane.climb_kp * alt_err;
    } else if (nav.vertical_mode == NAV_VERTICAL_MODE_CLIMB) {
      guidance_plane.climb_setpoint = nav.climb;
    } else {
      guidance_plane.climb_setpoint = 0.f;
    }
    BoundAbs(guidance_plane.climb_setpoint, guidance_plane.climb_max_setpoint);

    guidance_plane_set_pitch(in_flight);
    guidance_plane_set_throttle(in_flight);
  }

  return th_sp_from_thrust_i(guidance_plane.throttle_cmd, THRUST_AXIS_X);
}

void guidance_plane_enter(void)
{
  /* set nav_heading to current heading */
  //nav.heading = stateGetNedToBodyEulers_f()->psi;

  pitch_sum_err = 0.f;
  throttle_sum_err = 0.f;
}

