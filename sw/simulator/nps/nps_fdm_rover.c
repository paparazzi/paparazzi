/*
 * Copyright (C) 2021 Jesús Bautista <jesusbautistavillar@gmail.com>
 *                    Hector García  <noeth3r@gmail.com>
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

#include "nps_fdm.h"

#include <stdlib.h>
#include <stdio.h>

#include "std.h"
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_isa.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "generated/modules.h"
#include "state.h"

// Check if rover firmware
#ifndef ROVER_FIRMWARE
#error "The module nps_fdm_rover is designed for rovers and doesn't support other firmwares!!"
#endif

#if (defined COMMAND_STEERING) && (defined COMMAND_THROTTLE) // STEERING ROVER PHYSICS

// Friction coef, proportional to speed
#ifndef NPS_ROVER_FRICTION
#define NPS_ROVER_FRICTION 0.01f
#endif

//// Used for car-like rover (steering wheels)

// Maximum acceleration (m/s²)
#ifndef NPS_ROVER_ACCELERATION
#define NPS_ROVER_ACCELERATION 1.f
#endif

/** Physical model parameters **/
static float mu = NPS_ROVER_FRICTION;

#endif

//// Used for 2-wheels rover

#if (defined COMMAND_TURN) && (defined COMMAND_SPEED) // 2 wheels rover dynamic

// Maximum speed (m/s)
#ifndef NPS_ROVER_MAX_SPEED
#define NPS_ROVER_MAX_SPEED 2.f
#endif

// Maximum turn rate (rad/s)
#ifndef NPS_ROVER_TURN_RATE
#define NPS_ROVER_TURN_RATE 3.f
#endif

// Accel 1st order time constant
#ifndef NPS_ROVER_ACCEL_TAU
#define NPS_ROVER_ACCEL_TAU 0.2f
#endif

// Turn rate 1st order time constant
#ifndef NPS_ROVER_TURN_TAU
#define NPS_ROVER_TURN_TAU 0.1f
#endif

#include "filters/low_pass_filter.h"
static struct FirstOrderLowPass speed_filter;
static struct FirstOrderLowPass turn_filter;

#endif // 2 wheels

// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef;

// Static functions declaration
static void init_ltp(void);

/** Physical model structures **/
static struct NedCoor_d rover_pos;
static struct NedCoor_d rover_vel;
static struct NedCoor_d rover_acc;


/** NPS FDM rover init ***************************/
void nps_fdm_init(double dt)
{
  fdm.init_dt = dt; // (1 / simulation freq)
  fdm.curr_dt = dt;
  fdm.time = 0;

  fdm.on_ground = TRUE;
  fdm.nan_count = 0;
  fdm.pressure = -1;
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.total_pressure = -1;
  fdm.dynamic_pressure = -1;
  fdm.temperature = -1;
  fdm.ltpprz_to_body_eulers.psi = 0.0;
  init_ltp();

#if (defined COMMAND_TURN) && (defined COMMAND_SPEED) // 2 wheels rover dynamic
  init_first_order_low_pass(&speed_filter, NPS_ROVER_ACCEL_TAU, fdm.curr_dt, 0.f);
  init_first_order_low_pass(&turn_filter, NPS_ROVER_TURN_TAU, fdm.curr_dt, 0.f);
#endif
}

void nps_fdm_run_step(bool launch __attribute__((unused)), double *commands, int commands_nb __attribute__((unused)))
{
  fdm.time += fdm.curr_dt;

  /****************************************************************************
  PHYSICAL MODEL
  -------------
  The physical model of your rover goes here. This physics takes place in
  the LTP plane (so we transfer every integration step to NED and ECEF at the end).
  */

  // From previous step...
  ned_of_ecef_point_d(&rover_pos, &ltpdef, &fdm.ecef_pos);
  ned_of_ecef_vect_d(&rover_vel, &ltpdef, &fdm.ecef_ecef_vel);
  double phi = fdm.ltpprz_to_body_eulers.psi;

  #if (defined COMMAND_STEERING) && (defined COMMAND_THROTTLE) // STEERING ROVER PHYSICS

  // Steering rover cmds:
  //    COMMAND_STEERING -> delta parameter
  //    COMMAND_THROTTLE -> acceleration in heading direction

  /** Physical model for car-like robots .................. **/
  double delta = RadOfDeg(commands[COMMAND_STEERING] * MAX_DELTA);
  double speed = FLOAT_VECT2_NORM(rover_vel);
  double phi_d = tan(delta) / DRIVE_SHAFT_DISTANCE * speed;
  double accel = NPS_ROVER_ACCELERATION * commands[COMMAND_THROTTLE];

  // NED accel = R(psi) [accel; |V|*phi_d] - mu*V
  rover_acc.x = accel * cos(phi) - speed * phi_d * sin(phi) - mu * rover_vel.x;
  rover_acc.y = accel * sin(phi) + speed * phi_d * cos(phi) - mu * rover_vel.y;
  // Velocities (EULER INTEGRATION)
  rover_vel.x += rover_acc.x * fdm.curr_dt;
  rover_vel.y += rover_acc.y * fdm.curr_dt;

  #elif (defined COMMAND_TURN) && (defined COMMAND_SPEED) // 2 wheels rover dynamic

  double phi_sp = NPS_ROVER_TURN_RATE * commands[COMMAND_TURN];
  double phi_d = update_first_order_low_pass(&turn_filter, phi_sp);
  double speed_sp = NPS_ROVER_MAX_SPEED * commands[COMMAND_SPEED];
  double speed = update_first_order_low_pass(&speed_filter, speed_sp);
  double vx = speed * cos(phi);
  double vy = speed * sin(phi);
  rover_acc.x = (vx - rover_vel.x) / fdm.curr_dt;
  rover_acc.y = (vy - rover_vel.y) / fdm.curr_dt;
  rover_vel.x = vx;
  rover_vel.y = vy;

  #else
  #warning "The physics of this rover are not yet implemented in nps_fdm_rover!!"
  #endif // STEERING ROVER PHYSICS

  // Positions
  rover_pos.x += rover_vel.x * fdm.curr_dt;
  rover_pos.y += rover_vel.y * fdm.curr_dt;
  // phi have to be contained in [-180º,180º). So:
  phi += phi_d * fdm.curr_dt;
  NormRadAngle(phi);
  /****************************************************************************/
  // Coordenates transformations |
  // ----------------------------|

  /* in ECEF */
  ecef_of_ned_point_d(&fdm.ecef_pos, &ltpdef, &rover_pos);
  ecef_of_ned_vect_d(&fdm.ecef_ecef_vel, &ltpdef, &rover_vel);
  ecef_of_ned_vect_d(&fdm.ecef_ecef_accel, &ltpdef, &rover_acc);

  /* in NED */
  ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpdef, &fdm.ecef_pos);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_vel, &ltpdef, &fdm.ecef_ecef_vel);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_accel, &ltpdef, &fdm.ecef_ecef_accel);

  /* Height above sea level */
  fdm.hmsl = ltpdef.hmsl - rover_pos.z;

  /* Eulers */
  fdm.ltpprz_to_body_eulers.psi = phi;
  fdm.ltp_to_body_eulers.psi = phi;

  // Exporting Eulers to AHRS (in quaternions)
  double_quat_of_eulers(&fdm.ltp_to_body_quat, &fdm.ltp_to_body_eulers);

  // Angular vel & acc
  fdm.body_ecef_rotaccel.r = (phi_d - fdm.body_ecef_rotvel.r) / fdm.curr_dt;
  fdm.body_ecef_rotvel.r   = phi_d;

}


/**************************
 ** Generating LTP plane **
 **************************/

static void init_ltp(void)
{
  struct LlaCoor_d llh_nav0;
  llh_nav0.lat = RadOfDeg((double)NAV_LAT0 / 1e7);
  llh_nav0.lon = RadOfDeg((double)NAV_LON0 / 1e7);
  llh_nav0.alt = (double)(NAV_ALT0 + NAV_MSL0) / 1000.; /* Height above the ellipsoid */

  struct EcefCoor_d ecef_nav0;
  ecef_of_lla_d(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_d(&ltpdef, &ecef_nav0);
  ltpdef.hmsl = (double)NAV_ALT0 / 1000.; /* Height above the geoid */

  fdm.ecef_pos = ecef_nav0;
  fdm.hmsl = ltpdef.hmsl;
  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 0.; // accel data are already with the correct format


#ifdef AHRS_H_X
  fdm.ltp_h.x = AHRS_H_X;
  fdm.ltp_h.y = AHRS_H_Y;
  fdm.ltp_h.z = AHRS_H_Z;
  PRINT_CONFIG_MSG("Using magnetic field as defined in airframe file.")
#else
  fdm.ltp_h.x = 1.;
  fdm.ltp_h.y = 0.;
  fdm.ltp_h.z = 0.;
#endif

}


/*****************************************************/
// Atmosphere function (we don't need that features) //
void nps_fdm_set_wind(double speed __attribute__((unused)),
                      double dir __attribute__((unused)))
{
}

void nps_fdm_set_wind_ned(double wind_north __attribute__((unused)),
                          double wind_east __attribute__((unused)),
                          double wind_down __attribute__((unused)))
{
}

void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
                            int turbulence_severity __attribute__((unused)))
{
}

void nps_fdm_set_temperature(double temp __attribute__((unused)),
                             double h __attribute__((unused)))
{
}

