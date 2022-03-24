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

#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_isa.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "firmwares/rover/guidance/rover_guidance_steering.h"
#include "state.h"

// Check if rover firmware 
#ifndef ROVER_FIRMWARE
#error "The module nps_fdm_rover is designed for rovers and doesn't support other firmwares!!"
#endif

// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef;

// Static functions declaration
static void init_ltp(void);

/** Physical model structures **/
static struct EnuCoor_d rover_pos;
static struct EnuCoor_d rover_vel;
static struct EnuCoor_d rover_acc;

/** Physical model parameters **/
static float mu = 0.01;


/** NPS FDM rover init ***************************/
void nps_fdm_init(double dt)
{
  fdm.init_dt = dt; // (1 / simulation freq)
  fdm.curr_dt = 0.001; // ¿Configurable from GCS?
  fdm.time = dt;

  fdm.on_ground = TRUE;

  fdm.nan_count = 0;
  fdm.pressure = -1;
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.total_pressure = -1;
  fdm.dynamic_pressure = -1;
  fdm.temperature = -1;

  fdm.ltpprz_to_body_eulers.psi = 0.0;
  init_ltp();
}

void nps_fdm_run_step(bool launch __attribute__((unused)), double *commands, int commands_nb __attribute__((unused)))
{ 

  /****************************************************************************
  PHYSICAL MODEL
  -------------
  The physical model of your rover goes here. This physics takes place in
  the LTP plane (so we transfer every integration step to NED and ECEF at the end).
  */

  #ifdef COMMAND_STEERING // STEERING ROVER PHYSICS

  // Steering rover cmds: 
  //    COMMAND_STEERING -> delta parameter
  //    COMMAND_TRHOTTLE -> acceleration in heading direction

  double delta = RadOfDeg(commands[COMMAND_STEERING] * MAX_DELTA);

  /** Physical model for car-like robots .................. **/
  // From previous step...
  enu_of_ecef_point_d(&rover_pos, &ltpdef, &fdm.ecef_pos);
  enu_of_ecef_vect_d(&rover_vel, &ltpdef, &fdm.ecef_ecef_vel);
  double speed = FLOAT_VECT2_NORM(rover_vel);
  double phi = fdm.ltpprz_to_body_eulers.psi;
  double phi_d  = tan(delta) / DRIVE_SHAFT_DISTANCE * speed;

  // Setting accelerations
  rover_acc.x = commands[COMMAND_THROTTLE] * cos(phi) - speed * (sin(phi) * phi_d + cos(phi) * mu);
  rover_acc.y = commands[COMMAND_THROTTLE] * sin(phi) + speed * (cos(phi) * phi_d - sin(phi) * mu);
  double phi_dd = tan(delta) / DRIVE_SHAFT_DISTANCE * commands[COMMAND_THROTTLE];

  // Velocities (EULER INTEGRATION)
  rover_vel.x += rover_acc.x * fdm.curr_dt;
  rover_vel.y += rover_acc.y * fdm.curr_dt;
  phi_d  = tan(delta) / DRIVE_SHAFT_DISTANCE * speed;

  // Positions
  rover_pos.x += rover_vel.x * fdm.curr_dt;
  rover_pos.y += rover_vel.y * fdm.curr_dt;
  phi += phi_d * fdm.curr_dt;
  
  // phi have to be contained in [-180º,180º). So:
  phi = (phi > M_PI)? - 2*M_PI + phi : (phi < -M_PI)? 2*M_PI + phi : phi;

  #else
  #warning "The physics of this rover are not yet implemented in nps_fdm_rover!!"
  #endif // STEERING ROVER PHYSICS


  /****************************************************************************/
  // Coordenates transformations |
  // ----------------------------|

  /* in ECEF */
  ecef_of_enu_point_d(&fdm.ecef_pos, &ltpdef, &rover_pos);
  ecef_of_enu_vect_d(&fdm.ecef_ecef_vel, &ltpdef, &rover_vel);
  ecef_of_enu_vect_d(&fdm.ecef_ecef_accel, &ltpdef, &rover_acc);

  /* in NED */
  ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpdef, &fdm.ecef_pos);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_vel, &ltpdef, &fdm.ecef_ecef_vel);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_accel, &ltpdef, &fdm.ecef_ecef_accel);

  /* Eulers */
  fdm.ltpprz_to_body_eulers.psi = phi;

  // ENU to NED and exporting heading (psi euler angle)
  fdm.ltp_to_body_eulers.psi = - phi + M_PI / 2;

  // Exporting Eulers to AHRS (in quaternions)
  double_quat_of_eulers(&fdm.ltp_to_body_quat, &fdm.ltp_to_body_eulers);

  // Angular vel & acc
  fdm.body_ecef_rotvel.r   = phi_d; 
  fdm.body_ecef_rotaccel.r = phi_dd;

}


/**************************
 ** Generating LTP plane **
 **************************/

static void init_ltp(void)
{

  struct LlaCoor_d llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = RadOfDeg((double)NAV_LAT0 / 1e7);
  llh_nav0.lon = RadOfDeg((double)NAV_LON0 / 1e7);

  struct EcefCoor_d ecef_nav0;

  ecef_of_lla_d(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_d(&ltpdef, &ecef_nav0);
  fdm.ecef_pos = ecef_nav0;

  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 0.; // accel data are already with the correct format


#ifdef AHRS_H_X
  fdm.ltp_h.x = AHRS_H_X;
  fdm.ltp_h.y = AHRS_H_Y;
  fdm.ltp_h.z = AHRS_H_Z;
  PRINT_CONFIG_MSG("Using magnetic field as defined in airframe file.")
#else
  fdm.ltp_h.x = 0.4912;
  fdm.ltp_h.y = 0.1225;
  fdm.ltp_h.z = 0.8624;
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

