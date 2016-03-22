/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef NPS_FDM
#define NPS_FDM

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "../flight_gear.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_algebra_double.h"

/*
  Notations for fdm variables
  ---------------------------
  coordinate system [ frame ] name

  ecef_inertial_vel is the time derivative of position
  with respect to inertial frame expressed in ECEF ( Earth Centered Earth Fixed)
  coordinate system.
*/

struct NpsFdm {

  double time;
  double init_dt;
  double curr_dt;
  bool on_ground;
  int nan_count;

  /*  position */
  struct EcefCoor_d  ecef_pos;
  struct NedCoor_d ltpprz_pos;
  struct LlaCoor_d lla_pos;
  double hmsl;
  // for debugging
  struct LlaCoor_d lla_pos_pprz; //lla converted by pprz from ecef
  struct LlaCoor_d lla_pos_geod; //geodetic lla from jsbsim
  struct LlaCoor_d lla_pos_geoc; //geocentric lla from jsbsim
  double agl; //AGL from jsbsim in m

  /** velocity in ECEF frame, wrt ECEF frame */
  struct EcefCoor_d  ecef_ecef_vel;
  /** acceleration in ECEF frame, wrt ECEF frame */
  struct EcefCoor_d  ecef_ecef_accel;

  /** velocity in body frame, wrt ECEF frame */
  struct DoubleVect3 body_ecef_vel;   /* aka UVW */
  /** acceleration in body frame, wrt ECEF frame */
  struct DoubleVect3 body_ecef_accel;

  /** velocity in LTP frame, wrt ECEF frame */
  struct NedCoor_d ltp_ecef_vel;
  /** acceleration in LTP frame, wrt ECEF frame */
  struct NedCoor_d ltp_ecef_accel;

  /** velocity in ltppprz frame, wrt ECEF frame */
  struct NedCoor_d ltpprz_ecef_vel;
  /** accel in ltppprz frame, wrt ECEF frame */
  struct NedCoor_d ltpprz_ecef_accel;

  /** acceleration in body frame, wrt ECI inertial frame */
  struct DoubleVect3 body_inertial_accel;

  /** acceleration in body frame as measured by an accelerometer (incl. gravity) */
  struct DoubleVect3 body_accel;

  /* attitude */
  struct DoubleQuat   ecef_to_body_quat;
  struct DoubleQuat   ltp_to_body_quat;
  struct DoubleEulers ltp_to_body_eulers;
  struct DoubleQuat   ltpprz_to_body_quat;
  struct DoubleEulers ltpprz_to_body_eulers;

  /*  angular velocity and acceleration in body frame, wrt ECEF frame */
  struct DoubleRates  body_ecef_rotvel;
  struct DoubleRates  body_ecef_rotaccel;

  /*  angular velocity and acceleration in body frame, wrt inertial ECI frame */
  struct DoubleRates  body_inertial_rotvel;
  struct DoubleRates  body_inertial_rotaccel;

  struct DoubleVect3 ltp_g;
  struct DoubleVect3 ltp_h;

  struct DoubleVect3 wind; ///< velocity in m/s in NED

  double airspeed;         ///< equivalent airspeed in m/s
  double pressure;         ///< current (static) atmospheric pressure in Pascal
  double total_pressure;   ///< total atmospheric pressure in Pascal
  double dynamic_pressure; ///< dynamic pressure in Pascal
  double temperature;      ///< current temperature in degrees Celcius
  double pressure_sl;      ///< pressure at sea level in Pascal

  // Control surface positions (normalized values)
  float elevator;
  float flap;
  float left_aileron;
  float right_aileron;
  float rudder;

  //engine state for first engine
  uint32_t num_engines;
  uint32_t eng_state[FG_NET_FDM_MAX_ENGINES];// Engine state (off, cranking, running)
  float rpm[FG_NET_FDM_MAX_ENGINES];       // Engine RPM rev/min

};

extern struct NpsFdm fdm;

extern void nps_fdm_init(double dt);
extern void nps_fdm_run_step(bool launch, double *commands, int commands_nb);
extern void nps_fdm_set_wind(double speed, double dir);
extern void nps_fdm_set_wind_ned(double wind_north, double wind_east, double wind_down);
extern void nps_fdm_set_turbulence(double wind_speed, int turbulence_severity);
/** Set temperature in degrees Celcius at given height h above MSL */
extern void nps_fdm_set_temperature(double temp, double h);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NPS_FDM */
