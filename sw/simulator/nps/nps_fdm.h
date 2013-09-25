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


#include "std.h"
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
  bool_t on_ground;
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

  /*  velocity and acceleration wrt inertial frame expressed in ecef frame */
  //  struct EcefCoor_d  ecef_inertial_vel;
  //  struct EcefCoor_d  ecef_inertial_accel;
  /*  velocity and acceleration wrt ecef frame expressed in ecef frame     */
  struct EcefCoor_d  ecef_ecef_vel;
  struct EcefCoor_d  ecef_ecef_accel;
  /*  velocity and acceleration wrt ecef frame expressed in body frame     */
  struct DoubleVect3 body_ecef_vel;   /* aka UVW */
  struct DoubleVect3 body_ecef_accel;
  /*  velocity and acceleration wrt ecef frame expressed in ltp frame     */
  struct NedCoor_d ltp_ecef_vel;
  struct NedCoor_d ltp_ecef_accel;
  /*  velocity and acceleration wrt ecef frame expressed in ltppprz frame */
  struct NedCoor_d ltpprz_ecef_vel;
  struct NedCoor_d ltpprz_ecef_accel;

  /* attitude */
  struct DoubleQuat   ecef_to_body_quat;
  struct DoubleQuat   ltp_to_body_quat;
  struct DoubleEulers ltp_to_body_eulers;
  struct DoubleQuat   ltpprz_to_body_quat;
  struct DoubleEulers ltpprz_to_body_eulers;

  /*  velocity and acceleration wrt ecef frame expressed in body frame     */
  struct DoubleRates  body_ecef_rotvel;
  struct DoubleRates  body_ecef_rotaccel;

  struct DoubleVect3 ltp_g;
  struct DoubleVect3 ltp_h;

  struct DoubleVect3 wind; ///< velocity in m/s in NED

};

extern struct NpsFdm fdm;

extern void nps_fdm_init(double dt);
extern void nps_fdm_run_step(double* commands);
extern void nps_fdm_set_wind(double speed, double dir, int turbulence_severity);

#endif /* NPS_FDM */
