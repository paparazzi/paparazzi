/*
 * Copyright (C) 2012-2013 Freek van Tienen
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/ins/ins_ardrone2.c
 * INS implementation for ardrone2-sdk.
 */

#include "subsystems/ins/ins_ardrone2.h"
#include "subsystems/ahrs.h"
#include "subsystems/gps.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "math/pprz_geodetic_int.h"

#ifdef SITL
#include "nps_fdm.h"
#include <stdio.h>
#endif

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

struct InsArdrone2 ins_impl;

void ins_init() {
#if USE_INS_NAV_INIT
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_impl.ltp_def, &ecef_nav0);
  ins_impl.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_impl.ltp_def);

  ins_impl.ltp_initialized = TRUE;
#else
  ins_impl.ltp_initialized  = FALSE;
#endif

  INT32_VECT3_ZERO(ins_impl.ltp_pos);
  INT32_VECT3_ZERO(ins_impl.ltp_speed);
  INT32_VECT3_ZERO(ins_impl.ltp_accel);
}

void ins_periodic( void ) {
  if (ins_impl.ltp_initialized)
    ins.status = INS_RUNNING;
}

void ins_reset_local_origin( void ) {
  ins_impl.ltp_initialized = FALSE;
}

void ins_reset_altitude_ref( void ) {
#if USE_GPS
  struct LlaCoor_i lla = {
    state.ned_origin_i.lla.lon,
    state.ned_origin_i.lla.lat,
    gps.lla_pos.alt
  };
  ltp_def_from_lla_i(&ins_impl.ltp_def, &lla),
  ins_impl.ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ins_impl.ltp_def);
#endif
}

void ins_propagate(float __attribute__((unused)) dt) {
  /* untilt accels and speeds */
  float_rmat_transp_vmult(&ins_impl.ltp_accel, stateGetNedToBodyRMat_f(), &ahrs_impl.accel);
  float_rmat_transp_vmult(&ins_impl.ltp_speed, stateGetNedToBodyRMat_f(), &ahrs_impl.speed);

  //Add g to the accelerations
  ins_impl.ltp_accel.z += 9.81;

  //Save the accelerations and speeds
  stateSetAccelNed_f(&ins_impl.ltp_accel);
  stateSetSpeedNed_f(&ins_impl.ltp_speed);

  //Don't set the height if we use the one from the gps
#if !USE_GPS_HEIGHT
  //Set the height and save the position
  ins_impl.ltp_pos.z = -(ahrs_impl.altitude * INT32_POS_OF_CM_NUM) / INT32_POS_OF_CM_DEN;
  stateSetPositionNed_i(&ins_impl.ltp_pos);
#endif
}


void ins_update_gps(void) {
#if USE_GPS
  //Check for GPS fix
  if (gps.fix == GPS_FIX_3D) {
    //Set the initial coordinates
    if(!ins_impl.ltp_initialized) {
      ltp_def_from_ecef_i(&ins_impl.ltp_def, &gps.ecef_pos);
      ins_impl.ltp_def.lla.alt = gps.lla_pos.alt;
      ins_impl.ltp_def.hmsl = gps.hmsl;
      ins_impl.ltp_initialized = TRUE;
      stateSetLocalOrigin_i(&ins_impl.ltp_def);
    }

    //Set the x and y and maybe z position in ltp and save
    struct NedCoor_i ins_gps_pos_cm_ned;
    ned_of_ecef_point_i(&ins_gps_pos_cm_ned, &ins_impl.ltp_def, &gps.ecef_pos);

    //When we don't want to use the height of the navdata we can use the gps height
#if USE_GPS_HEIGHT
    INT32_VECT3_SCALE_2(ins_impl.ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
#else
    INT32_VECT2_SCALE_2(ins_impl.ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
#endif

    //Set the local origin
    stateSetPositionNed_i(&ins_impl.ltp_pos);
  }
#endif /* USE_GPS */
}
