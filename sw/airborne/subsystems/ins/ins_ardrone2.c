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


/* TODO: implement in state */
int32_t ins_qfe;
int32_t ins_baro_alt;

//Keep track of gps pos and the init pos
struct NedCoor_i ins_ltp_pos;
struct LtpDef_i ins_ltp_def;

// Keep track of INS LTP accel and speed
struct NedCoor_f ins_ltp_accel;
struct NedCoor_f ins_ltp_speed;

bool_t ins_ltp_initialised;

void ins_init() {
#if USE_INS_NAV_INIT
  struct LlaCoor_i llh_nav;

  /** FIXME: should use the same code than MOVE_WP in firmwares/rotorcraft/datalink.c */
  llh_nav.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh_nav.lon = INT32_RAD_OF_DEG(NAV_LON0);
  llh_nav.alt = NAV_ALT0 + NAV_MSL0;

  //Convert ltp
  ltp_def_from_lla_i(&ins_ltp_def, &llh_nav);
  ins_ltp_def.hmsl = NAV_ALT0;

  //Set the ltp
  stateSetLocalOrigin_i(&ins_ltp_def);

  ins_ltp_initialised = TRUE;
#else
  ins_ltp_initialised = FALSE;
#endif

  ins.vf_realign = FALSE;
  ins.hf_realign = FALSE;

  INT32_VECT3_ZERO(ins_ltp_pos);

  // TODO correct init
  ins.status = INS_RUNNING;
}

void ins_periodic( void ) {

}

void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)), struct FloatVect2 speed __attribute__ ((unused))) {

}

void ins_realign_v(float z __attribute__ ((unused))) {

}

void ins_propagate() {
  /* untilt accels and speeds */
  FLOAT_RMAT_VECT3_TRANSP_MUL(ins_ltp_accel, (*stateGetNedToBodyRMat_f()), ahrs_impl.accel);
  FLOAT_RMAT_VECT3_TRANSP_MUL(ins_ltp_speed, (*stateGetNedToBodyRMat_f()), ahrs_impl.speed);

  //Add g to the accelerations
  ins_ltp_accel.z += 9.81;

  //Save the accelerations and speeds
  stateSetAccelNed_f(&ins_ltp_accel);
  stateSetSpeedNed_f(&ins_ltp_speed);

  //Don't set the height if we use the one from the gps
#if !USE_GPS_HEIGHT
  //Set the height and save the position
  ins_ltp_pos.z = -(ahrs_impl.altitude * INT32_POS_OF_CM_NUM) / INT32_POS_OF_CM_DEN;
  stateSetPositionNed_i(&ins_ltp_pos);
#endif
}

void ins_update_baro() {

}


void ins_update_gps(void) {
#if USE_GPS
  //Check for GPS fix
  if (gps.fix == GPS_FIX_3D) {
    //Set the initial coordinates
    if(!ins_ltp_initialised) {
      ltp_def_from_ecef_i(&ins_ltp_def, &gps.ecef_pos);
      ins_ltp_def.lla.alt = gps.lla_pos.alt;
      ins_ltp_def.hmsl = gps.hmsl;
      ins_ltp_initialised = TRUE;
      stateSetLocalOrigin_i(&ins_ltp_def);
    }

    //Set the x and y and maybe z position in ltp and save
    struct NedCoor_i ins_gps_pos_cm_ned;
    ned_of_ecef_point_i(&ins_gps_pos_cm_ned, &ins_ltp_def, &gps.ecef_pos);

    //When we don't want to use the height of the navdata we can use the gps height
#if USE_GPS_HEIGHT
    INT32_VECT3_SCALE_2(ins_ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
#else
    INT32_VECT2_SCALE_2(ins_ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
#endif

    //Set the local origin
    stateSetPositionNed_i(&ins_ltp_pos);
  }
#endif /* USE_GPS */
}

void ins_update_sonar() {

}
