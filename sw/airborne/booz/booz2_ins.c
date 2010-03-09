/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2009 Felix Ruess <felix.ruess@gmail.com>
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

#include "booz2_ins.h"

#include "booz_imu.h"
#include "booz2_analog_baro.h"
#include "booz2_gps.h"

#include "airframe.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "booz_ahrs.h"

#ifdef USE_VFF
#include "ins/booz2_vf_float.h"
#endif

#ifdef USE_HFF
#include "ins/booz2_hf_float.h"
#endif

#ifdef SITL
#include "nps_fdm.h"
#include <stdio.h>
#endif


#include "math/pprz_geodetic_int.h"

#include "flight_plan.h"

/* gps transformed to LTP-NED  */
struct LtpDef_i  booz_ins_ltp_def;
         bool_t  booz_ins_ltp_initialised;
struct NedCoor_i booz_ins_gps_pos_cm_ned;
struct NedCoor_i booz_ins_gps_speed_cm_s_ned;
#ifdef USE_HFF
/* horizontal gps transformed to NED in meters as float */
struct FloatVect2 booz_ins_gps_pos_m_ned;
struct FloatVect2 booz_ins_gps_speed_m_s_ned;
#endif
bool_t booz_ins_hf_realign;

/* barometer                   */
#ifdef USE_VFF
int32_t booz_ins_qfe;
bool_t  booz_ins_baro_initialised;
int32_t booz_ins_baro_alt;
#endif
bool_t  booz_ins_vf_realign;

/* output                      */
struct NedCoor_i booz_ins_ltp_pos;
struct NedCoor_i booz_ins_ltp_speed;
struct NedCoor_i booz_ins_ltp_accel;
struct EnuCoor_i booz_ins_enu_pos;
struct EnuCoor_i booz_ins_enu_speed;
struct EnuCoor_i booz_ins_enu_accel;


void booz_ins_init() {
#ifdef USE_INS_NAV_INIT
  booz_ins_ltp_initialised  = TRUE;

  /** FIXME: should use the same code than MOVE_WP in booz2_datalink.c */
  struct LlaCoor_i llh; /* Height above the ellipsoid */
  llh.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh.lon = INT32_RAD_OF_DEG(NAV_LON0);
  //llh.alt = NAV_ALT0 - booz_ins_ltp_def.hmsl + booz_ins_ltp_def.lla.alt;
  llh.alt = NAV_ALT0 + NAV_HMSL0;

  struct EcefCoor_i nav_init;
  ecef_of_lla_i(&nav_init, &llh);

  ltp_def_from_ecef_i(&booz_ins_ltp_def, &nav_init);
  booz_ins_ltp_def.hmsl = NAV_ALT0;
#else
  booz_ins_ltp_initialised  = FALSE;
#endif
  booz_ins_vf_realign = FALSE;
#ifdef USE_VFF
  booz_ins_baro_initialised = FALSE;
  b2_vff_init(0., 0., 0.);
#endif
  booz_ins_hf_realign = FALSE;
#ifdef USE_HFF
  b2_hff_init(0., 0., 0., 0.);
#endif
  INT32_VECT3_ZERO(booz_ins_ltp_pos);
  INT32_VECT3_ZERO(booz_ins_ltp_speed);
  INT32_VECT3_ZERO(booz_ins_ltp_accel);
  INT32_VECT3_ZERO(booz_ins_enu_pos);
  INT32_VECT3_ZERO(booz_ins_enu_speed);
  INT32_VECT3_ZERO(booz_ins_enu_accel);
}

void booz_ins_periodic( void ) {
#ifndef USE_HFF
  struct NedCoor_i d_pos;
  VECT2_COPY(d_pos, booz_ins_ltp_speed);
  INT32_VECT2_RSHIFT(d_pos, d_pos, 15);
  VECT2_ADD(booz_ins_ltp_pos, d_pos);
#endif
}

void booz_ins_realign_h(struct FloatVect2 pos, struct FloatVect2 speed) {
#ifdef USE_HFF
  b2_hff_realign(pos, speed);
#endif
}

void booz_ins_realign_v(float z) {
#ifdef USE_VFF
  b2_vff_realign(z);
#endif
}

void booz_ins_propagate() {
  /* untilt accels */
  struct Int32Vect3 accel_body;
  INT32_RMAT_TRANSP_VMULT(accel_body, booz_imu.body_to_imu_rmat, booz_imu.accel);
  struct Int32Vect3 accel_ltp;
  INT32_RMAT_TRANSP_VMULT(accel_ltp, booz_ahrs.ltp_to_body_rmat, accel_body);
  float z_accel_float = ACCEL_FLOAT_OF_BFP(accel_ltp.z);

#ifdef USE_VFF
  if (booz2_analog_baro_status == BOOZ2_ANALOG_BARO_RUNNING && booz_ins_baro_initialised) {
    b2_vff_propagate(z_accel_float);
    booz_ins_ltp_accel.z = ACCEL_BFP_OF_REAL(b2_vff_zdotdot);
    booz_ins_ltp_speed.z = SPEED_BFP_OF_REAL(b2_vff_zdot);
    booz_ins_ltp_pos.z   = POS_BFP_OF_REAL(b2_vff_z);
  }
  else { // feed accel from the sensors
    booz_ins_ltp_accel.z = ACCEL_BFP_OF_REAL(z_accel_float);
  }
#else
  booz_ins_ltp_accel.z = ACCEL_BFP_OF_REAL(z_accel_float);
#endif /* USE_VFF */

#ifdef USE_HFF
  b2_hff_store_accel_body();
  /* propagate horizontal filter */
  b2_hff_propagate();
  if ( booz_ins_ltp_initialised ) {
    /* update ins state from horizontal filter */
    booz_ins_ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_state.xdotdot);
    booz_ins_ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_state.ydotdot);
    booz_ins_ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_state.xdot);
    booz_ins_ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_state.ydot);
    booz_ins_ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_state.x);
    booz_ins_ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_state.y);
  }
  else {
    booz_ins_ltp_accel.x = accel_ltp.x;
    booz_ins_ltp_accel.y = accel_ltp.y;
  }
#else
  booz_ins_ltp_accel.x = accel_ltp.x;
  booz_ins_ltp_accel.y = accel_ltp.y;
#endif /* USE_HFF */

  INT32_VECT3_ENU_OF_NED(booz_ins_enu_pos, booz_ins_ltp_pos);
  INT32_VECT3_ENU_OF_NED(booz_ins_enu_speed, booz_ins_ltp_speed);
  INT32_VECT3_ENU_OF_NED(booz_ins_enu_accel, booz_ins_ltp_accel);
}

void booz_ins_update_baro() {
#ifdef USE_VFF
  if (booz2_analog_baro_status == BOOZ2_ANALOG_BARO_RUNNING) {
    if (!booz_ins_baro_initialised) {
      booz_ins_qfe = booz2_analog_baro_value;
      booz_ins_baro_initialised = TRUE;
    }
    booz_ins_baro_alt = (((int32_t)booz2_analog_baro_value - booz_ins_qfe) * BOOZ_INS_BARO_SENS_NUM)/BOOZ_INS_BARO_SENS_DEN;
    float alt_float = POS_FLOAT_OF_BFP(booz_ins_baro_alt);
    if (booz_ins_vf_realign) {
      booz_ins_vf_realign = FALSE;
      booz_ins_qfe = booz2_analog_baro_value;
      b2_vff_realign(0.);
      booz_ins_ltp_accel.z = ACCEL_BFP_OF_REAL(b2_vff_zdotdot);
      booz_ins_ltp_speed.z = SPEED_BFP_OF_REAL(b2_vff_zdot);
      booz_ins_ltp_pos.z   = POS_BFP_OF_REAL(b2_vff_z);
      booz_ins_enu_pos.z = -booz_ins_ltp_pos.z;
      booz_ins_enu_speed.z = -booz_ins_ltp_speed.z;
      booz_ins_enu_accel.z = -booz_ins_ltp_accel.z;
    }
    b2_vff_update(alt_float);
  }
#endif
}


void booz_ins_update_gps(void) {
#ifdef USE_GPS
  if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D) {
    if (!booz_ins_ltp_initialised) {
      ltp_def_from_ecef_i(&booz_ins_ltp_def, &booz_gps_state.ecef_pos);
      booz_ins_ltp_def.lla.alt = booz_gps_state.lla_pos.alt;
      booz_ins_ltp_def.hmsl = booz_gps_state.hmsl;
      booz_ins_ltp_initialised = TRUE;
    }
    ned_of_ecef_point_i(&booz_ins_gps_pos_cm_ned, &booz_ins_ltp_def, &booz_gps_state.ecef_pos);
    ned_of_ecef_vect_i(&booz_ins_gps_speed_cm_s_ned, &booz_ins_ltp_def, &booz_gps_state.ecef_vel);

#ifdef USE_HFF
    VECT2_ASSIGN(booz_ins_gps_pos_m_ned, booz_ins_gps_pos_cm_ned.x, booz_ins_gps_pos_cm_ned.y);
    VECT2_SDIV(booz_ins_gps_pos_m_ned, booz_ins_gps_pos_m_ned, 100.);
    VECT2_ASSIGN(booz_ins_gps_speed_m_s_ned, booz_ins_gps_speed_cm_s_ned.x, booz_ins_gps_speed_cm_s_ned.y);
    VECT2_SDIV(booz_ins_gps_speed_m_s_ned, booz_ins_gps_speed_m_s_ned, 100.);
    if (booz_ins_hf_realign) {
      booz_ins_hf_realign = FALSE;
#ifdef SITL
      struct FloatVect2 true_pos, true_speed;
      VECT2_COPY(true_pos, fdm.ltpprz_pos);
      VECT2_COPY(true_speed, fdm.ltpprz_ecef_vel);
      b2_hff_realign(true_pos, true_speed);
#else
      const struct FloatVect2 zero = {0.0, 0.0};
      b2_hff_realign(booz_ins_gps_pos_m_ned, zero);
#endif
    }
    b2_hff_update_gps();
    booz_ins_ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_state.xdotdot);
    booz_ins_ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_state.ydotdot);
    booz_ins_ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_state.xdot);
    booz_ins_ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_state.ydot);
    booz_ins_ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_state.x);
    booz_ins_ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_state.y);

#ifndef USE_VFF /* vff not used */
    booz_ins_ltp_pos.z =  (booz_ins_gps_pos_cm_ned.z * INT32_POS_OF_CM_NUM) / INT32_POS_OF_CM_DEN;
    booz_ins_ltp_speed.z =  (booz_ins_gps_speed_cm_s_ned.z * INT32_SPEED_OF_CM_S_NUM) INT32_SPEED_OF_CM_S_DEN;
#endif /* vff not used */
#endif /* hff used */


#ifndef USE_HFF /* hff not used */
#ifndef USE_VFF /* neither hf nor vf used */
    INT32_VECT3_SCALE_3(booz_ins_ltp_pos, booz_ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT3_SCALE_3(booz_ins_ltp_speed, booz_ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#else /* only vff used */
    INT32_VECT2_SCALE_2(booz_ins_ltp_pos, booz_ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT2_SCALE_2(booz_ins_ltp_speed, booz_ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif

#ifdef USE_GPS_LAG_HACK
    VECT2_COPY(d_pos, booz_ins_ltp_speed);
    INT32_VECT2_RSHIFT(d_pos, d_pos, 11);
    VECT2_ADD(booz_ins_ltp_pos, d_pos);
#endif
#endif /* hff not used */

    INT32_VECT3_ENU_OF_NED(booz_ins_enu_pos, booz_ins_ltp_pos);
    INT32_VECT3_ENU_OF_NED(booz_ins_enu_speed, booz_ins_ltp_speed);
    INT32_VECT3_ENU_OF_NED(booz_ins_enu_accel, booz_ins_ltp_accel);
  }
#endif /* USE_GPS */
}


