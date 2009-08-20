/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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


#include "math/pprz_geodetic_int.h"


/* gps transformed to LTP-NED  */
struct LtpDef_i  booz_ins_ltp_def;
         bool_t  booz_ins_ltp_initialised;
struct NedCoor_i booz_ins_gps_pos_cm_ned;
struct NedCoor_i booz_ins_gps_speed_cm_s_ned;

/* barometer                   */
#ifdef USE_VFF
int32_t booz_ins_qfe;
bool_t  booz_ins_baro_initialised;
int32_t booz_ins_baro_alt;
bool_t  booz_ins_vff_realign;
#endif

/* output                      */
struct NedCoor_i booz_ins_ltp_pos;
struct NedCoor_i booz_ins_ltp_speed;
struct NedCoor_i booz_ins_ltp_accel;
struct EnuCoor_i booz_ins_enu_pos;
struct EnuCoor_i booz_ins_enu_speed;
struct EnuCoor_i booz_ins_enu_accel;


void booz_ins_init() {
#ifdef USE_VFF
  booz_ins_ltp_initialised  = FALSE;
  booz_ins_baro_initialised = FALSE;
  booz_ins_vff_realign = FALSE;
  b2_vff_init(0., 0., 0.);
#endif
#ifdef USE_HFF
  b2_hff_init(0., 0., 0., 0., 0., 0.);
#endif
  INT32_VECT3_ZERO(booz_ins_ltp_pos);
  INT32_VECT3_ZERO(booz_ins_ltp_speed);
  INT32_VECT3_ZERO(booz_ins_ltp_accel);
  INT32_VECT3_ZERO(booz_ins_enu_pos);
  INT32_VECT3_ZERO(booz_ins_enu_speed);
  INT32_VECT3_ZERO(booz_ins_enu_accel);
}

void booz_ins_propagate() {
#ifdef BOOZ_INS_UNTILT_ACCEL
  struct Int32Vect3 accel_body;
  INT32_RMAT_TRANSP_VMULT(accel_body, booz_imu.body_to_imu_rmat, booz_imu.accel);
  struct Int32Vect3 accel_ltp;
  INT32_RMAT_TRANSP_VMULT(accel_ltp, booz_ahrs.ltp_to_body_rmat, accel_body);
#ifdef USE_HFF
  float x_accel_float = ACCEL_FLOAT_OF_BFP(accel_ltp.x);
  float y_accel_float = ACCEL_FLOAT_OF_BFP(accel_ltp.y);
#endif
  float z_accel_float = ACCEL_FLOAT_OF_BFP(accel_ltp.z);
#else /* BOOZ_INS_UNTILT_ACCELS */
#ifdef USE_HFF
  float x_accel_float = ACCEL_FLOAT_OF_BFP(booz_imu.accel.x);
  float y_accel_float = ACCEL_FLOAT_OF_BFP(booz_imu.accel.y);
#endif
  float z_accel_float = ACCEL_FLOAT_OF_BFP(booz_imu.accel.z);
#endif /* BOOZ_INS_UNTILT_ACCELS */

#ifdef USE_VFF
  if (booz2_analog_baro_status == BOOZ2_ANALOG_BARO_RUNNING && booz_ins_baro_initialised) {
    b2_vff_propagate(z_accel_float);
    booz_ins_ltp_accel.z = ACCEL_BFP_OF_REAL(b2_vff_zdotdot);
    booz_ins_ltp_speed.z = SPEED_BFP_OF_REAL(b2_vff_zdot);
    booz_ins_ltp_pos.z   = POS_BFP_OF_REAL(b2_vff_z);
  }
#endif /* USE_VFF */

#ifdef USE_HFF
  if (booz_ahrs.status == BOOZ_AHRS_RUNNING && booz_gps_state.fix == BOOZ2_GPS_FIX_3D && booz_ins_ltp_initialised ) {
    b2_hff_propagate(x_accel_float, y_accel_float);
	booz_ins_ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_xdotdot);
	booz_ins_ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_ydotdot);
    booz_ins_ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_xdot);
	booz_ins_ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_ydot);
    booz_ins_ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_x);
	booz_ins_ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_y);
  }
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
    if (booz_ins_vff_realign) {
      booz_ins_vff_realign = FALSE;
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
      booz_ins_ltp_initialised = TRUE;
    }
    ned_of_ecef_point_i(&booz_ins_gps_pos_cm_ned, &booz_ins_ltp_def, &booz_gps_state.ecef_pos);
    ned_of_ecef_vect_i(&booz_ins_gps_speed_cm_s_ned, &booz_ins_ltp_def, &booz_gps_state.ecef_vel);
#ifdef USE_HFF
	struct FloatVect2 gps_float;
#ifdef B2_HFF_UPDATE_POS
	VECT2_ASSIGN(gps_float, booz_ins_gps_pos_cm_ned.x, booz_ins_gps_pos_cm_ned.y);
	VECT2_SDIV(gps_float, gps_float, 100.);
	b2_hff_update_pos(gps_float.x, gps_float.y);
#endif
#ifdef B2_HFF_UPDATE_SPEED
	VECT2_ASSIGN(gps_float, booz_ins_gps_speed_cm_s_ned.x, booz_ins_gps_speed_cm_s_ned.y);
	VECT2_SDIV(gps_float, gps_float, 100.);
	b2_hff_update_v(gps_float.x, gps_float.y);
#endif
	booz_ins_ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_xdotdot);
	booz_ins_ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_ydotdot);
    booz_ins_ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_xdot);
	booz_ins_ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_ydot);
    booz_ins_ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_x);
	booz_ins_ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_y);
#ifndef USE_VFF /* only hf */
	booz_ins_ltp_pos.z =  (booz_ins_gps_pos_cm_ned.z * INT32_POS_OF_CM_NUM) / INT32_POS_OF_CM_DEN;
	booz_ins_ltp_speed.z =  (booz_ins_gps_speed_cm_s_ned.z * INT32_SPEED_OF_CM_S_NUM) INT32_SPEED_OF_CM_S_DEN;
#endif /* only hf */
#else /* hf not used */
	INT32_VECT2_SCALE_2(booz_ins_ltp_pos, booz_ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT2_SCALE_2(booz_ins_ltp_speed, booz_ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#ifndef USE_VFF /* neither hf nor vf used */
    INT32_VECT3_SCALE_2(booz_ins_ltp_pos, booz_ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT3_SCALE_2(booz_ins_ltp_speed, booz_ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif /* neither hf nor vf used */
#endif /* USE_HFF */
	
    INT32_VECT3_ENU_OF_NED(booz_ins_enu_pos, booz_ins_ltp_pos);
    INT32_VECT3_ENU_OF_NED(booz_ins_enu_speed, booz_ins_ltp_speed);
    INT32_VECT3_ENU_OF_NED(booz_ins_enu_accel, booz_ins_ltp_accel);
  }
#endif /* USE_GPS */
}


