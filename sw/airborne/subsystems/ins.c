/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

#include "subsystems/ins.h"

#include "subsystems/imu.h"
#include "subsystems/sensors/baro.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "subsystems/ahrs.h"

#ifdef USE_VFF
#include "subsystems/ins/vf_float.h"
#endif

#ifdef USE_HFF
#include "subsystems/ins/hf_float.h"
#endif

#ifdef BOOZ2_SONAR
#include "generated/modules.h"
#endif

#ifdef SITL
#include "nps_fdm.h"
#include <stdio.h>
#endif


#include "math/pprz_geodetic_int.h"

#include "generated/flight_plan.h"

/* gps transformed to LTP-NED  */
struct LtpDef_i  ins_ltp_def;
         bool_t  ins_ltp_initialised;
struct NedCoor_i ins_gps_pos_cm_ned;
struct NedCoor_i ins_gps_speed_cm_s_ned;
#ifdef USE_HFF
/* horizontal gps transformed to NED in meters as float */
struct FloatVect2 ins_gps_pos_m_ned;
struct FloatVect2 ins_gps_speed_m_s_ned;
#endif
bool_t ins_hf_realign;

/* barometer                   */
#ifdef USE_VFF
int32_t ins_qfe;
bool_t  ins_baro_initialised;
int32_t ins_baro_alt;
#ifdef USE_SONAR
bool_t  ins_update_on_agl;
int32_t ins_sonar_offset;
#endif
#endif
bool_t  ins_vf_realign;

/* output                      */
struct NedCoor_i ins_ltp_pos;
struct NedCoor_i ins_ltp_speed;
struct NedCoor_i ins_ltp_accel;
struct EnuCoor_i ins_enu_pos;
struct EnuCoor_i ins_enu_speed;
struct EnuCoor_i ins_enu_accel;


void ins_init() {
#ifdef USE_INS_NAV_INIT
  ins_ltp_initialised = TRUE;

  /** FIXME: should use the same code than MOVE_WP in firmwares/rotorcraft/datalink.c */
  struct LlaCoor_i llh; /* Height above the ellipsoid */
  llh.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh.lon = INT32_RAD_OF_DEG(NAV_LON0);
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i nav_init;
  ecef_of_lla_i(&nav_init, &llh);

  ltp_def_from_ecef_i(&ins_ltp_def, &nav_init);
  ins_ltp_def.hmsl = NAV_ALT0;
#else
  ins_ltp_initialised  = FALSE;
#endif
#ifdef USE_VFF
  ins_baro_initialised = FALSE;
#ifdef BOOZ2_SONAR
  ins_update_on_agl = FALSE;
#endif
  vff_init(0., 0., 0.);
#endif
  ins_vf_realign = FALSE;
  ins_hf_realign = FALSE;
#ifdef USE_HFF
  b2_hff_init(0., 0., 0., 0.);
#endif
  INT32_VECT3_ZERO(ins_ltp_pos);
  INT32_VECT3_ZERO(ins_ltp_speed);
  INT32_VECT3_ZERO(ins_ltp_accel);
  INT32_VECT3_ZERO(ins_enu_pos);
  INT32_VECT3_ZERO(ins_enu_speed);
  INT32_VECT3_ZERO(ins_enu_accel);
}

void ins_periodic( void ) {
}

#ifdef USE_HFF
void ins_realign_h(struct FloatVect2 pos, struct FloatVect2 speed) {
  b2_hff_realign(pos, speed);
}
#else
void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)), struct FloatVect2 speed __attribute__ ((unused))) {}
#endif /* USE_HFF */


void ins_realign_v(float z) {
#ifdef USE_VFF
  vff_realign(z);
#endif
}

void ins_propagate() {
  /* untilt accels */
  struct Int32Vect3 accel_body;
  INT32_RMAT_TRANSP_VMULT(accel_body, imu.body_to_imu_rmat, imu.accel);
  struct Int32Vect3 accel_ltp;
  INT32_RMAT_TRANSP_VMULT(accel_ltp, ahrs.ltp_to_body_rmat, accel_body);
  float z_accel_float = ACCEL_FLOAT_OF_BFP(accel_ltp.z);

#ifdef USE_VFF
  if (baro.status == BS_RUNNING && ins_baro_initialised) {
    vff_propagate(z_accel_float);
    ins_ltp_accel.z = ACCEL_BFP_OF_REAL(vff_zdotdot);
    ins_ltp_speed.z = SPEED_BFP_OF_REAL(vff_zdot);
    ins_ltp_pos.z   = POS_BFP_OF_REAL(vff_z);
  }
  else { // feed accel from the sensors
    ins_ltp_accel.z = ACCEL_BFP_OF_REAL(z_accel_float);
  }
#else
  ins_ltp_accel.z = ACCEL_BFP_OF_REAL(z_accel_float);
#endif /* USE_VFF */

#ifdef USE_HFF
  /* propagate horizontal filter */
  b2_hff_propagate();
#else
  ins_ltp_accel.x = accel_ltp.x;
  ins_ltp_accel.y = accel_ltp.y;
#endif /* USE_HFF */

  INT32_VECT3_ENU_OF_NED(ins_enu_pos, ins_ltp_pos);
  INT32_VECT3_ENU_OF_NED(ins_enu_speed, ins_ltp_speed);
  INT32_VECT3_ENU_OF_NED(ins_enu_accel, ins_ltp_accel);
}

void ins_update_baro() {
#ifdef USE_VFF
  if (baro.status == BS_RUNNING) {
    if (!ins_baro_initialised) {
      ins_qfe = baro.absolute;
      ins_baro_initialised = TRUE;
    }
    ins_baro_alt = ((baro.absolute - ins_qfe) * INS_BARO_SENS_NUM)/INS_BARO_SENS_DEN;
    float alt_float = POS_FLOAT_OF_BFP(ins_baro_alt);
    if (ins_vf_realign) {
      ins_vf_realign = FALSE;
      ins_qfe = baro.absolute;
#ifdef USE_SONAR
      ins_sonar_offset = sonar_meas;
#endif
      vff_realign(0.);
      ins_ltp_accel.z = ACCEL_BFP_OF_REAL(vff_zdotdot);
      ins_ltp_speed.z = SPEED_BFP_OF_REAL(vff_zdot);
      ins_ltp_pos.z   = POS_BFP_OF_REAL(vff_z);
      ins_enu_pos.z = -ins_ltp_pos.z;
      ins_enu_speed.z = -ins_ltp_speed.z;
      ins_enu_accel.z = -ins_ltp_accel.z;
    }
    vff_update(alt_float);
  }
#endif
}


void ins_update_gps(void) {
#ifdef USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    if (!ins_ltp_initialised) {
      ltp_def_from_ecef_i(&ins_ltp_def, &gps.ecef_pos);
      ins_ltp_def.lla.alt = gps.lla_pos.alt;
      ins_ltp_def.hmsl = gps.hmsl;
      ins_ltp_initialised = TRUE;
    }
    ned_of_ecef_point_i(&ins_gps_pos_cm_ned, &ins_ltp_def, &gps.ecef_pos);
    ned_of_ecef_vect_i(&ins_gps_speed_cm_s_ned, &ins_ltp_def, &gps.ecef_vel);
#ifdef USE_HFF
    VECT2_ASSIGN(ins_gps_pos_m_ned, ins_gps_pos_cm_ned.x, ins_gps_pos_cm_ned.y);
    VECT2_SDIV(ins_gps_pos_m_ned, ins_gps_pos_m_ned, 100.);
    VECT2_ASSIGN(ins_gps_speed_m_s_ned, ins_gps_speed_cm_s_ned.x, ins_gps_speed_cm_s_ned.y);
    VECT2_SDIV(ins_gps_speed_m_s_ned, ins_gps_speed_m_s_ned, 100.);
    if (ins_hf_realign) {
      ins_hf_realign = FALSE;
#ifdef SITL
      struct FloatVect2 true_pos, true_speed;
      VECT2_COPY(true_pos, fdm.ltpprz_pos);
      VECT2_COPY(true_speed, fdm.ltpprz_ecef_vel);
      b2_hff_realign(true_pos, true_speed);
#else
      const struct FloatVect2 zero = {0.0, 0.0};
      b2_hff_realign(ins_gps_pos_m_ned, zero);
#endif
    }
    b2_hff_update_gps();
#ifndef USE_VFF /* vff not used */
    ins_ltp_pos.z =  (ins_gps_pos_cm_ned.z * INT32_POS_OF_CM_NUM) / INT32_POS_OF_CM_DEN;
    ins_ltp_speed.z =  (ins_gps_speed_cm_s_ned.z * INT32_SPEED_OF_CM_S_NUM) INT32_SPEED_OF_CM_S_DEN;
#endif /* vff not used */
#endif /* hff used */


#ifndef USE_HFF /* hff not used */
#ifndef USE_VFF /* neither hf nor vf used */
    INT32_VECT3_SCALE_3(ins_ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT3_SCALE_3(ins_ltp_speed, ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#else /* only vff used */
    INT32_VECT2_SCALE_2(ins_ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT2_SCALE_2(ins_ltp_speed, ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif

#ifdef USE_GPS_LAG_HACK
    VECT2_COPY(d_pos, ins_ltp_speed);
    INT32_VECT2_RSHIFT(d_pos, d_pos, 11);
    VECT2_ADD(ins_ltp_pos, d_pos);
#endif
#endif /* hff not used */

    INT32_VECT3_ENU_OF_NED(ins_enu_pos, ins_ltp_pos);
    INT32_VECT3_ENU_OF_NED(ins_enu_speed, ins_ltp_speed);
    INT32_VECT3_ENU_OF_NED(ins_enu_accel, ins_ltp_accel);
  }
#endif /* USE_GPS */
}

void ins_update_sonar() {
#if defined USE_SONAR && defined USE_VFF
  static int32_t sonar_filtered = 0;
  sonar_filtered = (sonar_meas + 2*sonar_filtered) / 3;
  /* update baro_qfe assuming a flat ground */
  if (ins_update_on_agl && baro.status == BS_RUNNING) {
    int32_t d_sonar = (((int32_t)sonar_filtered - ins_sonar_offset) * INS_SONAR_SENS_NUM) / INS_SONAR_SENS_DEN;
    ins_qfe = baro.absolute + (d_sonar * (INS_BARO_SENS_DEN))/INS_BARO_SENS_NUM;
  }
#endif
}
