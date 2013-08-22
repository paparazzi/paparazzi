/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 * Copyright (C) 2012 Gautier Hattenberger
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

/**
 * @file subsystems/ins/ins_int_extended.c
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#include "subsystems/ins/ins_int_extended.h"

#include "subsystems/imu.h"
#include "subsystems/sensors/baro.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "state.h"

#include "subsystems/ins/vf_extended_float.h"

#if USE_HFF
#include "subsystems/ins/hf_float.h"
#endif

#ifdef SITL
#include "nps_fdm.h"
#include <stdio.h>
#endif

#ifdef INS_SONAR_THROTTLE_THRESHOLD
#include "firmwares/rotorcraft/stabilization.h"
#endif

#include "math/pprz_geodetic_int.h"

#include "generated/flight_plan.h"

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif


/* gps transformed to LTP-NED  */
struct LtpDef_i  ins_ltp_def;
         bool_t  ins_ltp_initialised;
struct NedCoor_i ins_gps_pos_cm_ned;
struct NedCoor_i ins_gps_speed_cm_s_ned;
#if USE_HFF
/* horizontal gps transformed to NED in meters as float */
struct FloatVect2 ins_gps_pos_m_ned;
struct FloatVect2 ins_gps_speed_m_s_ned;
#endif

/* barometer                   */
int32_t ins_qfe;
bool_t  ins_baro_initialised;
int32_t ins_baro_alt;
#include "filters/median_filter.h"
struct MedianFilterInt baro_median;

#if USE_SONAR
/* sonar                       */
bool_t  ins_update_on_agl;
int32_t ins_sonar_alt;
int32_t ins_sonar_offset;
struct MedianFilterInt sonar_median;
#ifndef INS_SONAR_OFFSET
#define INS_SONAR_OFFSET 0
#endif
#define VFF_R_SONAR_0 0.1
#define VFF_R_SONAR_OF_M 0.2
#endif // USE_SONAR

/* output                      */
struct NedCoor_i ins_ltp_pos;
struct NedCoor_i ins_ltp_speed;
struct NedCoor_i ins_ltp_accel;


void ins_init() {
#if USE_INS_NAV_INIT
  ins_ltp_initialised = TRUE;

  /** FIXME: should use the same code than MOVE_WP in firmwares/rotorcraft/datalink.c */
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh_nav0.lon = INT32_RAD_OF_DEG(NAV_LON0);
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_ltp_def, &ecef_nav0);
  ins_ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_ltp_def);
#else
  ins_ltp_initialised  = FALSE;
#endif

  ins_baro_initialised = FALSE;
  init_median_filter(&baro_median);

#if USE_SONAR
  ins_update_on_agl = FALSE;
  init_median_filter(&sonar_median);
  ins_sonar_offset = INS_SONAR_OFFSET;
#endif

  vff_init(0., 0., 0., 0.);
  ins.vf_realign = FALSE;
  ins.hf_realign = FALSE;
#if USE_HFF
  b2_hff_init(0., 0., 0., 0.);
#endif
  INT32_VECT3_ZERO(ins_ltp_pos);
  INT32_VECT3_ZERO(ins_ltp_speed);
  INT32_VECT3_ZERO(ins_ltp_accel);
}

void ins_periodic( void ) {
}

#if USE_HFF
void ins_realign_h(struct FloatVect2 pos, struct FloatVect2 speed) {
  b2_hff_realign(pos, speed);
}
#else
void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)), struct FloatVect2 speed __attribute__ ((unused))) {}
#endif /* USE_HFF */


void ins_realign_v(float z) {
  vff_realign(z);
}

void ins_propagate() {
  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  INT32_RMAT_TRANSP_VMULT(accel_meas_body, imu.body_to_imu_rmat, imu.accel);
  struct Int32Vect3 accel_meas_ltp;
  INT32_RMAT_TRANSP_VMULT(accel_meas_ltp, *stateGetNedToBodyRMat_i(), accel_meas_body);

  float z_accel_meas_float = ACCEL_FLOAT_OF_BFP(accel_meas_ltp.z);
  if (baro.status == BS_RUNNING && ins_baro_initialised) {
    vff_propagate(z_accel_meas_float);
    ins_ltp_accel.z = ACCEL_BFP_OF_REAL(vff_zdotdot);
    ins_ltp_speed.z = SPEED_BFP_OF_REAL(vff_zdot);
    ins_ltp_pos.z   = POS_BFP_OF_REAL(vff_z);
  }
  else { // feed accel from the sensors
    // subtract -9.81m/s2 (acceleration measured due to gravity, but vehivle not accelerating in ltp)
    ins_ltp_accel.z = accel_meas_ltp.z + ACCEL_BFP_OF_REAL(9.81);
  }

#if USE_HFF
  /* propagate horizontal filter */
  b2_hff_propagate();
#else
  ins_ltp_accel.x = accel_meas_ltp.x;
  ins_ltp_accel.y = accel_meas_ltp.y;
#endif /* USE_HFF */

  INS_NED_TO_STATE();
}

void ins_update_baro() {
  int32_t baro_pressure = update_median_filter(&baro_median, baro.absolute);
  if (baro.status == BS_RUNNING) {
    if (!ins_baro_initialised) {
      ins_qfe = baro_pressure;
      ins_baro_initialised = TRUE;
    }
    if (ins.vf_realign) {
      ins.vf_realign = FALSE;
      ins_qfe = baro_pressure;
      vff_realign(0.);
      ins_ltp_accel.z = ACCEL_BFP_OF_REAL(vff_zdotdot);
      ins_ltp_speed.z = SPEED_BFP_OF_REAL(vff_zdot);
      ins_ltp_pos.z   = POS_BFP_OF_REAL(vff_z);
    }
    else { /* not realigning, so normal update with baro measurement */
      ins_baro_alt = ((baro_pressure - ins_qfe) * INS_BARO_SENS_NUM)/INS_BARO_SENS_DEN;
      float alt_float = POS_FLOAT_OF_BFP(ins_baro_alt);
      vff_update_baro(alt_float);
    }
  }
  INS_NED_TO_STATE();
}


void ins_update_gps(void) {
#if USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    if (!ins_ltp_initialised) {
      ltp_def_from_ecef_i(&ins_ltp_def, &gps.ecef_pos);
      ins_ltp_def.lla.alt = gps.lla_pos.alt;
      ins_ltp_def.hmsl = gps.hmsl;
      ins_ltp_initialised = TRUE;
      stateSetLocalOrigin_i(&ins_ltp_def);
    }
    ned_of_ecef_point_i(&ins_gps_pos_cm_ned, &ins_ltp_def, &gps.ecef_pos);
    ned_of_ecef_vect_i(&ins_gps_speed_cm_s_ned, &ins_ltp_def, &gps.ecef_vel);
#if USE_HFF
    VECT2_ASSIGN(ins_gps_pos_m_ned, ins_gps_pos_cm_ned.x, ins_gps_pos_cm_ned.y);
    VECT2_SDIV(ins_gps_pos_m_ned, ins_gps_pos_m_ned, 100.);
    VECT2_ASSIGN(ins_gps_speed_m_s_ned, ins_gps_speed_cm_s_ned.x, ins_gps_speed_cm_s_ned.y);
    VECT2_SDIV(ins_gps_speed_m_s_ned, ins_gps_speed_m_s_ned, 100.);
    if (ins.hf_realign) {
      ins.hf_realign = FALSE;
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
#endif /* hff used */

#if !USE_HFF /* hff not used */
    INT32_VECT2_SCALE_2(ins_ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT2_SCALE_2(ins_ltp_speed, ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif /* hff not used */

  }
  INS_NED_TO_STATE();
#endif /* USE_GPS */
}

//#define INS_SONAR_VARIANCE_THRESHOLD 0.01

#ifdef INS_SONAR_VARIANCE_THRESHOLD

#include "messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"

#include "math/pprz_stat.h"
#define VAR_ERR_MAX 10
float var_err[VAR_ERR_MAX];
uint8_t var_idx = 0;
#endif


void ins_update_sonar() {
#if USE_SONAR
  static float last_offset = 0.;
  // new value filtered with median_filter
  ins_sonar_alt = update_median_filter(&sonar_median, sonar_meas);
  float sonar = (ins_sonar_alt - ins_sonar_offset) * INS_SONAR_SENS;

#ifdef INS_SONAR_VARIANCE_THRESHOLD
  /* compute variance of error between sonar and baro alt */
  int32_t err = POS_BFP_OF_REAL(sonar) + ins_baro_alt; // sonar positive up, baro positive down !!!!
  var_err[var_idx] = POS_FLOAT_OF_BFP(err);
  var_idx = (var_idx + 1) % VAR_ERR_MAX;
  float var = variance_float(var_err, VAR_ERR_MAX);
  DOWNLINK_SEND_INS_SONAR(DefaultChannel,DefaultDevice,&err, &sonar, &var);
  //DOWNLINK_SEND_INS_SONAR(DefaultChannel,DefaultDevice,&ins_sonar_alt, &sonar, &var);
#endif

  /* update filter assuming a flat ground */
  if (sonar < INS_SONAR_MAX_RANGE
#ifdef INS_SONAR_MIN_RANGE
      && sonar > INS_SONAR_MIN_RANGE
#endif
#ifdef INS_SONAR_THROTTLE_THRESHOLD
      && stabilization_cmd[COMMAND_THRUST] < INS_SONAR_THROTTLE_THRESHOLD
#endif
#ifdef INS_SONAR_STAB_THRESHOLD
      && stabilization_cmd[COMMAND_ROLL] < INS_SONAR_STAB_THRESHOLD
      && stabilization_cmd[COMMAND_ROLL] > -INS_SONAR_STAB_THRESHOLD
      && stabilization_cmd[COMMAND_PITCH] < INS_SONAR_STAB_THRESHOLD
      && stabilization_cmd[COMMAND_PITCH] > -INS_SONAR_STAB_THRESHOLD
      && stabilization_cmd[COMMAND_YAW] < INS_SONAR_STAB_THRESHOLD
      && stabilization_cmd[COMMAND_YAW] > -INS_SONAR_STAB_THRESHOLD
#endif
#ifdef INS_SONAR_BARO_THRESHOLD
      && ins_baro_alt > -POS_BFP_OF_REAL(INS_SONAR_BARO_THRESHOLD) /* z down */
#endif
#ifdef INS_SONAR_VARIANCE_THRESHOLD
      && var < INS_SONAR_VARIANCE_THRESHOLD
#endif
      && ins_update_on_agl
      && baro.status == BS_RUNNING) {
    vff_update_alt_conf(-sonar, VFF_R_SONAR_0 + VFF_R_SONAR_OF_M * fabs(sonar));
    last_offset = vff_offset;
  }
  else {
    /* update offset with last value to avoid divergence */
    vff_update_offset(last_offset);
  }
#endif // USE_SONAR
}

