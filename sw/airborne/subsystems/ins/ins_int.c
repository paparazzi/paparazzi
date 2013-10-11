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

/**
 * @file subsystems/ins/ins_int.c
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#include "subsystems/ins/ins_int.h"

#include "subsystems/abi.h"

#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"

#if USE_VFF_EXTENDED
#include "subsystems/ins/vf_extended_float.h"
#else
#include "subsystems/ins/vf_float.h"
#endif

#if USE_HFF
#include "subsystems/ins/hf_float.h"
#endif

#if defined SITL && USE_NPS
//#include "nps_fdm.h"
#include "nps_autopilot.h"
#include <stdio.h>
#endif

#include "math/pprz_geodetic_int.h"
#include "math/pprz_isa.h"

#include "generated/flight_plan.h"


#if USE_SONAR

#if !USE_VFF_EXTENDED
#error USE_SONAR needs USE_VFF_EXTENDED
#endif

#ifdef INS_SONAR_THROTTLE_THRESHOLD
#include "firmwares/rotorcraft/stabilization.h"
#endif

#ifndef INS_SONAR_OFFSET
#define INS_SONAR_OFFSET 0
#endif
#define VFF_R_SONAR_0 0.1
#define VFF_R_SONAR_OF_M 0.2

#endif // USE_SONAR

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif


/** default barometer to use in INS */
#ifndef INS_BARO_ID
#define INS_BARO_ID BARO_BOARD_SENDER_ID
#endif
abi_event baro_ev;
static void baro_cb(uint8_t sender_id, const float *pressure);

struct InsInt ins_impl;


static void ins_init_origin_from_flightplan(void);
static void ins_ned_to_state(void);
#if USE_HFF
static void ins_update_from_hff(void);
#endif


void ins_init(void) {

#if USE_INS_NAV_INIT
  ins_init_origin_from_flightplan();
  ins_impl.ltp_initialized = TRUE;
#else
  ins_impl.ltp_initialized  = FALSE;
#endif

  // Bind to BARO_ABS message
  AbiBindMsgBARO_ABS(INS_BARO_ID, &baro_ev, baro_cb);
  ins_impl.baro_initialized = FALSE;

#if USE_SONAR
  ins_impl.update_on_agl = FALSE;
  init_median_filter(&ins_impl.sonar_median);
  ins_impl.sonar_offset = INS_SONAR_OFFSET;
#endif

  ins.vf_realign = FALSE;
  ins.hf_realign = FALSE;

  /* init vertical and horizontal filters */
#if USE_VFF_EXTENDED
  vff_init(0., 0., 0., 0.);
#else
  vff_init(0., 0., 0.);
#endif
#if USE_HFF
  b2_hff_init(0., 0., 0., 0.);
#endif

  INT32_VECT3_ZERO(ins_impl.ltp_pos);
  INT32_VECT3_ZERO(ins_impl.ltp_speed);
  INT32_VECT3_ZERO(ins_impl.ltp_accel);

}

void ins_periodic(void) {
  if (ins_impl.ltp_initialized)
    ins.status = INS_RUNNING;
}

#if USE_HFF
void ins_realign_h(struct FloatVect2 pos, struct FloatVect2 speed) {
  b2_hff_realign(pos, speed);
}
#else
void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)),
                   struct FloatVect2 speed __attribute__ ((unused))) {}
#endif /* USE_HFF */


void ins_realign_v(float z) {
  vff_realign(z);
}

void ins_propagate() {
  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  INT32_RMAT_TRANSP_VMULT(accel_meas_body, imu.body_to_imu_rmat, imu.accel);
  struct Int32Vect3 accel_meas_ltp;
  INT32_RMAT_TRANSP_VMULT(accel_meas_ltp, (*stateGetNedToBodyRMat_i()), accel_meas_body);

  float z_accel_meas_float = ACCEL_FLOAT_OF_BFP(accel_meas_ltp.z);
  if (ins_impl.baro_initialized) {
    vff_propagate(z_accel_meas_float);
    ins_impl.ltp_accel.z = ACCEL_BFP_OF_REAL(vff_zdotdot);
    ins_impl.ltp_speed.z = SPEED_BFP_OF_REAL(vff_zdot);
    ins_impl.ltp_pos.z   = POS_BFP_OF_REAL(vff_z);
  }
  else { // feed accel from the sensors
    // subtract -9.81m/s2 (acceleration measured due to gravity,
    // but vehicle not accelerating in ltp)
    ins_impl.ltp_accel.z = accel_meas_ltp.z + ACCEL_BFP_OF_REAL(9.81);
  }

#if USE_HFF
  /* propagate horizontal filter */
  b2_hff_propagate();
  /* convert and copy result to ins_impl */
  ins_update_from_hff();
#else
  ins_impl.ltp_accel.x = accel_meas_ltp.x;
  ins_impl.ltp_accel.y = accel_meas_ltp.y;
#endif /* USE_HFF */

  ins_ned_to_state();
}

static void baro_cb(uint8_t __attribute__((unused)) sender_id, const float *pressure) {
  if (!ins_impl.baro_initialized) {
    ins_impl.qfe = *pressure;
    ins_impl.baro_initialized = TRUE;
  }
  if (ins.vf_realign) {
    ins.vf_realign = FALSE;
    ins_impl.qfe = *pressure;
    vff_realign(0.);
    ins_impl.ltp_accel.z = ACCEL_BFP_OF_REAL(vff_zdotdot);
    ins_impl.ltp_speed.z = SPEED_BFP_OF_REAL(vff_zdot);
    ins_impl.ltp_pos.z   = POS_BFP_OF_REAL(vff_z);
  }
  else {
    ins_impl.baro_z = -pprz_isa_height_of_pressure(*pressure, ins_impl.qfe);
#if USE_VFF_EXTENDED
    vff_update_baro(ins_impl.baro_z);
#else
    vff_update(ins_impl.baro_z);
#endif
  }
  ins_ned_to_state();
}


void ins_update_baro() {
}


void ins_update_gps(void) {
#if USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    if (!ins_impl.ltp_initialized) {
      ltp_def_from_ecef_i(&ins_impl.ltp_def, &gps.ecef_pos);
      ins_impl.ltp_def.lla.alt = gps.lla_pos.alt;
      ins_impl.ltp_def.hmsl = gps.hmsl;
      ins_impl.ltp_initialized = TRUE;
      stateSetLocalOrigin_i(&ins_impl.ltp_def);
    }

    struct NedCoor_i gps_pos_cm_ned;
    ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_impl.ltp_def, &gps.ecef_pos);
    /// @todo maybe use gps.ned_vel directly??
    struct NedCoor_i gps_speed_cm_s_ned;
    ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_impl.ltp_def, &gps.ecef_vel);

#if USE_HFF
    /* horizontal gps transformed to NED in meters as float */
    struct FloatVect2 gps_pos_m_ned;
    VECT2_ASSIGN(gps_pos_m_ned, gps_pos_cm_ned.x, gps_pos_cm_ned.y);
    VECT2_SDIV(gps_pos_m_ned, gps_pos_m_ned, 100.);

    struct FloatVect2 gps_speed_m_s_ned;
    VECT2_ASSIGN(gps_speed_m_s_ned, gps_speed_cm_s_ned.x, gps_speed_cm_s_ned.y);
    VECT2_SDIV(gps_speed_m_s_ned, gps_speed_m_s_ned, 100.);

    if (ins.hf_realign) {
      ins.hf_realign = FALSE;
      const struct FloatVect2 zero = {0.0, 0.0};
      b2_hff_realign(gps_pos_m_ned, zero);
    }
    // run horizontal filter
    b2_hff_update_gps(&gps_pos_m_ned, &gps_speed_m_s_ned);
    // convert and copy result to ins_impl
    ins_update_from_hff();

#else  /* hff not used */
    /* simply copy horizontal pos/speed from gps */
    INT32_VECT2_SCALE_2(ins_impl.ltp_pos, gps_pos_cm_ned,
                        INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT2_SCALE_2(ins_impl.ltp_speed, gps_speed_cm_s_ned,
                        INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif /* USE_HFF */

    ins_ned_to_state();
  }
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
  ins_impl.sonar_alt = update_median_filter(&ins_impl.sonar_median, sonar_meas);
  float sonar = (ins_impl.sonar_alt - ins_impl.sonar_offset) * INS_SONAR_SENS;

#ifdef INS_SONAR_VARIANCE_THRESHOLD
  /* compute variance of error between sonar and baro alt */
  float err = sonar + ins_impl.baro_z; // sonar positive up, baro positive down !!!!
  var_err[var_idx] = err;
  var_idx = (var_idx + 1) % VAR_ERR_MAX;
  float var = variance_float(var_err, VAR_ERR_MAX);
  DOWNLINK_SEND_INS_SONAR(DefaultChannel,DefaultDevice,&err, &sonar, &var);
  //DOWNLINK_SEND_INS_SONAR(DefaultChannel,DefaultDevice,&ins_impl.sonar_alt, &sonar, &var);
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
      && ins_impl.baro_z > -INS_SONAR_BARO_THRESHOLD /* z down */
#endif
#ifdef INS_SONAR_VARIANCE_THRESHOLD
      && var < INS_SONAR_VARIANCE_THRESHOLD
#endif
      && ins_impl.update_on_agl
      && ins_impl.baro_initialized) {
    vff_update_alt_conf(-sonar, VFF_R_SONAR_0 + VFF_R_SONAR_OF_M * fabs(sonar));
    last_offset = vff_offset;
  }
  else {
    /* update offset with last value to avoid divergence */
    vff_update_offset(last_offset);
  }
#endif // USE_SONAR
}


/** initialize the local origin (ltp_def) from flight plan position */
static void ins_init_origin_from_flightplan(void) {

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh_nav0.lon = INT32_RAD_OF_DEG(NAV_LON0);
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_impl.ltp_def, &ecef_nav0);
  ins_impl.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_impl.ltp_def);

}

/** copy position and speed to state interface */
static void ins_ned_to_state(void) {
  stateSetPositionNed_i(&ins_impl.ltp_pos);
  stateSetSpeedNed_i(&ins_impl.ltp_speed);
  stateSetAccelNed_i(&ins_impl.ltp_accel);

#if defined SITL && USE_NPS
  if (nps_bypass_ins)
    sim_overwrite_ins();
#endif
}


#if USE_HFF
/** update ins state from horizontal filter */
static void ins_update_from_hff(void) {
  ins_impl.ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_state.xdotdot);
  ins_impl.ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_state.ydotdot);
  ins_impl.ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_state.xdot);
  ins_impl.ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_state.ydot);
  ins_impl.ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_state.x);
  ins_impl.ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_state.y);
}
#endif
