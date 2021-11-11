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

#include "modules/core/abi.h"

#include "modules/imu/imu.h"
#include "modules/gps/gps.h"

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
#include "math/pprz_stat.h"

#ifndef VFF_R_AGL
#define VFF_R_AGL 0.2
#endif

#if USE_SONAR
#if !USE_VFF_EXTENDED
#error USE_SONAR needs USE_VFF_EXTENDED
#endif

#ifdef INS_SONAR_THROTTLE_THRESHOLD
#include "firmwares/rotorcraft/stabilization.h"
#endif

#ifndef INS_SONAR_MIN_RANGE
#define INS_SONAR_MIN_RANGE 0.001
#endif
#ifndef INS_SONAR_MAX_RANGE
#define INS_SONAR_MAX_RANGE 4.0
#endif
#define VFF_R_SONAR_0 0.2
#ifndef VFF_R_SONAR_OF_M
#define VFF_R_SONAR_OF_M 0.2
#endif

#endif // USE_SONAR

#if USE_GPS
#ifndef INS_VFF_R_GPS
#define INS_VFF_R_GPS 2.0
#endif

#ifndef INS_VFF_VZ_R_GPS
#define INS_VFF_VZ_R_GPS 2.0
#endif
#endif // USE_GPS

/** maximum number of propagation steps without any updates in between */
#ifndef INS_MAX_PROPAGATION_STEPS
#define INS_MAX_PROPAGATION_STEPS 200
#endif

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

/** default barometer to use in INS */
#define INS_BARO_MAX_INIT_VAR 1.f  // variance threshold to set initial baro measurement
#ifndef INS_INT_BARO_ID
#if USE_BARO_BOARD
#define INS_INT_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_INT_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_INT_BARO_ID)
abi_event baro_ev;
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure);

/** ABI binding for IMU data.
 * Used accel ABI messages.
 */
#ifndef INS_INT_IMU_ID
#define INS_INT_IMU_ID ABI_BROADCAST
#endif
static abi_event accel_ev;
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);

#ifndef INS_INT_GPS_ID
#define INS_INT_GPS_ID GPS_MULTI_ID
#endif
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);

/** ABI binding for VELOCITY_ESTIMATE.
 * Usually this is coming from opticflow.
 */
#ifndef INS_INT_VEL_ID
#define INS_INT_VEL_ID ABI_BROADCAST
#endif
static abi_event vel_est_ev;
static void vel_est_cb(uint8_t sender_id,
                       uint32_t stamp,
                       float x, float y, float z,
                       float noise_x, float noise_y, float noise_z);
#ifndef INS_INT_POS_ID
#define INS_INT_POS_ID ABI_BROADCAST
#endif
static abi_event pos_est_ev;
static void pos_est_cb(uint8_t sender_id,
                       uint32_t stamp,
                       float x, float y, float z,
                       float noise_x, float noise_y, float noise_z);

/** ABI binding for AGL.
 * Usually this is comes from sonar or gps.
 */
#ifndef INS_INT_AGL_ID
#define INS_INT_AGL_ID ABI_BROADCAST
#endif
static abi_event agl_ev;                 ///< The agl ABI event
static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance);

struct InsInt ins_int;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_int.ltp_pos.x, &ins_int.ltp_pos.y, &ins_int.ltp_pos.z,
                    &ins_int.ltp_speed.x, &ins_int.ltp_speed.y, &ins_int.ltp_speed.z,
                    &ins_int.ltp_accel.x, &ins_int.ltp_accel.y, &ins_int.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_int.baro_z, &ins_int.ltp_pos.z, &ins_int.ltp_speed.z, &ins_int.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_int.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_int.ltp_def.ecef.x, &ins_int.ltp_def.ecef.y, &ins_int.ltp_def.ecef.z,
                          &ins_int.ltp_def.lla.lat, &ins_int.ltp_def.lla.lon, &ins_int.ltp_def.lla.alt,
                          &ins_int.ltp_def.hmsl, &ins_int.qfe);
  }
}
#endif

static void ins_ned_to_state(void);
static void ins_update_from_vff(void);
#if USE_HFF
static void ins_update_from_hff(void);
#endif


void ins_int_init(void)
{

#if USE_INS_NAV_INIT
  ins_init_origin_i_from_flightplan(&ins_int.ltp_def);
  ins_int.ltp_initialized = true;
#else
  ins_int.ltp_initialized  = false;
#endif

  /* we haven't had any measurement updates yet, so set the counter to max */
  ins_int.propagation_cnt = INS_MAX_PROPAGATION_STEPS;

  // Bind to BARO_ABS message
  AbiBindMsgBARO_ABS(INS_INT_BARO_ID, &baro_ev, baro_cb);
  ins_int.baro_initialized = false;

  ins_int.vf_reset = false;
  ins_int.hf_realign = false;

  /* init vertical and horizontal filters */
  vff_init_zero();
#if USE_HFF
  hff_init(0., 0., 0., 0.);
#endif

  INT32_VECT3_ZERO(ins_int.ltp_pos);
  INT32_VECT3_ZERO(ins_int.ltp_speed);
  INT32_VECT3_ZERO(ins_int.ltp_accel);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
#endif

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_ACCEL_INT32(INS_INT_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(INS_INT_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgVELOCITY_ESTIMATE(INS_INT_VEL_ID, &vel_est_ev, vel_est_cb);
  AbiBindMsgPOSITION_ESTIMATE(INS_INT_POS_ID, &pos_est_ev, pos_est_cb);
  AbiBindMsgAGL(INS_INT_AGL_ID, &agl_ev, agl_cb); // ABI to the altitude above ground level
}

void ins_reset_local_origin(void)
{
#if USE_GPS
  if (GpsFixValid()) {
    ltp_def_from_ecef_i(&ins_int.ltp_def, &gps.ecef_pos);
    ins_int.ltp_def.lla.alt = gps.lla_pos.alt;
    ins_int.ltp_def.hmsl = gps.hmsl;
    ins_int.ltp_initialized = true;
    stateSetLocalOrigin_i(&ins_int.ltp_def);
  } else {
    ins_int.ltp_initialized = false;
  }
#else
  ins_int.ltp_initialized = false;
#endif

#if USE_HFF
  ins_int.hf_realign = true;
#endif
  ins_int.vf_reset = true;
}

void ins_reset_altitude_ref(void)
{
#if USE_GPS
  if (GpsFixValid()) {
    struct LlaCoor_i lla = {
      .lat = state.ned_origin_i.lla.lat,
      .lon = state.ned_origin_i.lla.lon,
      .alt = gps.lla_pos.alt
    };
    ltp_def_from_lla_i(&ins_int.ltp_def, &lla);
    ins_int.ltp_def.hmsl = gps.hmsl;
    stateSetLocalOrigin_i(&ins_int.ltp_def);
  }
#endif
  ins_int.vf_reset = true;
}

void ins_int_propagate(struct Int32Vect3 *accel, float dt)
{
  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&accel_meas_body, body_to_imu_rmat, accel);
  stateSetAccelBody_i(&accel_meas_body);
  struct Int32Vect3 accel_meas_ltp;
  int32_rmat_transp_vmult(&accel_meas_ltp, stateGetNedToBodyRMat_i(), &accel_meas_body);

  float z_accel_meas_float = ACCEL_FLOAT_OF_BFP(accel_meas_ltp.z);

  /* Propagate only if we got any measurement during the last INS_MAX_PROPAGATION_STEPS.
   * Otherwise halt the propagation to not diverge and only set the acceleration.
   * This should only be relevant in the startup phase when the baro is not yet initialized
   * and there is no gps fix yet...
   */
  if (ins_int.propagation_cnt < INS_MAX_PROPAGATION_STEPS) {
    vff_propagate(z_accel_meas_float, dt);
    ins_update_from_vff();
  } else {
    // feed accel from the sensors
    // subtract -9.81m/s2 (acceleration measured due to gravity,
    // but vehicle not accelerating in ltp)
    ins_int.ltp_accel.z = accel_meas_ltp.z + ACCEL_BFP_OF_REAL(9.81);
  }

#if USE_HFF
  /* propagate horizontal filter */
  hff_propagate();
  /* convert and copy result to ins_int */
  ins_update_from_hff();
#else
  ins_int.ltp_accel.x = accel_meas_ltp.x;
  ins_int.ltp_accel.y = accel_meas_ltp.y;
#endif /* USE_HFF */

  ins_ned_to_state();

  /* increment the propagation counter, while making sure it doesn't overflow */
  if (ins_int.propagation_cnt < 100 * INS_MAX_PROPAGATION_STEPS) {
    ins_int.propagation_cnt++;
  }
}

static void baro_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, float pressure)
{
  if (pressure < 1.f)
  {
    // bad baro pressure, don't use
    return;
  }

  if (!ins_int.baro_initialized) {
#define press_hist_len 10
    static float press_hist[press_hist_len];
    static uint8_t idx = 0;

    press_hist[idx] = pressure;
    idx = (idx + 1) % press_hist_len;
    float var = variance_f(press_hist, press_hist_len);
    if (var < INS_BARO_MAX_INIT_VAR){
      // wait for a first positive value
      ins_int.vf_reset = true;
      ins_int.baro_initialized = true;
    }
  }

  if (ins_int.baro_initialized) {
    float height_correction = 0.f;
    if(ins_int.ltp_initialized){
      // Calculate the distance to the origin
      struct EnuCoor_f *enu = stateGetPositionEnu_f();
      double dist2_to_origin = enu->x * enu->x + enu->y * enu->y;

      // correction for the earth's curvature
      const double earth_radius = 6378137.0;
      height_correction = (float)(sqrt(earth_radius * earth_radius + dist2_to_origin) - earth_radius);
    }

    if (ins_int.vf_reset) {
      ins_int.vf_reset = false;
      ins_int.qfe = pressure;
      vff_realign(height_correction);
      ins_update_from_vff();
    }

    float baro_up = pprz_isa_height_of_pressure(pressure, ins_int.qfe);

    // The VFF will update in the NED frame
    ins_int.baro_z = -(baro_up - height_correction);

#if USE_VFF_EXTENDED
    vff_update_baro(ins_int.baro_z);
#else
    vff_update(ins_int.baro_z);
#endif

    /* reset the counter to indicate we just had a measurement update */
    ins_int.propagation_cnt = 0;
  }
}

#if USE_GPS
void ins_int_update_gps(struct GpsState *gps_s)
{
  if (gps_s->fix < GPS_FIX_3D) {
    return;
  }

  if (!ins_int.ltp_initialized) {
    ins_reset_local_origin();
  }

  struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_int.ltp_def, &gps_s->ecef_pos);

  /* calculate body frame position taking BODY_TO_GPS translation (in cm) into account */
#ifdef INS_BODY_TO_GPS_X
  /* body2gps translation in body frame */
  struct Int32Vect3 b2g_b = {
    .x = INS_BODY_TO_GPS_X,
    .y = INS_BODY_TO_GPS_Y,
    .z = INS_BODY_TO_GPS_Z
  };
  /* rotate offset given in body frame to navigation/ltp frame using current attitude */
  struct Int32Quat q_b2n = *stateGetNedToBodyQuat_i();
  QUAT_INVERT(q_b2n, q_b2n);
  struct Int32Vect3 b2g_n;
  int32_quat_vmult(&b2g_n, &q_b2n, &b2g_b);
  /* subtract body2gps translation in ltp from gps position */
  VECT3_SUB(gps_pos_cm_ned, b2g_n);
#endif

  /// @todo maybe use gps_s->ned_vel directly??
  struct NedCoor_i gps_speed_cm_s_ned;
  ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_int.ltp_def, &gps_s->ecef_vel);

#if INS_USE_GPS_ALT
  vff_update_z_conf(((float)gps_pos_cm_ned.z) / 100.0, INS_VFF_R_GPS);
#endif
#if INS_USE_GPS_ALT_SPEED
  vff_update_vz_conf(((float)gps_speed_cm_s_ned.z) / 100.0, INS_VFF_VZ_R_GPS);
  ins_int.propagation_cnt = 0;
#endif

#if USE_HFF
  /* horizontal gps transformed to NED in meters as float */
  struct FloatVect2 gps_pos_m_ned;
  VECT2_ASSIGN(gps_pos_m_ned, gps_pos_cm_ned.x, gps_pos_cm_ned.y);
  VECT2_SDIV(gps_pos_m_ned, gps_pos_m_ned, 100.0f);

  struct FloatVect2 gps_speed_m_s_ned;
  VECT2_ASSIGN(gps_speed_m_s_ned, gps_speed_cm_s_ned.x, gps_speed_cm_s_ned.y);
  VECT2_SDIV(gps_speed_m_s_ned, gps_speed_m_s_ned, 100.f);

  if (ins_int.hf_realign) {
    ins_int.hf_realign = false;
    hff_realign(gps_pos_m_ned, gps_speed_m_s_ned);
  }
  // run horizontal filter
  hff_update_gps(&gps_pos_m_ned, &gps_speed_m_s_ned);
  // convert and copy result to ins_int
  ins_update_from_hff();

#else  /* hff not used */
  /* simply copy horizontal pos/speed from gps */
  INT32_VECT2_SCALE_2(ins_int.ltp_pos, gps_pos_cm_ned,
                      INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  INT32_VECT2_SCALE_2(ins_int.ltp_speed, gps_speed_cm_s_ned,
                      INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
#endif /* USE_HFF */

  ins_ned_to_state();

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}
#else
void ins_int_update_gps(struct GpsState *gps_s __attribute__((unused))) {}
#endif /* USE_GPS */

/** agl_cb
 * This callback handles all estimates of the height of the vehicle above the ground under it
 * This is only used with the extended version of the vertical float filter
 */
#if USE_VFF_EXTENDED
static void agl_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, float distance) {
  if (distance <= 0 || !(ins_int.baro_initialized)) {
    return;
  }

#if USE_SONAR
  if (distance > INS_SONAR_MAX_RANGE || distance < INS_SONAR_MIN_RANGE){
    return;
  }
#endif
#ifdef INS_AGL_THROTTLE_THRESHOLD
   if(stabilization_cmd[COMMAND_THRUST] < INS_AGL_THROTTLE_THRESHOLD){
     return;
   }
#endif
#ifdef INS_AGL_BARO_THRESHOLD
  if(ins_int.baro_z < -INS_SONAR_BARO_THRESHOLD){ /* z down */
    return;
  }
#endif

#if USE_SONAR
  vff_update_agl(-distance, VFF_R_SONAR_0 + VFF_R_SONAR_OF_M * fabsf(distance));
#else
  // TODO: this assumes that you will either have sonar or other agl sensor never both
  vff_update_agl(-distance, VFF_R_AGL);
#endif
    /* reset the counter to indicate we just had a measurement update */
    ins_int.propagation_cnt = 0;
}
#else
static void agl_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, __attribute__((unused)) float distance) {}
#endif

/** copy position and speed to state interface */
static void ins_ned_to_state(void)
{
  stateSetPositionNed_i(&ins_int.ltp_pos);
  stateSetSpeedNed_i(&ins_int.ltp_speed);
  stateSetAccelNed_i(&ins_int.ltp_accel);

#if defined SITL && USE_NPS
  if (nps_bypass_ins) {
    sim_overwrite_ins();
  }
#endif
}

/** update ins state from vertical filter */
static void ins_update_from_vff(void)
{
  ins_int.ltp_accel.z = ACCEL_BFP_OF_REAL(vff.zdotdot);
  ins_int.ltp_speed.z = SPEED_BFP_OF_REAL(vff.zdot);
  ins_int.ltp_pos.z   = POS_BFP_OF_REAL(vff.z);
}

#if USE_HFF
/** update ins state from horizontal filter */
static void ins_update_from_hff(void)
{
  ins_int.ltp_accel.x = ACCEL_BFP_OF_REAL(hff.xdotdot);
  ins_int.ltp_accel.y = ACCEL_BFP_OF_REAL(hff.ydotdot);
  ins_int.ltp_speed.x = SPEED_BFP_OF_REAL(hff.xdot);
  ins_int.ltp_speed.y = SPEED_BFP_OF_REAL(hff.ydot);
  ins_int.ltp_pos.x   = POS_BFP_OF_REAL(hff.x);
  ins_int.ltp_pos.y   = POS_BFP_OF_REAL(hff.y);
}
#endif


static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
  PRINT_CONFIG_MSG("Calculating dt for INS int propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ins_int_propagate(accel, dt);
  }
  last_stamp = stamp;
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ins_int_update_gps(gps_s);
}

/* body relative velocity estimate
 *
 */
static void vel_est_cb(uint8_t sender_id __attribute__((unused)),
                       uint32_t stamp __attribute__((unused)),
                       float x, float y, float z,
                       float noise_x, float noise_y, float noise_z)
{
  struct FloatVect3 vel_body = {x, y, z};

  /* rotate velocity estimate to nav/ltp frame */
  struct FloatQuat q_b2n = *stateGetNedToBodyQuat_f();
  QUAT_INVERT(q_b2n, q_b2n);
  struct FloatVect3 vel_ned;
  float_quat_vmult(&vel_ned, &q_b2n, &vel_body);

  // abi message contains an update to the horizontal velocity estimate
#if USE_HFF
  struct FloatVect2 vel = {vel_ned.x, vel_ned.y};
  struct FloatVect2 Rvel = {noise_x, noise_y};

  hff_update_vel(vel,  Rvel);
  ins_update_from_hff();
#else
  if (noise_x >= 0.f)
  {
    ins_int.ltp_speed.x = SPEED_BFP_OF_REAL(vel_ned.x);
  }
  if (noise_y >= 0.f)
  {
    ins_int.ltp_speed.y = SPEED_BFP_OF_REAL(vel_ned.y);
  }

  static uint32_t last_stamp_x = 0, last_stamp_y = 0;
  if (noise_x >= 0.f) {
    if (last_stamp_x > 0)
    {
      float dt = (float)(stamp - last_stamp_x) * 1e-6;
      ins_int.ltp_pos.x += lround(POS_BFP_OF_REAL(dt * vel_ned.x));
    }
    last_stamp_x = stamp;
  }

  if (noise_y >= 0.f)
  {
    if (last_stamp_y > 0)
    {
      float dt = (float)(stamp - last_stamp_y) * 1e-6;
      ins_int.ltp_pos.y += lround(POS_BFP_OF_REAL(dt * vel_ned.y));
    }
    last_stamp_y = stamp;
  }
#endif

  // abi message contains an update to the vertical velocity estimate
  vff_update_vz_conf(vel_ned.z, noise_z);

  ins_ned_to_state();

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}

/* NED position estimate relative to ltp origin
 */
static void pos_est_cb(uint8_t sender_id __attribute__((unused)),
                       uint32_t stamp __attribute__((unused)),
                       float x, float y, float z,
                       float noise_x, float noise_y, float noise_z)
{
  
#if USE_HFF
  struct FloatVect2 pos = {x, y};
  struct FloatVect2 Rpos = {noise_x, noise_y};

  hff_update_pos(pos, Rpos);
  ins_update_from_hff();
#else
  if (noise_x >= 0.f)
  {
    ins_int.ltp_pos.x = POS_BFP_OF_REAL(x);
  }
  if (noise_y >= 0.f)
  {
    ins_int.ltp_pos.y = POS_BFP_OF_REAL(y);
  }
#endif

  vff_update_z_conf(z, noise_z);

  ins_ned_to_state();

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}
