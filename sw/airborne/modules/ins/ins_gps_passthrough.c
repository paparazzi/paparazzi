/*
 * Copyright (C) 2004-2012 The Paparazzi Team
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
 * @file modules/ins/ins_gps_passthrough.c
 *
 * Simply converts GPS ECEF position and velocity to NED
 * and passes it through to the state interface.
 */

#include "modules/ins/ins_gps_passthrough.h"
#include "modules/ins/ins.h"

#include <inttypes.h>
#include <math.h>

#include "state.h"
#include "modules/gps/gps.h"
#include "modules/core/abi.h"

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

#if USE_INS_NAV_INIT
#include "generated/flight_plan.h"
#endif


struct InsGpsPassthrough {
  struct LtpDef_i  ltp_def;
  bool           ltp_initialized;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;
};

struct InsGpsPassthrough ins_gp;

/** ABI bindings on ACCEL data
 */
#ifndef INS_PT_IMU_ID
#define INS_PT_IMU_ID ABI_BROADCAST
#endif
static abi_event accel_ev;
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);

static abi_event body_to_imu_ev;
static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);
static struct OrientationReps body_to_imu;


/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_PT_GPS_ID
#define INS_PT_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_PT_GPS_ID)
static abi_event gps_ev;

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  if (gps_s->fix < GPS_FIX_3D) {
    return;
  }
  if (!ins_gp.ltp_initialized) {
    ins_reset_local_origin();
  }

  /* simply scale and copy pos/speed from gps */
  struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_gp.ltp_def, &gps_s->ecef_pos);
  INT32_VECT3_SCALE_2(ins_gp.ltp_pos, gps_pos_cm_ned,
                      INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  stateSetPositionNed_i(&ins_gp.ltp_pos);

  struct NedCoor_i gps_speed_cm_s_ned;
  ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_gp.ltp_def, &gps_s->ecef_vel);
  INT32_VECT3_SCALE_2(ins_gp.ltp_speed, gps_speed_cm_s_ned,
                      INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
  stateSetSpeedNed_i(&ins_gp.ltp_speed);
}


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_gp.ltp_pos.x, &ins_gp.ltp_pos.y, &ins_gp.ltp_pos.z,
                    &ins_gp.ltp_speed.x, &ins_gp.ltp_speed.y, &ins_gp.ltp_speed.z,
                    &ins_gp.ltp_accel.x, &ins_gp.ltp_accel.y, &ins_gp.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_baro_z = 0.0;
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      (float *)&fake_baro_z, &ins_gp.ltp_pos.z,
                      &ins_gp.ltp_speed.z, &ins_gp.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_qfe = 0.0;
  if (ins_gp.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_gp.ltp_def.ecef.x, &ins_gp.ltp_def.ecef.y, &ins_gp.ltp_def.ecef.z,
                          &ins_gp.ltp_def.lla.lat, &ins_gp.ltp_def.lla.lon, &ins_gp.ltp_def.lla.alt,
                          &ins_gp.ltp_def.hmsl, (float *)&fake_qfe);
  }
}
#endif

void ins_gps_passthrough_init(void)
{

#if USE_INS_NAV_INIT
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_gp.ltp_def, &ecef_nav0);
  ins_gp.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_gp.ltp_def);

  ins_gp.ltp_initialized = true;
#else
  ins_gp.ltp_initialized  = false;
#endif

  INT32_VECT3_ZERO(ins_gp.ltp_pos);
  INT32_VECT3_ZERO(ins_gp.ltp_speed);
  INT32_VECT3_ZERO(ins_gp.ltp_accel);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
#endif

  AbiBindMsgGPS(INS_PT_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgIMU_ACCEL(INS_PT_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(INS_PT_IMU_ID, &body_to_imu_ev, body_to_imu_cb);
}

void ins_reset_local_origin(void)
{
  ltp_def_from_ecef_i(&ins_gp.ltp_def, &gps.ecef_pos);
  ins_gp.ltp_def.lla.alt = gps.lla_pos.alt;
  ins_gp.ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ins_gp.ltp_def);
  ins_gp.ltp_initialized = true;
}

void ins_reset_altitude_ref(void)
{
  struct LlaCoor_i lla = {
    .lat = state.ned_origin_i.lla.lat,
    .lon = state.ned_origin_i.lla.lon,
    .alt = gps.lla_pos.alt
  };
  ltp_def_from_lla_i(&ins_gp.ltp_def, &lla);
  ins_gp.ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ins_gp.ltp_def);
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  // untilt accel and remove gravity
  struct Int32Vect3 accel_body, accel_ned;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&body_to_imu);
  int32_rmat_transp_vmult(&accel_body, body_to_imu_rmat, accel);
  stateSetAccelBody_i(&accel_body);
  struct Int32RMat *ned_to_body_rmat = stateGetNedToBodyRMat_i();
  int32_rmat_transp_vmult(&accel_ned, ned_to_body_rmat, &accel_body);
  accel_ned.z += ACCEL_BFP_OF_REAL(9.81);
  stateSetAccelNed_i((struct NedCoor_i *)&accel_ned);
  VECT3_COPY(ins_gp.ltp_accel, accel_ned);
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  orientationSetQuat_f(&body_to_imu, q_b2i_f);
}

