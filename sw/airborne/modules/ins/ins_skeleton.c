/*
 * Copyright (C) 2016 Felix Ruess <felix.ruess@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ins/ins_skeleton.c
 *
 * Paparazzi specific wrapper to run simple module based INS.
 */

#include "modules/ins/ins_skeleton.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
#include "message_pragmas.h"

#include "state.h"

#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

/*
 * ABI bindings
 */
/** baro */
#ifndef INS_MODULE_BARO_ID
#if USE_BARO_BOARD
#define INS_MODULE_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_MODULE_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_MODULE_BARO_ID)

/** IMU (accel, body_to_imu) */
#ifndef INS_MODULE_IMU_ID
#define INS_MODULE_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_MODULE_IMU_ID)

/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_MODULE_GPS_ID
#define INS_MODULE_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_MODULE_GPS_ID)

static abi_event baro_ev;
static abi_event accel_ev;
static abi_event gps_ev;
static abi_event body_to_imu_ev;

struct InsModuleInt ins_module;

void ins_module_wrapper_init(void);

/** copy position and speed to state interface */
static void ins_ned_to_state(void)
{
  stateSetPositionNed_i(&ins_module.ltp_pos);
  stateSetSpeedNed_i(&ins_module.ltp_speed);
  stateSetAccelNed_i(&ins_module.ltp_accel);

#if defined SITL && USE_NPS
  if (nps_bypass_ins) {
    sim_overwrite_ins();
  }
#endif
}

/***********************************************************
 * ABI callback functions
 **********************************************************/

static void baro_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, float pressure)
{
  /* call module implementation */
  ins_module_update_baro(pressure);
  ins_ned_to_state();
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    /* call module implementation */
    ins_module_propagate(accel, dt);
    ins_ned_to_state();
  }
  last_stamp = stamp;
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp, struct GpsState *gps_s)
{
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;

    /* copy GPS state */
    ins_module.gps = *gps_s;

    if (!ins_module.ltp_initialized) {
      ins_reset_local_origin();
    }

    if (gps_s->fix >= GPS_FIX_3D) {
      /* call module implementation */
      ins_module_update_gps(gps_s, dt);
      ins_ned_to_state();
    }
  }
  last_stamp = stamp;
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  orientationSetQuat_f(&ins_module.body_to_imu, q_b2i_f);
}
/*********************************************************************
 * weak functions that are used if not implemented in a module
 ********************************************************************/

void WEAK ins_module_init(void)
{
}

void WEAK ins_module_update_baro(float pressure __attribute__((unused)))
{
}

void WEAK ins_module_update_gps(struct GpsState *gps_s, float dt __attribute__((unused)))
{
  /* copy velocity from GPS */
  if (bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT)) {
    /* convert speed from cm/s to m/s in BFP with INT32_SPEED_FRAC */
    INT32_VECT3_SCALE_2(ins_module.ltp_speed, gps_s->ned_vel,
                        INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
  }
  else if (bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_ECEF_BIT)) {
    /* convert ECEF to NED */
    struct NedCoor_i gps_speed_cm_s_ned;
    ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_module.ltp_def, &gps_s->ecef_vel);
    /* convert speed from cm/s to m/s in BFP with INT32_SPEED_FRAC */
    INT32_VECT3_SCALE_2(ins_module.ltp_speed, gps_speed_cm_s_ned,
                        INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
  }

  /* copy position from GPS */
  if (bit_is_set(gps_s->valid_fields, GPS_VALID_POS_ECEF_BIT)) {
    /* convert ECEF to NED */
    struct NedCoor_i gps_pos_cm_ned;
    ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_module.ltp_def, &gps_s->ecef_pos);
    /* convert pos from cm to m in BFP with INT32_POS_FRAC */
    INT32_VECT3_SCALE_2(ins_module.ltp_pos, gps_pos_cm_ned,
                        INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  }
}

void WEAK ins_module_propagate(struct Int32Vect3 *accel, float dt __attribute__((unused)))
{
  /* untilt accels */
  struct Int32Vect3 accel_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&ins_module.body_to_imu);
  int32_rmat_transp_vmult(&accel_meas_body, body_to_imu_rmat, accel);
  stateSetAccelBody_i(&accel_meas_body);
  struct Int32Vect3 accel_meas_ltp;
  int32_rmat_transp_vmult(&accel_meas_ltp, stateGetNedToBodyRMat_i(), &accel_meas_body);

  VECT3_COPY(ins_module.ltp_accel, accel_meas_ltp);
}

void WEAK ins_module_reset_local_origin(void)
{
}


/***********************************************************
 * wrapper functions
 **********************************************************/

void ins_reset_local_origin(void)
{
  if (ins_module.gps.fix >= GPS_FIX_3D) {
    ltp_def_from_ecef_i(&ins_module.ltp_def, &ins_module.gps.ecef_pos);
    ins_module.ltp_def.lla.alt = ins_module.gps.lla_pos.alt;
    ins_module.ltp_def.hmsl = ins_module.gps.hmsl;
    ins_module.ltp_initialized = true;
    stateSetLocalOrigin_i(&ins_module.ltp_def);
  } else {
    ins_module.ltp_initialized = false;
  }

  ins_module_reset_local_origin();
}


void ins_module_wrapper_init(void)
{
#if USE_INS_NAV_INIT
  ins_init_origin_i_from_flightplan(&ins_module.ltp_def);
  ins_module.ltp_initialized = true;
#else
  ins_module.ltp_initialized  = false;
#endif

  INT32_VECT3_ZERO(ins_module.ltp_pos);
  INT32_VECT3_ZERO(ins_module.ltp_speed);
  INT32_VECT3_ZERO(ins_module.ltp_accel);

  ins_module_init();

   // Bind to ABI messages
  AbiBindMsgBARO_ABS(INS_MODULE_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgIMU_ACCEL(INS_MODULE_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(INS_MODULE_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(INS_MODULE_IMU_ID, &body_to_imu_ev, body_to_imu_cb);
}

