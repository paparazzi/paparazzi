/*
 * Copyright (C) 2023 MAVLab
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
 * @file modules/ins/ins_ext_pose.c
 * Integrated Navigation System interface.
 */


#include <time.h>

#include "ins_ext_pose.h"
#include "state.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "modules/imu/imu.h"
#include "modules/ahrs/ahrs.h"
#include "modules/ins/ins.h"
#include "generated/flight_plan.h"
#include "modules/core/abi.h"

#ifndef INS_EXT_POSE_P0
#define INS_EXT_POSE_P0 {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.}
#endif

#ifndef INS_EXT_POSE_Q_NOISE
#define INS_EXT_POSE_Q_NOISE {1.0, 1.0, 1.0, 0.0173, 4.878e-4, 3.547e-4}
#endif

#ifndef INS_EXT_POSE_R_NOISE
#define INS_EXT_POSE_R_NOISE {8.372e-6, 3.832e-6, 4.761e-6, 2.830e-4, 8.684e-6, 7.013e-6}
#endif

#if INS_EXT_POSE_DEBUG_PRINT
#include <stdio.h>
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) {}
#endif

#ifdef INS_EXT_VISION_ROTATION
struct FloatQuat ins_ext_vision_rot;
#endif

/** Data for telemetry and LTP origin.
 */
struct InsExtPose {
  /* Inputs */
  struct FloatRates gyros_f;
  struct FloatVect3 accels_f;
  bool   has_new_gyro;
  bool   has_new_acc;

  struct FloatVect3 ev_pos;
  struct FloatVect3 ev_vel;
  struct FloatEulers ev_att;
  struct FloatQuat ev_quat;
  bool   has_new_ext_pose;
  float  ev_time;

  /* Origin */
  struct LtpDef_i  ltp_def;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  /* status */
  bool started;
};

struct InsExtPose ins_ext_pose;


static void ins_ext_pose_init_from_flightplan(void)
{
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_ext_pose.ltp_def, &ecef_nav0);
  ins_ext_pose.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(MODULE_INS_EXT_POSE_ID, &ins_ext_pose.ltp_def);
  /* update local ENU coordinates of global waypoints */
  waypoints_localize_all();
}


/** Provide telemetry.
 */

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_ext_pose.ltp_pos.x, &ins_ext_pose.ltp_pos.y, &ins_ext_pose.ltp_pos.z,
                    &ins_ext_pose.ltp_speed.x, &ins_ext_pose.ltp_speed.y, &ins_ext_pose.ltp_speed.z,
                    &ins_ext_pose.ltp_accel.x, &ins_ext_pose.ltp_accel.y, &ins_ext_pose.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_baro_z = 0.0;
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      (float *)&fake_baro_z, &ins_ext_pose.ltp_pos.z,
                      &ins_ext_pose.ltp_speed.z, &ins_ext_pose.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_qfe = 0.0;
  pprz_msg_send_INS_REF(trans, dev, AC_ID,
                        &ins_ext_pose.ltp_def.ecef.x, &ins_ext_pose.ltp_def.ecef.y, &ins_ext_pose.ltp_def.ecef.z,
                        &ins_ext_pose.ltp_def.lla.lat, &ins_ext_pose.ltp_def.lla.lon, &ins_ext_pose.ltp_def.lla.alt,
                        &ins_ext_pose.ltp_def.hmsl, (float *)&fake_qfe);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t ahrs_id = AHRS_COMP_ID_GENERIC;
  uint8_t state_filter_mode = 0;
  uint16_t value = 0;

  if (ins_ext_pose.started) {
    state_filter_mode = 3; // OK
  }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_id, &state_filter_mode, &value);
}

static void send_external_pose_down(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_EXTERNAL_POSE_DOWN(trans, dev, AC_ID,
                        &ins_ext_pose.ev_time,
                        &ins_ext_pose.ev_pos.x,
                        &ins_ext_pose.ev_pos.y,
                        &ins_ext_pose.ev_pos.z,
                        &ins_ext_pose.ev_vel.x,
                        &ins_ext_pose.ev_vel.y,
                        &ins_ext_pose.ev_vel.z,
                        &ins_ext_pose.ev_quat.qi,
                        &ins_ext_pose.ev_quat.qx,
                        &ins_ext_pose.ev_quat.qy,
                        &ins_ext_pose.ev_quat.qz);
}

static void send_ahrs_bias(struct transport_tx *trans, struct link_device *dev)
{
  float dummy0 = 0.0;
  pprz_msg_send_AHRS_BIAS(trans, dev, AC_ID,
                &ekf_X[9],
                &ekf_X[10],
                &ekf_X[11],
                &ekf_X[12],
                &ekf_X[13],
                &ekf_X[14],
                &dummy0,
                &dummy0,
                &dummy0);
}
#endif


/**
 * Import Gyro and Acc from ABI.
 */

#ifndef INS_EXT_POSE_IMU_ID
#define INS_EXT_POSE_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EXT_POSE_IMU_ID)

static abi_event accel_ev;
static abi_event gyro_ev;

static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);


static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
  RATES_FLOAT_OF_BFP(ins_ext_pose.gyros_f, *gyro);
  ins_ext_pose.has_new_gyro = true;
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  ACCELS_FLOAT_OF_BFP(ins_ext_pose.accels_f, *accel);
  ins_ext_pose.has_new_acc = true;
}


/**
 * Import External Pose Message
 */

void ins_ext_pose_msg_update(uint8_t *buf)
{
  if (DL_EXTERNAL_POSE_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  struct EnuCoor_f enu_pos, enu_vel;
  enu_pos.x  = DL_EXTERNAL_POSE_enu_x(buf);
  enu_pos.y  = DL_EXTERNAL_POSE_enu_y(buf);
  enu_pos.z  = DL_EXTERNAL_POSE_enu_z(buf);
  enu_vel.x = DL_EXTERNAL_POSE_enu_xd(buf);
  enu_vel.y = DL_EXTERNAL_POSE_enu_yd(buf);
  enu_vel.z = DL_EXTERNAL_POSE_enu_zd(buf);
  float quat_i = DL_EXTERNAL_POSE_body_qi(buf);
  float quat_x = DL_EXTERNAL_POSE_body_qx(buf);
  float quat_y = DL_EXTERNAL_POSE_body_qy(buf);
  float quat_z = DL_EXTERNAL_POSE_body_qz(buf);

  DEBUG_PRINT("EXT_UPDATE\n");

  struct FloatQuat orient;
  struct FloatEulers orient_eulers;

  // Transformation of External Pose. Optitrack motive 2.X Yup
  orient.qi = quat_i ;
  orient.qx = quat_y ;
  orient.qy = quat_x ;
  orient.qz = -quat_z;

#ifdef INS_EXT_VISION_ROTATION
  // Rotate the quaternion
  struct FloatQuat rot_q;
  float_quat_comp(&rot_q, &orient, &ins_ext_vision_rot);
  QUAT_COPY(orient, rot_q);
#endif

  float_eulers_of_quat(&orient_eulers, &orient);

  ins_ext_pose.ev_time       = get_sys_time_usec();
  ENU_OF_TO_NED(ins_ext_pose.ev_pos, enu_pos);
  ENU_OF_TO_NED(ins_ext_pose.ev_vel, enu_vel);
  EULERS_COPY(ins_ext_pose.ev_att, orient_eulers);
  QUAT_COPY(ins_ext_pose.ev_quat, orient);

  ins_ext_pose.has_new_ext_pose = true;

  DEBUG_PRINT("Att = %f %f %f \n", ins_ext_pose.ev_att.phi, ins_ext_pose.ev_att.theta, ins_ext_pose.ev_att.psi);
}

/** EKF protos
 */

static void ekf_init(void);
static void ekf_run(void);

/** Module
 */


void ins_ext_pose_init(void)
{

  // Initialize inputs
  ins_ext_pose.has_new_acc = false;
  ins_ext_pose.has_new_gyro = false;
  ins_ext_pose.has_new_ext_pose = false;
  ins_ext_pose.started = false;

  // Get External Pose Origin From Flightplan
  ins_ext_pose_init_from_flightplan();

  // Provide telemetry
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EXTERNAL_POSE_DOWN, send_external_pose_down);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_BIAS, send_ahrs_bias);
#endif

  // Get IMU through ABI
  AbiBindMsgIMU_ACCEL(INS_EXT_POSE_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_GYRO(INS_EXT_POSE_IMU_ID, &gyro_ev, gyro_cb);
  // Get External Pose through datalink message: setup in xml

  // Initialize EKF
  ekf_init();
}

void ins_ext_pose_run(void)
{
  ekf_run();
}




/***************************************************
 * Kalman Filter.
 */
static void ekf_f(const float X[EKF_NUM_STATES],
                  const float U[EKF_NUM_INPUTS],
                  float out[EKF_NUM_STATES]);
static void ekf_F(const float X[EKF_NUM_STATES],
                  const float U[EKF_NUM_INPUTS],
                  float out[EKF_NUM_STATES][EKF_NUM_STATES]);
static void ekf_L(const float X[EKF_NUM_STATES],
                  const float U[EKF_NUM_INPUTS],
                  float out[EKF_NUM_STATES][EKF_NUM_INPUTS]);

static void ekf_prediction_step(const float U[EKF_NUM_INPUTS], const float dt);
static void ekf_measurement_step(const float Z[EKF_NUM_OUTPUTS]);



float ekf_X[EKF_NUM_STATES];
static float ekf_U[EKF_NUM_INPUTS];
static float ekf_Z[EKF_NUM_OUTPUTS];
static float ekf_P[EKF_NUM_STATES][EKF_NUM_STATES];
static float ekf_Q[EKF_NUM_INPUTS][EKF_NUM_INPUTS];
static float ekf_R[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];

static float ekf_H[EKF_NUM_OUTPUTS][EKF_NUM_STATES] = {
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}};


static float t0;
static float t1;

static void ekf_set_diag(float **a, float *b, int n)
{
  int i, j;
  for (i = 0 ; i < n; i++) {
    for (j = 0 ; j < n; j++) {
      if (i == j) {
        a[i][j] = b[i];
      } else {
        a[i][j] = 0.0;
      }
    }
  }
}



static void ekf_init(void)
{

  DEBUG_PRINT("ekf init");
  float X0[EKF_NUM_STATES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float U0[EKF_NUM_INPUTS] = {0, 0, 0, 0, 0, 0};
  float Z0[EKF_NUM_OUTPUTS] = {0, 0, 0, 0, 0, 0};

  float Pdiag[EKF_NUM_STATES] = INS_EXT_POSE_P0;
  float Qdiag[EKF_NUM_INPUTS] = INS_EXT_POSE_Q_NOISE;
  float Rdiag[EKF_NUM_OUTPUTS] = INS_EXT_POSE_R_NOISE;

  MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_INPUTS);
  MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);

  ekf_set_diag(ekf_P_, Pdiag, EKF_NUM_STATES);
  ekf_set_diag(ekf_Q_, Qdiag, EKF_NUM_INPUTS);
  ekf_set_diag(ekf_R_, Rdiag, EKF_NUM_OUTPUTS);
  float_vect_copy(ekf_X, X0, EKF_NUM_STATES);
  float_vect_copy(ekf_U, U0, EKF_NUM_INPUTS);
  float_vect_copy(ekf_Z, Z0, EKF_NUM_OUTPUTS);
}

static void ekf_f(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES])
{
  const float x0 = cosf(X[EKF_X_PSI]);
  const float x1 = U[EKF_U_ACC_X] - X[EKF_X_A_BIAS_X];
  const float x2 = cosf(X[EKF_X_THETA]);
  const float x3 = x1 * x2;
  const float x4 = U[EKF_U_ACC_Z] - X[EKF_X_A_BIAS_Z];
  const float x5 = sinf(X[EKF_X_PHI]);
  const float x6 = sinf(X[EKF_X_PSI]);
  const float x7 = x5 * x6;
  const float x8 = sinf(X[EKF_X_THETA]);
  const float x9 = cosf(X[EKF_X_PHI]);
  const float x10 = x0 * x9;
  const float x11 = U[EKF_U_ACC_Y] - X[EKF_X_A_BIAS_Y];
  const float x12 = x6 * x9;
  const float x13 = x0 * x5;
  const float x14 = tanf(X[EKF_X_THETA]);
  const float x15 = U[EKF_U_GYRO_Q] - X[EKF_X_G_BIAS_Q];
  const float x16 = x15 * x5;
  const float x17 = U[EKF_U_GYRO_R] - X[EKF_X_G_BIAS_R];
  const float x18 = x17 * x9;
  const float x19 = 1.0f / x2; // FIXME prevent div by zero
  out[EKF_X_POS_X] = X[EKF_X_VEL_X];
  out[EKF_X_POS_Y] = X[EKF_X_VEL_Y];
  out[EKF_X_POS_Z] = X[EKF_X_VEL_Z];
  out[EKF_X_VEL_X] = x0 * x3 + x11 * (-x12 + x13 * x8) + x4 * (x10 * x8 + x7);
  out[EKF_X_VEL_Y] = x11 * (x10 + x7 * x8) + x3 * x6 + x4 * (x12 * x8 - x13);
  out[EKF_X_VEL_Z] = -x1 * x8 + x11 * x2 * x5 + x2 * x4 * x9 + 9.81f;
  out[EKF_X_PHI] = U[EKF_U_GYRO_P] - X[EKF_X_G_BIAS_P] + x14 * x16 + x14 * x18;
  out[EKF_X_THETA] = x15 * x9 - x17 * x5;
  out[EKF_X_PSI] = x16 * x19 + x18 * x19;
  out[EKF_X_A_BIAS_X] = 0.f;
  out[EKF_X_A_BIAS_Y] = 0.f;
  out[EKF_X_A_BIAS_Z] = 0.f;
  out[EKF_X_G_BIAS_P] = 0.f;
  out[EKF_X_G_BIAS_Q] = 0.f;
  out[EKF_X_G_BIAS_R] = 0.f;
}

static void ekf_F(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS],
                         float out[EKF_NUM_STATES][EKF_NUM_STATES])
{
  const float x0 = U[EKF_U_ACC_Y] - X[EKF_X_A_BIAS_Y];
  const float x1 = sinf(X[EKF_X_PHI]);
  const float x2 = sinf(X[EKF_X_PSI]);
  const float x3 = x1 * x2;
  const float x4 = sinf(X[EKF_X_THETA]);
  const float x5 = cosf(X[EKF_X_PHI]);
  const float x6 = cosf(X[EKF_X_PSI]);
  const float x7 = x5 * x6;
  const float x8 = x4 * x7;
  const float x9 = x3 + x8;
  const float x10 = U[EKF_U_ACC_Z] - X[EKF_X_A_BIAS_Z];
  const float x11 = x2 * x5;
  const float x12 = x1 * x6;
  const float x13 = x12 * x4;
  const float x14 = x11 - x13;
  const float x15 = U[EKF_U_ACC_X] - X[EKF_X_A_BIAS_X];
  const float x16 = x15 * x4;
  const float x17 = cosf(X[EKF_X_THETA]);
  const float x18 = x0 * x17;
  const float x19 = x10 * x17;
  const float x20 = x17 * x2;
  const float x21 = x11 * x4;
  const float x22 = x12 - x21;
  const float x23 = -x3 * x4 - x7;
  const float x24 = x17 * x6;
  const float x25 = x17 * x5;
  const float x26 = x1 * x17;
  const float x27 = x4 * x5;
  const float x28 = U[EKF_U_GYRO_Q] - X[EKF_X_G_BIAS_Q];
  const float x29 = tanf(X[EKF_X_THETA]);
  const float x30 = x29 * x5;
  const float x31 = U[EKF_U_GYRO_R] - X[EKF_X_G_BIAS_R];
  const float x32 = x1 * x29;
  const float x33 = powf(x29, 2.f) + 1.f;
  const float x34 = x1 * x28;
  const float x35 = 1.0f / x17;
  const float x36 = x35 * x5;
  const float x37 = x1 * x35;
  const float x38 = powf(x17, -2.f);

  memset(out, 0, (EKF_NUM_STATES*EKF_NUM_STATES) * (sizeof **out));
  out[EKF_X_POS_X][EKF_X_VEL_X] = 1.f;
  out[EKF_X_POS_Y][EKF_X_VEL_Y] = 1.f;
  out[EKF_X_POS_Z][EKF_X_VEL_Z] = 1.f;
  out[EKF_X_VEL_X][EKF_X_PHI] = x0 * x9 + x10 * x14;
  out[EKF_X_VEL_X][EKF_X_THETA] = x12 * x18 - x16 * x6 + x19 * x7;
  out[EKF_X_VEL_X][EKF_X_PSI] = x0 * x23 + x10 * x22 - x15 * x20;
  out[EKF_X_VEL_X][EKF_X_A_BIAS_X] = -x24;
  out[EKF_X_VEL_X][EKF_X_A_BIAS_Y] = x14;
  out[EKF_X_VEL_X][EKF_X_A_BIAS_Z] = -x3 - x8;
  out[EKF_X_VEL_Y][EKF_X_PHI] = x0 * (-x12 + x21) + x10 * x23;
  out[EKF_X_VEL_Y][EKF_X_THETA] = x11 * x19 - x16 * x2 + x18 * x3;
  out[EKF_X_VEL_Y][EKF_X_PSI] = x0 * (-x11 + x13) + x10 * x9 + x15 * x24;
  out[EKF_X_VEL_Y][EKF_X_A_BIAS_X] = -x20;
  out[EKF_X_VEL_Y][EKF_X_A_BIAS_Y] = x23;
  out[EKF_X_VEL_Y][EKF_X_A_BIAS_Z] = x22;
  out[EKF_X_VEL_Z][EKF_X_PHI] = x0 * x25 - x10 * x26;
  out[EKF_X_VEL_Z][EKF_X_THETA] = -x0 * x1 * x4 - x10 * x27 + x17 * (-U[EKF_U_ACC_X] + X[EKF_X_A_BIAS_X]);
  out[EKF_X_VEL_Z][EKF_X_A_BIAS_X] = x4;
  out[EKF_X_VEL_Z][EKF_X_A_BIAS_Y] = -x26;
  out[EKF_X_VEL_Z][EKF_X_A_BIAS_Z] = -x25;
  out[EKF_X_PHI][EKF_X_PHI] = x28 * x30 - x31 * x32;
  out[EKF_X_PHI][EKF_X_THETA] = x31 * x33 * x5 + x33 * x34;
  out[EKF_X_PHI][EKF_X_G_BIAS_P] = -1.f;
  out[EKF_X_PHI][EKF_X_G_BIAS_Q] = -x32;
  out[EKF_X_PHI][EKF_X_G_BIAS_R] = -x30;
  out[EKF_X_THETA][EKF_X_PHI] = -x34 + x5 * (-U[EKF_U_GYRO_R] + X[EKF_X_G_BIAS_R]);
  out[EKF_X_THETA][EKF_X_G_BIAS_Q] = -x5;
  out[EKF_X_THETA][EKF_X_G_BIAS_R] = x1;
  out[EKF_X_PSI][EKF_X_PHI] = x28 * x36 - x31 * x37;
  out[EKF_X_PSI][EKF_X_THETA] = x27 * x31 * x38 + x34 * x38 * x4;
  out[EKF_X_PSI][EKF_X_G_BIAS_Q] = -x37;
  out[EKF_X_PSI][EKF_X_G_BIAS_R] = -x36;
}

static void ekf_L(const float X[EKF_NUM_STATES], __attribute__((unused))  const float U[EKF_NUM_INPUTS],
                         float out[EKF_NUM_STATES][EKF_NUM_INPUTS])
{
  const float x0 = cosf(X[EKF_X_THETA]);
  const float x1 = cosf(X[EKF_X_PSI]);
  const float x2 = sinf(X[EKF_X_PSI]);
  const float x3 = cosf(X[EKF_X_PHI]);
  const float x4 = x2 * x3;
  const float x5 = sinf(X[EKF_X_THETA]);
  const float x6 = sinf(X[EKF_X_PHI]);
  const float x7 = x1 * x6;
  const float x8 = x2 * x6;
  const float x9 = x1 * x3;
  const float x10 = tanf(X[EKF_X_THETA]);
  const float x11 = 1.f / x0; // FIXME protect against div by 0

  memset(out, 0, (EKF_NUM_STATES*EKF_NUM_INPUTS) * (sizeof **out));
  out[EKF_X_VEL_X][EKF_Z_POS_X] = -x0 * x1;
  out[EKF_X_VEL_X][EKF_Z_POS_Y] = x4 - x5 * x7;
  out[EKF_X_VEL_X][EKF_Z_POS_Z] = -x5 * x9 - x8;
  out[EKF_X_VEL_Y][EKF_Z_POS_X] = -x0 * x2;
  out[EKF_X_VEL_Y][EKF_Z_POS_Y] = -x5 * x8 - x9;
  out[EKF_X_VEL_Y][EKF_Z_POS_Z] = -x4 * x5 + x7;
  out[EKF_X_VEL_Z][EKF_Z_POS_X] = x5;
  out[EKF_X_VEL_Z][EKF_Z_POS_Y] = -x0 * x6;
  out[EKF_X_VEL_Z][EKF_Z_POS_Z] = -x0 * x3;
  out[EKF_X_PHI][EKF_Z_PHI] = -1.f;
  out[EKF_X_PHI][EKF_Z_THETA] = -x10 * x6;
  out[EKF_X_PHI][EKF_Z_PSI] = -x10 * x3;
  out[EKF_X_THETA][EKF_Z_THETA] = -x3;
  out[EKF_X_THETA][EKF_Z_PSI] = x6;
  out[EKF_X_PSI][EKF_Z_THETA] = -x11 * x6;
  out[EKF_X_PSI][EKF_Z_PSI] = -x11 * x3;
}


#if INS_EXT_POSE_USE_RK4

static void ekf_f_rk4(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], const float dt,
                             float out[EKF_NUM_STATES])
{
  float k1[EKF_NUM_STATES];
  float k2[EKF_NUM_STATES];
  float k3[EKF_NUM_STATES];
  float k4[EKF_NUM_STATES];

  float Xtmp[EKF_NUM_STATES];

  // k1   = f(X,U)
  ekf_f(X, U, k1);

  // Xtmp = X+dt*k1/2
  float_vect_smul(Xtmp, k1, dt / 2.f, EKF_NUM_STATES);
  float_vect_add(Xtmp, X, EKF_NUM_STATES);

  // k2   = f(Xtmp,U)
  ekf_f(Xtmp, U, k2);

  // Xtmp = X+dt*k2/2
  float_vect_smul(Xtmp, k2, dt / 2.f, EKF_NUM_STATES);
  float_vect_add(Xtmp, X, EKF_NUM_STATES);

  // k3   = f(Xtmp,U)
  ekf_f(Xtmp, U, k3);

  // Xtmp = X+dt*k3
  float_vect_smul(Xtmp, k3, dt, EKF_NUM_STATES);
  float_vect_add(Xtmp, X, EKF_NUM_STATES);

  // k4   = f(Xtmp,U)
  ekf_f(Xtmp, U, k4);

  // out = k2+k3
  float_vect_sum(out, k2, k3, EKF_NUM_STATES);
  // out *= 2
  float_vect_scale(out, 2.f, EKF_NUM_STATES);
  // out += k1
  float_vect_add(out, k1, EKF_NUM_STATES);
  // out += k4
  float_vect_add(out, k4, EKF_NUM_STATES);
  // out *= dt/6
  float_vect_scale(out, dt / 6.f, EKF_NUM_STATES);
  // out += X
  float_vect_add(out, X, EKF_NUM_STATES);
}

#endif

static void ekf_prediction_step(const float U[EKF_NUM_INPUTS], const float dt)
{
  float tmp[EKF_NUM_STATES][EKF_NUM_STATES];
  MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);

#if INS_EXT_POSE_USE_RK4
  // [1] Predicted (a priori) state estimate:
  float Xkk_1[EKF_NUM_STATES];
  ekf_f_rk4(ekf_X, U, dt, Xkk_1);

  // [2] Get matrices
  float F[EKF_NUM_STATES][EKF_NUM_STATES];
  float L[EKF_NUM_STATES][EKF_NUM_INPUTS];
  ekf_F(ekf_X, U, F);
  ekf_L(ekf_X, U, L);


  // [3] Continuous to discrete
  // Fd = eye(N) + F*dt + F*F*dt**2/2 = I + [I+F*dt/2]*F*dt
  // Ld = L*dt+F*L*dt**2/2            = [I+F*dt/2]*L*dt
  float Fd[EKF_NUM_STATES][EKF_NUM_STATES];
  float Ld[EKF_NUM_STATES][EKF_NUM_INPUTS];

  MAKE_MATRIX_PTR(F_, F, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(L_, L, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(Fd_, Fd, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(Ld_, Ld, EKF_NUM_STATES);

  // tmp = I+F*dt/2
  float_mat_diagonal_scal(tmp_, 1.f, EKF_NUM_STATES);
  float_mat_sum_scaled(tmp_, F_, dt / 2.f, EKF_NUM_STATES, EKF_NUM_STATES);

  // Ld = tmp*L*dt
  float_mat_mul(Ld_, tmp_, L_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_INPUTS);
  float_mat_scale(Ld_, dt, EKF_NUM_STATES, EKF_NUM_INPUTS);

  // Fd = tmp*F*dt
  float_mat_mul(Fd_, tmp_, F_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
  float_mat_scale(Fd_, dt, EKF_NUM_STATES, EKF_NUM_STATES);

  // Fd += I
  int i;
  for (i = 0; i < EKF_NUM_STATES; i++) {
    Fd[i][i] += 1.f;
  }

#else

  // [1] Predicted (a priori) state estimate:
  float Xkk_1[EKF_NUM_STATES];
  // Xkk_1 = f(X,U)
  ekf_f(ekf_X, U, Xkk_1);
  // Xkk_1 *= dt
  float_vect_scale(Xkk_1, dt, EKF_NUM_STATES);
  // Xkk_1 += X
  float_vect_add(Xkk_1, ekf_X, EKF_NUM_STATES);


  // [2] Get matrices
  float F[EKF_NUM_STATES][EKF_NUM_STATES];
  float Ld[EKF_NUM_STATES][EKF_NUM_INPUTS];
  ekf_F(ekf_X, U, F);
  ekf_L(ekf_X, U, Ld);


  // [3] Continuous to discrete
  // Fd = eye(N) + F*dt
  // Ld = L*dt
  float Fd[EKF_NUM_STATES][EKF_NUM_STATES];

  MAKE_MATRIX_PTR(F_, F, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(Fd_, Fd, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(Ld_, Ld, EKF_NUM_STATES);

  // Fd = I+F*dt
  float_mat_diagonal_scal(Fd_, 1.f, EKF_NUM_STATES);
  float_mat_sum_scaled(Fd_, F_, dt, EKF_NUM_STATES, EKF_NUM_STATES);

  // Ld = Ld*dt
  float_mat_scale(Ld_, dt, EKF_NUM_STATES, EKF_NUM_INPUTS);
#endif

  // [4] Predicted covariance estimate:
  // Pkk_1 = Fd*P*Fd.T + Ld*Q*Ld.T
  float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
  float LdT[EKF_NUM_INPUTS][EKF_NUM_STATES];
  float QLdT[EKF_NUM_INPUTS][EKF_NUM_STATES];

  MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(LdT_, LdT, EKF_NUM_INPUTS);
  MAKE_MATRIX_PTR(QLdT_, QLdT, EKF_NUM_INPUTS);

  // Fd = Fd.T
  float_mat_transpose_square(Fd_, EKF_NUM_STATES);

  // tmp = P*Fd
  float_mat_mul(tmp_, ekf_P_, Fd_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

  // Fd = Fd.T
  float_mat_transpose_square(Fd_, EKF_NUM_STATES);

  // Pkk_1 = Fd*tmp
  float_mat_mul(Pkk_1_, Fd_, tmp_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

  // LdT = Ld.T
  float_mat_transpose(LdT_, Ld_, EKF_NUM_STATES, EKF_NUM_INPUTS);

  // QLdT = Q*LdT
  float_mat_mul(QLdT_, ekf_Q_, LdT_, EKF_NUM_INPUTS, EKF_NUM_INPUTS, EKF_NUM_STATES);

  // tmp = Ld*QLdT
  float_mat_mul(tmp_, Ld_, QLdT_, EKF_NUM_STATES, EKF_NUM_INPUTS, EKF_NUM_STATES);

  // Pkk_1 += tmp
  float_mat_sum_scaled(Pkk_1_, tmp_, 1, EKF_NUM_STATES, EKF_NUM_STATES);

  // Store state and covariance

  // X = Xkk_1
  float_vect_copy(ekf_X, Xkk_1, EKF_NUM_STATES);

  // P = Pkk_1
  float_mat_copy(ekf_P_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES);
}

static void ekf_measurement_step(const float Z[EKF_NUM_OUTPUTS])
{
  // Xkk_1 = X
  float Xkk_1[EKF_NUM_STATES];
  float_vect_copy(Xkk_1, ekf_X, EKF_NUM_STATES);

  // Pkk_1 = P
  float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
  MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
  float_mat_copy(Pkk_1_, ekf_P_, EKF_NUM_STATES, EKF_NUM_STATES);

  // [5] Measurement residual:
  // yk = Z - H*Xkk_1
  float yk[EKF_NUM_OUTPUTS];

  MAKE_MATRIX_PTR(ekf_H_, ekf_H, EKF_NUM_OUTPUTS);

  float_mat_vect_mul(yk, ekf_H_, Xkk_1, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
  float_vect_scale(yk, -1.f, EKF_NUM_OUTPUTS);
  float_vect_add(yk, Z, EKF_NUM_OUTPUTS);


  // [6] Residual covariance:
  // Sk = H*Pkk_1*H.T + R
  float Sk[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
  float PHT[EKF_NUM_STATES][EKF_NUM_OUTPUTS];

  MAKE_MATRIX_PTR(Sk_, Sk, EKF_NUM_OUTPUTS);
  MAKE_MATRIX_PTR(PHT_, PHT, EKF_NUM_STATES);
  MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);

  // PHT = Pkk_1*H.T
  float_mat_transpose(PHT_, ekf_H_, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
  float_mat_mul_copy(PHT_, Pkk_1_, PHT_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_OUTPUTS);

  // Sk = H*PHT
  float_mat_mul(Sk_, ekf_H_, PHT_, EKF_NUM_OUTPUTS, EKF_NUM_STATES, EKF_NUM_OUTPUTS);

  // Sk += R
  float_mat_sum_scaled(Sk_, ekf_R_, 1.f, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


  // [7] Near-optimal Kalman gain:
  // K = Pkk_1*H.T*inv(Sk)
  float Sk_inv[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
  float K[EKF_NUM_STATES][EKF_NUM_OUTPUTS];

  MAKE_MATRIX_PTR(Sk_inv_, Sk_inv, EKF_NUM_OUTPUTS);
  MAKE_MATRIX_PTR(K_, K, EKF_NUM_STATES);

  // Sk_inv = inv(Sk)
  float_mat_invert(Sk_inv_, Sk_, EKF_NUM_OUTPUTS);

  // K = PHT*Sk_inv
  float_mat_mul(K_, PHT_, Sk_inv_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


  // [8] Updated state estimate
  // Xkk = Xkk_1 + K*yk
  float_mat_vect_mul(ekf_X, K_, yk, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
  float_vect_add(ekf_X, Xkk_1, EKF_NUM_STATES);


  // [9] Updated covariance estimate:
  // Pkk = (I - K*H)*Pkk_1
  float tmp[EKF_NUM_STATES][EKF_NUM_STATES];
  MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);

  // tmp = K*H
  float_mat_mul(tmp_, K_, ekf_H_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_STATES);

  // tmp *= -1
  float_mat_scale(tmp_, -1.f, EKF_NUM_STATES, EKF_NUM_STATES);

  // tmp += I
  int i;
  for (i = 0; i < EKF_NUM_STATES; i++) {
    tmp_[i][i] += 1.f;
  }
  // P = tmp*Pkk_1
  float_mat_mul(ekf_P_, tmp_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
}


static void ekf_run(void)
{
  // Time
  t1 = get_sys_time_float();
  float dt = t1 - t0;
  t0 = t1;

  // Only Start If External Pose is Available
  if (!ins_ext_pose.started) {
    // ekf starts at the first ev update
    if (ins_ext_pose.has_new_ext_pose) {
      ins_ext_pose.started = true;

      // initial guess
      ekf_X[EKF_X_POS_X] = ins_ext_pose.ev_pos.x;
      ekf_X[EKF_X_POS_Y] = ins_ext_pose.ev_pos.y;
      ekf_X[EKF_X_POS_Z] = ins_ext_pose.ev_pos.z;
      ekf_X[EKF_X_PHI] = ins_ext_pose.ev_att.phi;
      ekf_X[EKF_X_THETA] = ins_ext_pose.ev_att.theta;
      ekf_X[EKF_X_PSI] = ins_ext_pose.ev_att.psi;
    }
  }

  // set input values
  if (ins_ext_pose.has_new_acc) {
    ekf_U[EKF_U_ACC_X] = ins_ext_pose.accels_f.x;
    ekf_U[EKF_U_ACC_Y] = ins_ext_pose.accels_f.y;
    ekf_U[EKF_U_ACC_Z] = ins_ext_pose.accels_f.z;
    ins_ext_pose.has_new_acc = false;
  } else {
    DEBUG_PRINT("ekf missing acc\n");
  }
  if (ins_ext_pose.has_new_gyro) {
    ekf_U[EKF_U_GYRO_P] = ins_ext_pose.gyros_f.p;
    ekf_U[EKF_U_GYRO_Q] = ins_ext_pose.gyros_f.q;
    ekf_U[EKF_U_GYRO_R] = ins_ext_pose.gyros_f.r;
    ins_ext_pose.has_new_gyro = false;
  } else {
    DEBUG_PRINT("ekf missing gyro\n");
  }

  if (ins_ext_pose.started) {

    // prediction step
    ekf_prediction_step(ekf_U, dt);

    DEBUG_PRINT("ekf prediction step U = %f, %f, %f, %f, %f, %f dt = %f \n",
        ekf_U[0], ekf_U[1], ekf_U[2], ekf_U[3], ekf_U[4], ekf_U[5], dt);

    // measurement step
    if (ins_ext_pose.has_new_ext_pose) {

      //fix psi
      static float last_psi = 0;
      float delta_psi = ins_ext_pose.ev_att.psi - last_psi;
      last_psi = ins_ext_pose.ev_att.psi;

      if (delta_psi > M_PI) {
        delta_psi -= 2.f * M_PI;
      } else if (delta_psi < -M_PI) {
        delta_psi += 2.f * M_PI;
      }

      ekf_Z[EKF_Z_POS_X] = ins_ext_pose.ev_pos.x;
      ekf_Z[EKF_Z_POS_Y] = ins_ext_pose.ev_pos.y;
      ekf_Z[EKF_Z_POS_Z] = ins_ext_pose.ev_pos.z;
      ekf_Z[EKF_Z_PHI] = ins_ext_pose.ev_att.phi;
      ekf_Z[EKF_Z_THETA] = ins_ext_pose.ev_att.theta;
      ekf_Z[EKF_Z_PSI] += delta_psi;
      ins_ext_pose.has_new_ext_pose = false;

      ekf_measurement_step(ekf_Z);

      DEBUG_PRINT("ekf measurement step Z = %f, %f, %f, %f \n",
          ekf_Z[0], ekf_Z[1], ekf_Z[2], ekf_Z[3]);
    }
  }

  // Export Results
  struct NedCoor_f ned_pos;
  ned_pos.x = ekf_X[EKF_X_POS_X];
  ned_pos.y = ekf_X[EKF_X_POS_Y];
  ned_pos.z = ekf_X[EKF_X_POS_Z];
  POSITIONS_BFP_OF_REAL(ins_ext_pose.ltp_pos, ned_pos);

  struct NedCoor_f ned_speed;
  ned_speed.x = ekf_X[EKF_X_VEL_X];
  ned_speed.y = ekf_X[EKF_X_VEL_Y];
  ned_speed.z = ekf_X[EKF_X_VEL_Z];
  SPEEDS_BFP_OF_REAL(ins_ext_pose.ltp_speed, ned_speed);

  struct FloatEulers ned_to_body_eulers;
  ned_to_body_eulers.phi = ekf_X[EKF_X_PHI];
  ned_to_body_eulers.theta = ekf_X[EKF_X_THETA];
  ned_to_body_eulers.psi = ekf_X[EKF_X_PSI];

  struct FloatRates rates;
  rates.p = ekf_U[EKF_U_GYRO_P] - ekf_X[EKF_X_G_BIAS_P];
  rates.q = ekf_U[EKF_U_GYRO_Q] - ekf_X[EKF_X_G_BIAS_Q];
  rates.r = ekf_U[EKF_U_GYRO_R] - ekf_X[EKF_X_G_BIAS_R];

  struct FloatVect3 body_accel;
  body_accel.x = ekf_U[EKF_U_ACC_X] - ekf_X[EKF_X_A_BIAS_X];
  body_accel.y = ekf_U[EKF_U_ACC_Y] - ekf_X[EKF_X_A_BIAS_Y];
  body_accel.z = ekf_U[EKF_U_ACC_Z] - ekf_X[EKF_X_A_BIAS_Z];
  struct Int32Vect3 body_accel_i;
  ACCELS_BFP_OF_REAL(body_accel_i, body_accel);

  struct FloatVect3 ned_accel_v;
  struct FloatRMat ned_to_body_rmat;
  float_rmat_of_eulers(&ned_to_body_rmat, &ned_to_body_eulers);
  float_rmat_transp_vmult(&ned_accel_v, &ned_to_body_rmat, &body_accel);
  ned_accel_v.z += 9.81f;
  ACCELS_BFP_OF_REAL(ins_ext_pose.ltp_accel, ned_accel_v);
  struct NedCoor_f ned_accel;
  VECT3_COPY(ned_accel, ned_accel_v);

  stateSetPositionNed_f(MODULE_INS_EXT_POSE_ID, &ned_pos);
  stateSetSpeedNed_f(MODULE_INS_EXT_POSE_ID, &ned_speed);
  stateSetNedToBodyEulers_f(MODULE_INS_EXT_POSE_ID, &ned_to_body_eulers);
  stateSetBodyRates_f(MODULE_INS_EXT_POSE_ID, &rates);
  stateSetAccelNed_f(MODULE_INS_EXT_POSE_ID, &ned_accel);
  stateSetAccelBody_i(MODULE_INS_EXT_POSE_ID, &body_accel_i);

}



/**
 * Logging
 */

void ins_ext_pose_log_header(FILE *file)
{
  fprintf(file,
          "ekf_X1,ekf_X2,ekf_X3,ekf_X4,ekf_X5,ekf_X6,ekf_X7,ekf_X8,ekf_X9,ekf_X10,ekf_X11,ekf_X12,ekf_X13,ekf_X14,ekf_X15,");
  fprintf(file, "ekf_U1,ekf_U2,ekf_U3,ekf_U4,ekf_U5,ekf_U6,");
  fprintf(file, "ekf_Z1,ekf_Z2,ekf_Z3,ekf_Z4,");
}

void ins_ext_pose_log_data(FILE *file)
{
  fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,",
      ekf_X[0], ekf_X[1], ekf_X[2], ekf_X[3], ekf_X[4],
      ekf_X[5], ekf_X[6], ekf_X[7], ekf_X[8], ekf_X[9],
      ekf_X[10], ekf_X[11], ekf_X[12], ekf_X[13], ekf_X[14]);
  fprintf(file, "%f,%f,%f,%f,%f,%f,",
      ekf_U[0], ekf_U[1], ekf_U[2], ekf_U[3], ekf_U[4], ekf_U[5]);
  fprintf(file, "%f,%f,%f,%f,", ekf_Z[0], ekf_Z[1], ekf_Z[2], ekf_Z[3]);
}
