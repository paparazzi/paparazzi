/*
 * Copyright (C) 2021 Guido de Croon <g.c.h.e.decroon@tudelft.nl>
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
 * @file subsystems/ins/ins_flow.c
 *
 * INS_flow estimates the attitude (and can also estimate velocity and height), based on optical flow and gyro
 * measurements alone. This module is inspired by the fact that flying insects do not have accelerometers. It
 * formed the basis for the experiments in which the attitude is estimated with only optical
 * flow and gyro measurements (without accelerometers), published in Nature:
 *
 * De Croon, G.C.H.E., Dupeyroux, J.J., De Wagter, C., Chatterjee, A., Olejnik, D. A., & Ruffier, F. (2022).
 * Accommodating unobservability to control flight attitude with optic flow. Nature, 610(7932), 485-490.
 * https://www.nature.com/articles/s41586-022-05182-2
 *
 */

#include "ins_flow.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "mcu_periph/sys_time.h"
#include "modules/actuators/motor_mixing.h"
#include <stdio.h>

#define DEBUG_INS_FLOW 0
#if DEBUG_INS_FLOW
#include <stdio.h>
#include "math/pprz_simple_matrix.h"
#define DEBUG_PRINT  printf
#define DEBUG_MAT_PRINT MAT_PRINT
//#define DEBUG_MAT_PRINT(...)
#else
#define DEBUG_PRINT(...)
#define DEBUG_MAT_PRINT(...)
#endif

#ifndef AHRS_ICQ_OUTPUT_ENABLED
#define AHRS_ICQ_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_ICQ_OUTPUT_ENABLED)

/* default Gyro to use in INS */
#ifndef INS_FLOW_GYRO_ID
#define INS_FLOW_GYRO_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FLOW_GYRO_ID)

/* default Accelerometer to use in INS */
#ifndef INS_FLOW_ACCEL_ID
#define INS_FLOW_ACCEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FLOW_ACCEL_ID)

/* default IMU lowpass to use in INS */
#ifndef INS_FLOW_IMU_ID
#define INS_FLOW_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FLOW_IMU_ID)


/* default GPS to use in INS */
#ifndef INS_FLOW_GPS_ID
#define INS_FLOW_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_FLOW_GPS_ID)

/* Use optical flow estimates */
#ifndef INS_OPTICAL_FLOW_ID
#define INS_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_OPTICAL_FLOW_ID)

// reading RPMs:
#ifndef INS_RPM_ID
#define INS_RPM_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_RPM_ID)

/* All registered ABI events */
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event gps_ev;
static abi_event body_to_imu_ev;
static abi_event ins_optical_flow_ev;
static abi_event ins_RPM_ev;
static abi_event aligner_ev;
//static abi_event mag_ev;
//static abi_event geo_mag_ev;

/* All ABI callbacks */
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
/*static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t __attribute__((unused)) stamp,
                   struct Int32Vect3 *mag);
static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h);*/
//static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
void ins_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x,
                         int32_t flow_y, int32_t flow_der_x, int32_t flow_der_y, float quality, float size_divergence);
static void ins_rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t num_act);
static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag);
static void print_ins_flow_state(void);
static void print_true_state(void);
/* Static local functions */
//static bool ahrs_icq_output_enabled;
static uint32_t ahrs_icq_last_stamp;
static uint8_t ahrs_flow_id = AHRS_COMP_ID_FLOW;  ///< Component ID for FLOW
static void set_body_state_from_quat(void);
static void ins_reset_filter(void);

struct InsFlow {

  // data elements for gps passthrough:
  struct LtpDef_i  ltp_def;
  bool ltp_initialized;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  // vision measurements:
  float optical_flow_x;
  float optical_flow_y;
  float divergence;
  float vision_time; // perhaps better to use microseconds (us) instead of float in seconds
  bool new_flow_measurement;

  // RPMs:
  uint16_t RPM[8]; // max an octocopter
  uint8_t RPM_num_act;

  float lp_gyro_pitch;
  float lp_gyro_bias_pitch; // determine the bias before take-off
  float lp_gyro_roll;
  float lp_gyro_bias_roll; // determine the bias before take-off
  float thrust_factor; // determine the additional required scale factor to have unbiased thrust estimates
  float lp_thrust;
  float lp_roll_command;

};
struct InsFlow ins_flow;

// Kalman filter parameters and variables:

#define MOMENT_DELAY 20
float moments[MOMENT_DELAY] = {0.};
int moment_ind;

float OF_X[N_STATES_OF_KF] = {0.};
float OF_Q[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
float OF_P[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
float OF_R[N_MEAS_OF_KF][N_MEAS_OF_KF] = {{0.}};

#define OF_N_ROTORS 4
float RPM_FACTORS[OF_N_ROTORS];

float of_time;
float of_prev_time;
float lp_factor;
float lp_factor_strong;
bool reset_filter;
int use_filter;
bool run_filter;
uint32_t counter;
float thrust_factor;

float GT_phi;
float GT_theta;


#define USE_STANDARD_PARAMS 0

#if USE_STANDARD_PARAMS
// Note that not all of the values are used.
// Moment of Inertia, mass, distance motors from center of gravity, 4 params for thrust and moment generation,
// measurement noise R (2), actuation noise Q(5),
// initial P (5), linear drag factor
#if USE_NPS
float parameters[20] = {0.0018244, 0.400, 0.085, 0.152163; 0.170734; 0.103436; 0.122109,
                        0.1, 0.1, 0.1, 1.0f * M_PI / 180.0f, 100.0f * M_PI / 180.0f, 0.1f, 3.0f,
                        1.0f, 10.0f * M_PI / 180.0f, 10.0f * M_PI / 180.0f, 1.0f, 1.0f, 0.5f
                       };
#else
float parameters[20] = {0.0018244, 0.400, 0.085, 0.108068 0.115448 0.201207 0.208834,
                        0.1, 0.1, 0.1, 1.0f * M_PI / 180.0f, 100.0f * M_PI / 180.0f, 0.1f, 3.0f,
                        1.0f, 10.0f * M_PI / 180.0f, 10.0f * M_PI / 180.0f, 1.0f, 1.0f, 0.5f
                       };
#endif
#else
// Define parameters for the filter, fitted in MATLAB:
#if USE_NPS
#if N_MEAS_OF_KF == 3
// with rate measurement:
float parameters[20] = {1.234994e-01, 3.603662e-01, 8.751691e-02, 1.636867e-01, 1.561769e-01, 1.856140e-01, 1.601066e-02, 1.187989e-01, 1.507075e-01, 2.471644e-01, 7.934140e-02, 1.770048e+00, 1.345862e-01, 2.881410e+00, 1.003584e+00, 1.280523e-01, 7.549402e-02, 9.640423e-01, 1.078312e+00, 3.468849e-01};
#else
// without rate state / measurement:
#if CONSTANT_ALT_FILTER
#if OF_DRAG
float parameters[20] = {1.396428e-01, 2.517970e-01, 3.575834e-02, 2.626194e-01, 1.078661e-01, 3.126137e-01, 4.621823e-02, 3.258048e-01, 8.456147e-02, 2.275105e-01, 2.820394e-02, 1.937395e+00, -4.259889e-02, 2.755648e+00, 1.000810e+00, -3.474577e-03, 3.146387e-01, 8.809383e-01, 9.878757e-01, 6.741976e-01};
#else
float parameters[20] = {3.363769e-01, 4.917425e-01, 1.903805e-01, 2.945672e-01, 1.258647e-01, 1.513736e-01, 5.894541e-01, 2.162745e-01, 5.527361e-01, 1.385623e-01, 8.307731e-01, 1.488212e+00, 2.439721e-01, 3.052758e+00, 8.246426e-01, 9.988101e-02, 1.247046e-01, 8.834364e-01, 7.971876e-01, 1.112319e+00};
#endif
#else
float parameters[20] = {4.370754e-02, 3.770587e-01, 1.187542e-01, 1.174995e-01, 1.419432e-01, 6.950201e-02, 2.251078e-01, 9.113943e-02, 2.230198e-01, 5.767389e-02, 1.855676e-02, 1.676359e+00, 5.822681e-02, 2.869468e+00, 1.140625e+00, 6.831383e-02, 1.600776e-01, 9.853843e-01, 1.000381e+00, 5.081224e-01};
#endif
#endif
#else
// TODO: train constant alt filter without drag, also with and without measuring the gyro.
#if CONSTANT_ALT_FILTER
#if OF_DRAG
// float parameters[20] = {1.557784e-01, 3.186275e-01, 8.341852e-02, 9.320449e-02, 1.706694e-01, 3.950497e-01, 3.338107e-01, 1.947852e-01, 2.429782e-01, 1.216562e-01, 2.885142e-01, 1.765480e+00, 2.427392e-01, 3.014556e+00, 1.004227e+00, 1.798174e-01, 2.821081e-01, 9.314043e-01, 1.005090e+00, 2.630276e-01};
float parameters[20] = {8.407886e-03, 4.056093e-01, 1.555919e-01, 1.291584e-01, 2.594766e-01, 1.927331e-01, 9.599609e-02, 1.688265e-01, 5.589618e-02, 1.605665e-01, 1.195912e-01, 1.809532e+00, 4.268251e-02, 3.003060e+00, 1.098473e+00, 1.944433e-01, 2.363352e-01, 1.110390e+00, 1.190994e+00, 6.211962e-01};
#endif
#else
#if N_MEAS_OF_KF == 3
#if PREDICT_GYROS == 0
// with rate measurement (last two values copied from the last condition)
float parameters[22] = {0.041001, 1.015066, -0.058495, 0.498353, -0.156362, 0.383511, 0.924635, 0.681918, 0.318947, 0.298235, 0.224906, 1.371037, 0.008888, 3.045428, 0.893953, 0.529789, 0.295028, 1.297515, 0.767550, 0.334040, 0.192238, 0.301966};
#else
// Estimate gyro directly instead of moment:
float parameters[26] = {1.065019, 0.270407, 0.164520, 0.008321, 0.083628, 0.261853, 0.210707, 0.204501, 0.164267, 0.097979, 0.053705, 1.640180, 0.151171, 3.086366, 1.025684, 0.011813, 0.177164, 0.995710, 1.050374, 0.617920, 0.028360, 0.447258, 0.077277, 0.360559, 0.555940, 0.133979};
#endif
#else
// without rate measurement:
// float parameters[20] = {4.098677e-01, 7.766318e-01, 3.614751e-01, 4.745865e-01, 5.144065e-01, 3.113647e-01, -8.737287e-03, 6.370274e-01, 3.863760e-01, -3.527670e-01, 4.873666e-01, 1.688456e+00, -6.037967e-02, 2.759148e+00, 1.385455e+00, 1.044881e-01, -1.170409e-01, 1.126136e+00, 1.097562e+00, 2.680243e-01};
float parameters[22] = {0.219033, 0.376572, 0.184002, 0.096388, 0.240843, 0.172390, 0.133111, 0.495885, 0.357086, 0.233624, 0.125611, 1.661682, 0.136735, 2.812652, 0.715887, 0.166932, 0.371409, 1.043920, 0.840683, 0.567703, 0.192238, 0.301966};
#endif
#endif
#endif
#endif
// parameter indices (TODO: how important are these numbers? Some are not used, others, like P may be not so important).
#define PAR_IX 0
#define PAR_MASS 1
#define PAR_BASE 2
#define PAR_K0 3
#define PAR_K1 4
#define PAR_K2 5
#define PAR_K3 6
#define PAR_R0 7
#define PAR_R1 8
#define PAR_Q0 9
#define PAR_Q1 10
#define PAR_Q2 11
#define PAR_Q3 12
#define PAR_Q4 13
#define PAR_P0 14
#define PAR_P1 15
#define PAR_P2 16
#define PAR_P3 17
#define PAR_P4 18
#define PAR_KD 19
#define PAR_Q_TB 20
#define PAR_P_TB 21
#define PAR_PRED_ROLL_1 22
#define PAR_PRED_ROLL_2 23
#define PAR_PRED_ROLL_3 24



/*
struct InsFlowState {
  // vector representation of state:
  // v, angle, angle_dot, z, z_dot


};
struct InsFlowState ins_flow_state;
*/

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"
#include "state.h"

// attitude part:
static void send_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_QUAT_INT(trans, dev, AC_ID,
                              &ahrs_icq.weight,
                              &ahrs_icq.ltp_to_body_quat.qi,
                              &ahrs_icq.ltp_to_body_quat.qx,
                              &ahrs_icq.ltp_to_body_quat.qy,
                              &ahrs_icq.ltp_to_body_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz),
                              &ahrs_flow_id);
}

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Eulers ltp_to_imu_euler;
  int32_eulers_of_quat(&ltp_to_imu_euler, &ahrs_icq.ltp_to_body_quat);
  struct Int32Eulers *eulers = stateGetNedToBodyEulers_i();
  pprz_msg_send_AHRS_EULER_INT(trans, dev, AC_ID,
                               &ltp_to_imu_euler.phi,
                               &ltp_to_imu_euler.theta,
                               &ltp_to_imu_euler.psi,
                               &(eulers->phi),
                               &(eulers->theta),
                               &(eulers->psi),
                               &ahrs_flow_id);

}

static void send_bias(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
                                   &ahrs_icq.gyro_bias.p, &ahrs_icq.gyro_bias.q,
                                   &ahrs_icq.gyro_bias.r, &ahrs_flow_id);
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatVect3 h_float;
  h_float.x = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.x);
  h_float.y = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.y);
  h_float.z = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.z);
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &h_float.x, &h_float.y, &h_float.z, &ahrs_flow_id);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_icq.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ahrs_icq_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_flow_id, &mde, &val);
}

// ins part
static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_flow.ltp_pos.x, &ins_flow.ltp_pos.y, &ins_flow.ltp_pos.z,
                    &ins_flow.ltp_speed.x, &ins_flow.ltp_speed.y, &ins_flow.ltp_speed.z,
                    &ins_flow.ltp_accel.x, &ins_flow.ltp_accel.y, &ins_flow.ltp_accel.z);
}

/*static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_baro_z = 0.0;
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      (float *)&fake_baro_z, &ins_flow.ltp_pos.z,
                      &ins_flow.ltp_speed.z, &ins_flow.ltp_accel.z);
}*/

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_qfe = 0.0;
  if (ins_flow.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_flow.ltp_def.ecef.x, &ins_flow.ltp_def.ecef.y, &ins_flow.ltp_def.ecef.z,
                          &ins_flow.ltp_def.lla.lat, &ins_flow.ltp_def.lla.lon, &ins_flow.ltp_def.lla.alt,
                          &ins_flow.ltp_def.hmsl, (float *)&fake_qfe);
  }
}


#endif

static void send_ins_flow(struct transport_tx *trans, struct link_device *dev)
{
  // TODO: add sending of theta:
  struct FloatEulers *eulers = stateGetNedToBodyEulers_f();
  struct NedCoor_f *position = stateGetPositionNed_f();
  struct NedCoor_f *velocities = stateGetSpeedNed_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  float phi = (180.0 / M_PI) * OF_X[OF_ANGLE_IND];
  float theta;
  if (OF_TWO_DIM) {
    theta = (180.0 / M_PI) * OF_X[OF_THETA_IND];
  } else {
    // if not filtering the second dimension, just take the ground truth
    theta = (180.0 / M_PI) * eulers->theta;
  }

  float phi_dot = 0.0f;
  float z_dot = 0.0f;
  if (!CONSTANT_ALT_FILTER) {
    phi_dot = (180.0 / M_PI) * OF_X[OF_ANGLE_DOT_IND];
    z_dot = OF_X[OF_Z_DOT_IND];
  }

  struct FloatRMat *NTB = stateGetNedToBodyRMat_f();
  struct FloatVect3 NED_velocities, body_velocities;
  NED_velocities.x = velocities->x;
  NED_velocities.y = velocities->y;
  NED_velocities.z = velocities->z;
  float_rmat_vmult(&body_velocities, NTB, &NED_velocities);

  float vy_GT = body_velocities.y;

  float phi_GT;
  if (use_filter < USE_ANGLE) {
    phi_GT = (180.0 / M_PI) * eulers->phi;
  } else {
    phi_GT = (180.0 / M_PI) * GT_phi;
  }
  float vx_GT = body_velocities.x;
  float theta_GT;
  if (use_filter < USE_ANGLE) {
    theta_GT = (180.0 / M_PI) * eulers->theta;
  } else if (OF_TWO_DIM) {
    theta_GT = (180.0 / M_PI) * GT_theta;
  }
  float p_GT = rates->p;
  float q_GT = rates->q;
  float z_GT = -position->z;
  float vz_GT = -velocities->z;

  float vy = OF_X[OF_V_IND];
  float vx;
  if (OF_TWO_DIM) {
    vx = OF_X[OF_VX_IND];
  } else {
    vx = vx_GT;
  }
  float z = OF_X[OF_Z_IND];
  float p, q;
  if (!CONSTANT_ALT_FILTER) {
    // normally:
    p = phi_dot;
    // when estimating the gyros:
    // // p = -1.8457e-04 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);
    // p = -2.0e-03 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);
    // TODO: expand the full filter later as well, to include q:
    q = q_GT;
  } else {
    p = ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll;
    // p = -2.0e-03 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);
    q = ins_flow.lp_gyro_pitch - ins_flow.lp_gyro_bias_pitch;
  }

  float thrust_bias;
  if (!OF_THRUST_BIAS || CONSTANT_ALT_FILTER) {
    thrust_bias = 0.0f;
  } else {
    thrust_bias = OF_X[OF_THRUST_BIAS_IND];
  }
  // This code is to actually compare the unbiased thrust with gravity:
  // TODO: code copied from below, put in a function?
  float thrust = 0.0f;
  for (int i = 0; i < OF_N_ROTORS; i++) {
    thrust += RPM_FACTORS[i] * ins_flow.RPM[i] * ins_flow.RPM[i];
  }
  thrust *= thrust_factor; // ins_flow.thrust_factor;
  thrust -= thrust_bias;
  float mass = parameters[PAR_MASS];
  float g = 9.81;
  float actual_lp_thrust = mass * g;
  thrust_bias = thrust - actual_lp_thrust;

  pprz_msg_send_INS_FLOW_INFO(trans, dev, AC_ID,
                              &vy, &phi, &p, &vx, &theta, &q, &z, &z_dot,
                              &vy_GT, &phi_GT, &p_GT, &vx_GT, &theta_GT, &q_GT,
                              &z_GT, &vz_GT, &thrust_bias, &use_filter);
}

void ins_reset_filter(void)
{

  // (re-)initialize the state:
  for (int i = 0; i < N_STATES_OF_KF; i++) {
    OF_X[i] = 0.0f;
  }
  OF_X[OF_Z_IND] = 1.0; // nonzero z

  // P-matrix:
  for (int i = 0; i < N_STATES_OF_KF; i++) {
    for (int j = 0; j < N_STATES_OF_KF; j++) {
      OF_P[i][j] = 0.0f;
    }
  }
  if (CONSTANT_ALT_FILTER == 1) {
    OF_P[OF_V_IND][OF_V_IND] = parameters[PAR_P0];
    OF_P[OF_ANGLE_IND][OF_ANGLE_IND] = parameters[PAR_P1];
    OF_P[OF_Z_IND][OF_Z_IND] = parameters[PAR_P3];
    OF_P[OF_ANGLE_DOT_IND][OF_ANGLE_DOT_IND] = parameters[PAR_P2];
    if (OF_TWO_DIM) {
      OF_P[OF_THETA_IND][OF_THETA_IND] = parameters[PAR_P1];
      OF_P[OF_VX_IND][OF_VX_IND] = parameters[PAR_P0];
    }
  } else {
    OF_P[OF_V_IND][OF_V_IND] = parameters[PAR_P0];
    OF_P[OF_ANGLE_IND][OF_ANGLE_IND] = parameters[PAR_P1];
    OF_P[OF_ANGLE_DOT_IND][OF_ANGLE_DOT_IND] = parameters[PAR_P2];
    OF_P[OF_Z_IND][OF_Z_IND] = parameters[PAR_P3];
    OF_P[OF_Z_DOT_IND][OF_Z_DOT_IND] = parameters[PAR_P4];
    if (OF_THRUST_BIAS) {
      OF_P[OF_THRUST_BIAS_IND][OF_THRUST_BIAS_IND] = parameters[PAR_P_TB];//OF_TB_P;
    }
  }

  counter = 0;

  // TODO: what to do with thrust, gyro bias, and low-passed roll command?

}


/* Initialize the flow ins */
void ins_flow_init(void)
{

  //ahrs_icq_output_enabled = AHRS_ICQ_OUTPUT_ENABLED;
  ahrs_icq_init();
  //ahrs_register_impl(ahrs_icq_enable_output);

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);
  struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &ecef_nav0);

  ltp_def_from_ecef_i(&ins_flow.ltp_def, &ecef_nav0);
  ins_flow.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_flow.ltp_def);
  ins_flow.ltp_initialized = true;
  ins_flow.new_flow_measurement = false;
  ins_flow.lp_gyro_pitch = 0.0f;
  ins_flow.lp_gyro_bias_pitch = 0.0f;
  ins_flow.lp_gyro_roll = 0.0f;
  ins_flow.lp_gyro_bias_roll = 0.0f;
  ins_flow.thrust_factor = 1.0f;
  ins_flow.lp_thrust = 0.0f;
  ins_flow.lp_roll_command = 0.0f;

  lp_factor = 0.95;
  lp_factor_strong = 1 - 1E-3;

  GT_phi = 0.0f;
  GT_theta = 0.0f;

  // Extended Kalman filter:
  // reset the state and P matrix:
  ins_reset_filter();

  // R-matrix, measurement noise (TODO: make params)
  OF_R[OF_LAT_FLOW_IND][OF_LAT_FLOW_IND] = parameters[PAR_R0];
  OF_R[OF_DIV_FLOW_IND][OF_DIV_FLOW_IND] = parameters[PAR_R1];
  if (OF_TWO_DIM) {
    OF_R[OF_LAT_FLOW_X_IND][OF_LAT_FLOW_X_IND] = parameters[PAR_R0];
  } else if (N_MEAS_OF_KF == 3) {
    OF_R[OF_RATE_IND][OF_RATE_IND] = 1.0 * (M_PI / 180.0f); // not a param yet, used to be 10, but we could trust it more
  }
  // Q-matrix, actuation noise (TODO: make params)
  OF_Q[OF_V_IND][OF_V_IND] = parameters[PAR_Q0];
  OF_Q[OF_ANGLE_IND][OF_ANGLE_IND] = parameters[PAR_Q1];
  OF_Q[OF_Z_IND][OF_Z_IND] = parameters[PAR_Q3];
  OF_Q[OF_ANGLE_DOT_IND][OF_ANGLE_DOT_IND] = parameters[PAR_Q2];
  if (!CONSTANT_ALT_FILTER) {
    OF_Q[OF_Z_DOT_IND][OF_Z_DOT_IND] = parameters[PAR_Q4];
  } else if (OF_TWO_DIM) {
    OF_Q[OF_VX_IND][OF_VX_IND] = parameters[PAR_Q0];
    OF_Q[OF_THETA_IND][OF_THETA_IND] = parameters[PAR_Q1];
  }
  if (OF_THRUST_BIAS) {
    OF_Q[OF_THRUST_BIAS_IND][OF_THRUST_BIAS_IND] = parameters[PAR_Q_TB];//OF_TB_Q;
  }


  // based on a fit, factor * rpm^2:
  // TODO: with the parameters, we don't need this if / else any more.
#if USE_NPS
  // K = [0.152163; 0.170734; 0.103436; 0.122109] * 1E-7;
  // K = [0.222949; 0.160458; 0.114227; 0.051396] * 1E-7;
  // rpm:
  // [2708.807954; 2587.641476; -379.728916; -501.203388]
  RPM_FACTORS[0] = parameters[PAR_K0] * 1E-7;
  RPM_FACTORS[1] = parameters[PAR_K1] * 1E-7;
  RPM_FACTORS[2] = parameters[PAR_K2] * 1E-7;
  RPM_FACTORS[3] = parameters[PAR_K3] * 1E-7;
#else
  // % Bebop 2, #45
  // From fit_TM_2 script:
  // K = [0.108068; 0.115448; 0.201207; 0.208834] * 1E-7
  RPM_FACTORS[0] = parameters[PAR_K0] * 1E-7;
  RPM_FACTORS[1] = parameters[PAR_K1] * 1E-7;
  RPM_FACTORS[2] = parameters[PAR_K2] * 1E-7;
  RPM_FACTORS[3] = parameters[PAR_K3] * 1E-7;
#endif

  reset_filter = false;
  use_filter = 0;
  run_filter = false;

  of_time = get_sys_time_float();
  of_prev_time = get_sys_time_float();

  // align the AHRS:
  ahrs_aligner_init();
  // set the initial attitude:
  set_body_state_from_quat();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_QUAT_INT, send_quat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER_INT, send_euler);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_GYRO_BIAS_INT, send_bias);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GEO_MAG, send_geo_mag);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
#endif

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
// TODO: warning: passing argument 3 of ‘AbiBindMsgIMU_GYRO_INT’ from incompatible pointer type
  AbiBindMsgIMU_GYRO(INS_FLOW_GYRO_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL(INS_FLOW_ACCEL_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(INS_FLOW_GPS_ID, &gps_ev, gps_cb);
  //AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgOPTICAL_FLOW(INS_OPTICAL_FLOW_ID, &ins_optical_flow_ev, ins_optical_flow_cb);
  AbiBindMsgRPM(INS_RPM_ID, &ins_RPM_ev, ins_rpm_cb);
  AbiBindMsgIMU_LOWPASSED(INS_FLOW_IMU_ID, &aligner_ev, aligner_cb);
  // AbiBindMsgIMU_MAG_INT32(ABI_BROADCAST, &mag_ev, mag_cb);
  // AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);

  // Telemetry:
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_FLOW_INFO, send_ins_flow);

  moment_ind = 0;

}

void ins_reset_local_origin(void)
{
  ltp_def_from_ecef_i(&ins_flow.ltp_def, &gps.ecef_pos);
  ins_flow.ltp_def.lla.alt = gps.lla_pos.alt;
  ins_flow.ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ins_flow.ltp_def);
  ins_flow.ltp_initialized = true;
}

void ins_optical_flow_cb(uint8_t sender_id UNUSED, uint32_t stamp, int32_t flow_x UNUSED,
                         int32_t flow_y UNUSED,
                         int32_t flow_der_x, int32_t flow_der_y, float quality UNUSED, float size_divergence)
{

  // TODO: make parameters:
  float subpixel_factor = 10.0f;
  float focal_x = 347.22;
  float new_time = ((float)stamp) / 1e6;
  float fps = 1.0f / (new_time - ins_flow.vision_time);
  ins_flow.optical_flow_x = (((float)flow_x) * fps) / (subpixel_factor * focal_x);
  ins_flow.optical_flow_y = (((float)flow_y) * fps) / (subpixel_factor * focal_x);
  ins_flow.divergence = 1.27 * size_divergence * fps;
  //printf("Reading %f, %f, %f\n", ins_flow.optical_flow_x, ins_flow.optical_flow_y, ins_flow.divergence);
  ins_flow.vision_time = new_time;
  ins_flow.new_flow_measurement = true;

}

void print_ins_flow_state(void)
{
  if (CONSTANT_ALT_FILTER) {
    if (!OF_TWO_DIM) {
      printf("v = %f, angle = %f, angle_dot = %f, z = %f.\n",
             OF_X[OF_V_IND], OF_X[OF_ANGLE_IND], OF_X[OF_ANGLE_DOT_IND], OF_X[OF_Z_IND]);
    } else {
      printf("v = %f, angle = %f, angle_dot = %f, z = %f, vx = %f, theta = %f.\n",
             OF_X[OF_V_IND], OF_X[OF_ANGLE_IND], OF_X[OF_ANGLE_DOT_IND], OF_X[OF_Z_IND], OF_X[OF_VX_IND], OF_X[OF_THETA_IND]);
    }

  } else {
    if (!OF_THRUST_BIAS) {
      printf("v = %f, angle = %f, angle_dot = %f, z = %f, z_dot = %f.\n",
             OF_X[OF_V_IND], OF_X[OF_ANGLE_IND], OF_X[OF_ANGLE_DOT_IND], OF_X[OF_Z_IND], OF_X[OF_Z_DOT_IND]);
    } else {
      printf("v = %f, angle = %f, angle_dot = %f, z = %f, z_dot = %f, thrust bias = %f.\n",
             OF_X[OF_V_IND], OF_X[OF_ANGLE_IND], OF_X[OF_ANGLE_DOT_IND], OF_X[OF_Z_IND], OF_X[OF_Z_DOT_IND],
             OF_X[OF_THRUST_BIAS_IND]);
    }

  }

}

void print_true_state(void)
{
  // TODO: rotate velocities to body frame:
  // TODO: add also the theta axis:
  struct FloatEulers *eulers = stateGetNedToBodyEulers_f();
  struct NedCoor_f *position = stateGetPositionNed_f();
  struct NedCoor_f *velocities = stateGetSpeedNed_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  printf("True: v = %f, angle = %f, angle_dot = %f, z = %f, z_dot = %f.\n",
         velocities->y, eulers->phi, rates->p, -position->z, -velocities->z);
}

void ins_flow_update(void)
{
  float mass = parameters[PAR_MASS]; // 0.400;
  float moment = 0.0f; // for now assumed to be 0
  float Ix = parameters[PAR_IX]; // 0.0018244;
  //float b = parameters[PAR_BASE];
  float g = 9.81; // TODO: get a more accurate definition from pprz
  float kd = parameters[PAR_KD]; // 0.5
  float drag = 0.0f;

  if (reset_filter) {
    ins_reset_filter();
    reset_filter = false;
  }

  // get ground truth data:
  //struct FloatEulers* eulers = stateGetNedToBodyEulers_f();
  //struct NedCoor_f* position = stateGetPositionNed_f();
  //struct NedCoor_f *velocities = stateGetSpeedNed_f();
  //struct FloatRates *rates = stateGetBodyRates_f();

  // TODO: record when starting from the ground: does that screw up the filter? Yes it does : )


  // assuming that the typical case is no rotation, we can estimate the (initial) bias of the gyro:
  ins_flow.lp_gyro_bias_roll = lp_factor_strong * ins_flow.lp_gyro_bias_roll + (1 - lp_factor_strong) *
                               ins_flow.lp_gyro_roll;
  if (OF_TWO_DIM) {
    ins_flow.lp_gyro_bias_pitch = lp_factor_strong * ins_flow.lp_gyro_bias_pitch + (1 - lp_factor_strong) *
                                  ins_flow.lp_gyro_pitch;
  }
  // same assumption for the roll command: assuming a close-to-hover situation and roll trim for staying in place:
  ins_flow.lp_roll_command = lp_factor_strong * ins_flow.lp_roll_command + (1 - lp_factor_strong) *
                             stabilization_cmd[COMMAND_ROLL];

  // only start estimation when flying (and above 1 meter: || position->z > -1.0f )
  // I removed the condition on height, since (1) we need to start the filter anyway explicitly now, and (2) it created a dependence on GPS fix.
  if (!autopilot_in_flight()) {
    return;
  }

  if (!run_filter) {

    // TODO: should we do this if we have a thrust bias?

    // Drone is flying but not yet running the filter, in order to obtain unbiased thrust estimates we estimate an additional factor.
    // Assumption is that the drone is hovering, which means that over a longer period of time, the thrust should equal gravity.
    // If the low pass thrust is lower than the one expected by gravity, then it needs to be increased and viceversa.
    float thrust = 0.0f;
    for (int i = 0; i < OF_N_ROTORS; i++) {
      thrust += RPM_FACTORS[i] * ins_flow.RPM[i] * ins_flow.RPM[i];
    }
    if (ins_flow.lp_thrust < 1E-3) {
      // first time directly initialize with thrust to get quicker convergence:
      ins_flow.lp_thrust = thrust;
    } else {
      ins_flow.lp_thrust = lp_factor_strong * ins_flow.lp_thrust + (1 - lp_factor_strong) * thrust;
    }
    float actual_lp_thrust = mass * g;
    ins_flow.thrust_factor = actual_lp_thrust / ins_flow.lp_thrust;
    thrust_factor = ins_flow.thrust_factor;
    //printf("Low pass predicted thrust = %f. Expected thrust = %f. Thrust factor = %f.\n", ins_flow.lp_thrust, actual_lp_thrust, ins_flow.thrust_factor);
    // don't run the filter just yet:
    return;
  }

  if (DEBUG_INS_FLOW) { print_true_state(); }

  // in the sim, the gyro bias wanders so fast, that this does not seem to be useful:
  // TODO: verify how this is in reality, and if not useful, remove all code to estimate this bias (or do it differently)
  // ins_flow.lp_gyro_bias_roll = 0.0f;

  // This module needs to run at the autopilot speed when not yet using this filter. Plus it needs to estimate thrust and gyro bias at that speed.
  // However, the updates of the filter themselves should be slower:
  /*counter++;
  if(counter < 5) {
      return;
  }
  else {
      counter = 0;
  }*/

  // get the new time:
  of_time = get_sys_time_float();
  float dt = of_time - of_prev_time;
  //printf("dt = %f.\n", dt);
  if (dt > 1.0f) {
    dt = 0.01f;
  }

  // predict the thrust and moment:
  float thrust = 0.0f;
  for (int i = 0; i < OF_N_ROTORS; i++) {
    thrust += RPM_FACTORS[i] * ins_flow.RPM[i] * ins_flow.RPM[i];
  }
  thrust *= thrust_factor; // ins_flow.thrust_factor;
  if (OF_THRUST_BIAS && !CONSTANT_ALT_FILTER) {
    thrust -= OF_X[OF_THRUST_BIAS_IND];
  }
  DEBUG_PRINT("Thrust acceleration = %f, g = %f\n", thrust / mass, g);

  // TODO: do we have an optimization that used the moment?
  /*moment = b * RPM_FACTORS[0] * ins_flow.RPM[0]*ins_flow.RPM[0] -
           b * RPM_FACTORS[1] * ins_flow.RPM[1]*ins_flow.RPM[1] -
     b * RPM_FACTORS[2] * ins_flow.RPM[2]*ins_flow.RPM[2] +
     b * RPM_FACTORS[3] * ins_flow.RPM[3]*ins_flow.RPM[3];*/
  // M_est = Ix * (-0.000553060716181365 * cmd_roll(k) -3.23315441805895 * Xe(3, k));
#if USE_NPS
  // TODO: moment in simulation is very easy to estimate with the roll command, so add that:
  moment = 0;
#else

  /*
  moments[moment_ind] = Ix *(-0.000553060716181365 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command) -3.23315441805895 * OF_X[OF_ANGLE_DOT_IND]);

  int select_ind = moment_ind - MOMENT_DELAY;
  if(select_ind < 0) {
  select_ind += MOMENT_DELAY;
  }

  // current moment is a delayed version:
  moment = moments[select_ind];

  // update the moment's ind:
  moment_ind++;
  if(moment_ind >= MOMENT_DELAY) {
  moment_ind = 0;
  }
  */
  // moment = Ix *(-0.000553060716181365 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command) -3.23315441805895 * OF_X[OF_ANGLE_DOT_IND]);
  moment = 0;
#endif

  // printf("Predicted moment = %f, gyro = %f\n", moment, dt * (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f);


  // propagate the state with Euler integration:
  DEBUG_PRINT("Before prediction: ");
  if (DEBUG_INS_FLOW) { print_ins_flow_state(); }
  if (CONSTANT_ALT_FILTER) {
    OF_X[OF_V_IND] += dt * (g * tan(OF_X[OF_ANGLE_IND]));
    if (OF_DRAG) {
      // quadratic drag acceleration:
      drag = dt * kd * (OF_X[OF_V_IND] * OF_X[OF_V_IND]) / mass;
      // apply it in the right direction:
      if (OF_X[OF_V_IND] > 0) { OF_X[OF_V_IND] -= drag; }
      else { OF_X[OF_V_IND] += drag; }
    }

    /* // if we use gyros here, the angle dot estimate is ignored:
     * if(OF_USE_GYROS) {
    // OF_X[OF_ANGLE_IND] += dt * (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f; // Code says scaled by 12, but... that does not fit...
    } */

    // temporary insertion of gyro estimate here, for quicker effect:
    // OF_X[OF_ANGLE_IND] += dt * -2.0e-03 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);

    OF_X[OF_ANGLE_IND] += dt * OF_X[OF_ANGLE_DOT_IND];
    OF_X[OF_ANGLE_DOT_IND] += dt * (moment / Ix);

    if (OF_TWO_DIM) {
      // Second axis, decoupled formulation:
      OF_X[OF_VX_IND] += dt * (g * tan(OF_X[OF_THETA_IND]));
      if (OF_DRAG) {
        // quadratic drag acceleration:
        drag = dt * kd * (OF_X[OF_VX_IND] * OF_X[OF_VX_IND]) / mass;
        // apply it in the right direction:
        if (OF_X[OF_VX_IND] > 0) { OF_X[OF_VX_IND] -= drag; }
        else { OF_X[OF_VX_IND] += drag; }
      }
      // TODO: here also a moment estimate?
      // TODO: add a THETA_DOT_IND
      OF_X[OF_THETA_IND] += dt * (ins_flow.lp_gyro_pitch - ins_flow.lp_gyro_bias_pitch) *
                            (M_PI / 180.0f) / 74.0f; // Code says scaled by 12, but... that does not fit...
    }

    DEBUG_PRINT("Rate p = %f, gyro p = %f\n", rates->p,
                (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI / 180.0f) / 74.0f);
  } else {
    // make sure that the right hand state terms appear before they change:
    OF_X[OF_V_IND] += dt * (thrust * sin(OF_X[OF_ANGLE_IND]) / mass);
    OF_X[OF_Z_IND] += dt * OF_X[OF_Z_DOT_IND];
    OF_X[OF_Z_DOT_IND] += dt * (thrust * cos(OF_X[OF_ANGLE_IND]) / mass - g);
    OF_X[OF_ANGLE_IND] += dt * OF_X[OF_ANGLE_DOT_IND];
    /*
     * // TODO: We now only keep this here because it worked on the real drone. It also worked without it. So to be deleted if it works as is.
    else {
    OF_X[OF_ANGLE_IND] += dt * (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f;
    }*/
    OF_X[OF_ANGLE_DOT_IND] += dt * (moment / Ix);

    // thrust bias does not change over time according to our model

    if (OF_DRAG) {
      // quadratic drag acceleration:
      drag = dt * kd * (OF_X[OF_V_IND] * OF_X[OF_V_IND]) / mass;
      // apply it in the right direction:
      if (OF_X[OF_V_IND] > 0) { OF_X[OF_V_IND] -= drag; }
      else { OF_X[OF_V_IND] += drag; }
    }
  }

  // ensure that z is not 0 (or lower)
  if (OF_X[OF_Z_IND] < 1e-2) {
    OF_X[OF_Z_IND] = 1e-2;
  }

  DEBUG_PRINT("After prediction: ");
  if (DEBUG_INS_FLOW) { print_ins_flow_state(); }

  // prepare the update and correction step:
  // we have to recompute these all the time, as they depend on the state:
  // discrete version of state transition matrix F: (ignoring t^2)
  float F[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
  for (int i = 0; i < N_STATES_OF_KF; i++) {
    F[i][i] = 1.0f;
  }
  if (CONSTANT_ALT_FILTER) {
    F[OF_V_IND][OF_ANGLE_IND] = dt * (g / (cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND])));
    F[OF_ANGLE_IND][OF_ANGLE_DOT_IND] = dt * 1.0f;
    if (OF_TWO_DIM) {
      F[OF_VX_IND][OF_THETA_IND] = dt * (g / (cos(OF_X[OF_THETA_IND]) * cos(OF_X[OF_THETA_IND])));
    }
  } else {
    F[OF_V_IND][OF_ANGLE_IND] = dt * (thrust * cos(OF_X[OF_ANGLE_IND]) / mass);
    F[OF_ANGLE_IND][OF_ANGLE_DOT_IND] = dt * 1.0f;
    F[OF_Z_IND][OF_Z_DOT_IND] = dt * 1.0f;
    F[OF_Z_DOT_IND][OF_ANGLE_IND] = dt * (-thrust * sin(OF_X[OF_ANGLE_IND]) / mass);
    if (OF_THRUST_BIAS) {
      F[OF_V_IND][OF_THRUST_BIAS_IND] =  -dt * sin(OF_X[OF_ANGLE_IND]) / mass;
      F[OF_Z_DOT_IND][OF_THRUST_BIAS_IND] = -dt * cos(OF_X[OF_ANGLE_IND]) / mass;
    }
  }
  if (OF_DRAG) {
    // In MATLAB: -sign(v)*2*kd*v/m (always minus, whether v is positive or negative):
    F[OF_V_IND][OF_V_IND] -=  dt * 2 * kd * abs(OF_X[OF_V_IND]) / mass;
    if (OF_TWO_DIM) {
      F[OF_VX_IND][OF_VX_IND] -=  dt * 2 * kd * abs(OF_X[OF_VX_IND]) / mass;
    }
  }

  // G matrix (whatever it may be):
  float G[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
  // TODO: we miss an off-diagonal element here (compare with MATLAB)
  for (int i = 0; i < N_STATES_OF_KF; i++) {
    G[i][i] = dt;
  }

  // Jacobian observation matrix H:
  float H[N_MEAS_OF_KF][N_STATES_OF_KF] = {{0.}};

  if (CONSTANT_ALT_FILTER) {
    // Hx = [-cos(theta)^2/z, (v*sin(theta))/ z, (v* cos(theta)^2)/z^2];
    // lateral flow:
    H[OF_LAT_FLOW_IND][OF_V_IND] = -cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];
    H[OF_LAT_FLOW_IND][OF_ANGLE_IND] = OF_X[OF_V_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];
    H[OF_LAT_FLOW_IND][OF_Z_IND] = OF_X[OF_V_IND] * cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) /
                                   (OF_X[OF_Z_IND] * OF_X[OF_Z_IND]);
    H[OF_LAT_FLOW_IND][OF_ANGLE_DOT_IND] = 1.0f;
    // divergence:
    H[OF_DIV_FLOW_IND][OF_V_IND] = -sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND]);
    H[OF_DIV_FLOW_IND][OF_ANGLE_IND] = -OF_X[OF_V_IND] * cos(2 * OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];
    H[OF_DIV_FLOW_IND][OF_Z_IND] = OF_X[OF_V_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND] * OF_X[OF_Z_IND]);
    H[OF_DIV_FLOW_IND][OF_ANGLE_DOT_IND] = 0.0f;

    if (OF_TWO_DIM) {
      // divergence measurement couples the two axes actually...:
      H[OF_DIV_FLOW_IND][OF_VX_IND] = -sin(2 * OF_X[OF_THETA_IND]) / (2 * OF_X[OF_Z_IND]);
      H[OF_DIV_FLOW_IND][OF_THETA_IND] = -OF_X[OF_VX_IND] * cos(2 * OF_X[OF_THETA_IND]) / OF_X[OF_Z_IND];

      // lateral flow in x direction:
      H[OF_LAT_FLOW_X_IND][OF_VX_IND] = -cos(OF_X[OF_THETA_IND]) * cos(OF_X[OF_THETA_IND]) / OF_X[OF_Z_IND];
      H[OF_LAT_FLOW_X_IND][OF_THETA_IND] = OF_X[OF_VX_IND] * sin(2 * OF_X[OF_THETA_IND]) / OF_X[OF_Z_IND];
      H[OF_LAT_FLOW_X_IND][OF_Z_IND] = OF_X[OF_VX_IND] * cos(OF_X[OF_THETA_IND]) * cos(OF_X[OF_THETA_IND]) /
                                       (OF_X[OF_Z_IND] * OF_X[OF_Z_IND]);
    }
  } else {
    // lateral flow:
    H[OF_LAT_FLOW_IND][OF_V_IND] = -cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];
    H[OF_LAT_FLOW_IND][OF_ANGLE_IND] = OF_X[OF_V_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND]
                                       + OF_X[OF_Z_DOT_IND] * cos(2 * OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];
    H[OF_LAT_FLOW_IND][OF_ANGLE_DOT_IND] = 1.0f;
    H[OF_LAT_FLOW_IND][OF_Z_IND] = OF_X[OF_V_IND] * cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) /
                                   (OF_X[OF_Z_IND] * OF_X[OF_Z_IND])
                                   - OF_X[OF_Z_DOT_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND] * OF_X[OF_Z_IND]);
    H[OF_LAT_FLOW_IND][OF_Z_DOT_IND] = sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND]);
    // divergence:
    H[OF_DIV_FLOW_IND][OF_V_IND] = -sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND]);
    H[OF_DIV_FLOW_IND][OF_ANGLE_IND] = -OF_X[OF_V_IND] * cos(2 * OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND]
                                       + OF_X[OF_Z_DOT_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];
    H[OF_DIV_FLOW_IND][OF_ANGLE_DOT_IND] = 0.0f;
    H[OF_DIV_FLOW_IND][OF_Z_IND] = OF_X[OF_V_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND] * OF_X[OF_Z_IND])
                                   + OF_X[OF_Z_DOT_IND] * cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) / (OF_X[OF_Z_IND] * OF_X[OF_Z_IND]);
    H[OF_DIV_FLOW_IND][OF_Z_DOT_IND] = -cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];
  }

  // rate measurement:
  if (OF_USE_GYROS) {
    H[OF_RATE_IND][OF_V_IND] = 0.0f;
    H[OF_RATE_IND][OF_ANGLE_IND] = 0.0f;
    H[OF_RATE_IND][OF_ANGLE_DOT_IND] = 1.0f;
    H[OF_RATE_IND][OF_Z_IND] = 0.0f;
    H[OF_RATE_IND][OF_Z_DOT_IND] = 0.0f;
  }

  // propagate uncertainty:
  // TODO: make pointers that don't change to init:
  MAKE_MATRIX_PTR(Phi, F, N_STATES_OF_KF);
  MAKE_MATRIX_PTR(P, OF_P, N_STATES_OF_KF);
  MAKE_MATRIX_PTR(Gamma, G, N_STATES_OF_KF);
  MAKE_MATRIX_PTR(Q, OF_Q, N_STATES_OF_KF);
  MAKE_MATRIX_PTR(R, OF_R, N_MEAS_OF_KF);
  MAKE_MATRIX_PTR(Jac, H, N_MEAS_OF_KF);

  DEBUG_PRINT("Phi:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, Phi);

  DEBUG_PRINT("P:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

  DEBUG_PRINT("Gamma:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, G);

  DEBUG_PRINT("Q:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, Q);

  DEBUG_PRINT("R:\n");
  DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, R);

  DEBUG_PRINT("Jacobian:\n");
  DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_STATES_OF_KF, Jac);

  // Corresponding MATLAB statement:    :O
  // P_k1_k = Phi_k1_k*P*Phi_k1_k' + Gamma_k1_k*Q*Gamma_k1_k';
  float _PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PhiT, _PhiT, N_STATES_OF_KF);
  float _P_PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PPhiT, _P_PhiT, N_STATES_OF_KF);
  float _Phi_P_PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PhiPPhiT, _Phi_P_PhiT, N_STATES_OF_KF);

  float_mat_transpose(PhiT, Phi, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(PPhiT, P, PhiT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(PhiPPhiT, Phi, PPhiT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);

  DEBUG_PRINT("Phi*P*PhiT:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, PhiPPhiT);

  float _GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(GT, _GT, N_STATES_OF_KF);
  float _Q_GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(QGT, _Q_GT, N_STATES_OF_KF);
  float _G_Q_GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(GQGT, _G_Q_GT, N_STATES_OF_KF);

  float_mat_transpose(GT, Gamma, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(QGT, Q, GT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(GQGT, Gamma, QGT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);

  DEBUG_PRINT("Gamma*Q*GammaT:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, GQGT);

  float_mat_sum(P, PhiPPhiT, GQGT, N_STATES_OF_KF, N_STATES_OF_KF);
  DEBUG_PRINT("P:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

  // correct state when there is a new vision measurement:
  if (ins_flow.new_flow_measurement) {

    DEBUG_PRINT("*********************\n");
    DEBUG_PRINT("   NEW MEASUREMENT   \n");
    DEBUG_PRINT("*********************\n");

    // determine Kalman gain:
    // MATLAB statement:
    // S_k = Hx*P_k1_k*Hx' + R;
    float _JacT[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(JacT, _JacT, N_STATES_OF_KF);
    float _P_JacT[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(PJacT, _P_JacT, N_STATES_OF_KF);
    float _Jac_P_JacT[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(JacPJacT, _Jac_P_JacT, N_MEAS_OF_KF);

    float_mat_transpose(JacT, Jac, N_MEAS_OF_KF, N_STATES_OF_KF);
    float_mat_mul(PJacT, P, JacT, N_STATES_OF_KF, N_STATES_OF_KF, N_MEAS_OF_KF);
    DEBUG_PRINT("P*JacT:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_MEAS_OF_KF, PJacT);

    float_mat_mul(JacPJacT, Jac, PJacT, N_MEAS_OF_KF, N_STATES_OF_KF, N_MEAS_OF_KF);

    DEBUG_PRINT("Jac*P*JacT:\n");
    DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, JacPJacT);

    float _S[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(S, _S, N_MEAS_OF_KF);
    float_mat_sum(S, JacPJacT, R, N_MEAS_OF_KF, N_MEAS_OF_KF);

    DEBUG_PRINT("S:\n");
    DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, S);

    // MATLAB statement:
    // K_k1 = P_k1_k*Hx' * inv(S_k);
    float _K[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(K, _K, N_STATES_OF_KF);
    float _INVS[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(INVS, _INVS, N_MEAS_OF_KF);
    float_mat_invert(INVS, S, N_MEAS_OF_KF);
    if (DEBUG_INS_FLOW) {
      // This should be the identity matrix:
      float _SINVS[N_MEAS_OF_KF][N_MEAS_OF_KF];
      MAKE_MATRIX_PTR(SINVS, _SINVS, N_MEAS_OF_KF);
      float_mat_mul(SINVS, S, INVS, N_MEAS_OF_KF, N_MEAS_OF_KF, N_MEAS_OF_KF);
      DEBUG_PRINT("S*Inv(S):\n");
      DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, SINVS);
    }

    float_mat_mul(K, PJacT, INVS, N_STATES_OF_KF, N_MEAS_OF_KF, N_MEAS_OF_KF);
    DEBUG_PRINT("K:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_MEAS_OF_KF, K);

    // Correct the state:
    // MATLAB:
    // Z_expected = [-v*cos(theta)*cos(theta)/z + zd*sin(2*theta)/(2*z) + thetad;
    //      (-v*sin(2*theta)/(2*z)) - zd*cos(theta)*cos(theta)/z];
    float Z_expected[N_MEAS_OF_KF];

    // TODO: take this var out? It was meant for debugging...
    //float Z_expect_GT_angle;

    if (CONSTANT_ALT_FILTER) {
      Z_expected[OF_LAT_FLOW_IND] = -OF_X[OF_V_IND] * cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND]
                                    + OF_X[OF_ANGLE_DOT_IND]; // TODO: Currently, no p works better than using p here. Analyze!

      /* TODO: remove later, just for debugging:
      float Z_exp_no_rate = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];
      float Z_exp_with_rate = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]+OF_X[OF_ANGLE_DOT_IND];
      printf("Z_exp_no_rate = %f, Z_exp_with_rate = %f, measured = %f, angle dot = %f, p = %f: ", Z_exp_no_rate, Z_exp_with_rate,
       ins_flow.optical_flow_x, OF_X[OF_ANGLE_DOT_IND], dt * (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f);
      if(fabs(ins_flow.optical_flow_x - Z_exp_no_rate) < fabs(ins_flow.optical_flow_x - Z_exp_with_rate)) {
      printf("NO RATE WINS!");
      }
      printf("\n");*/

      Z_expected[OF_DIV_FLOW_IND] = -OF_X[OF_V_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND]);

      if (OF_TWO_DIM) {
        Z_expected[OF_LAT_FLOW_X_IND] = -OF_X[OF_VX_IND] * cos(OF_X[OF_THETA_IND]) * cos(OF_X[OF_THETA_IND]) /
                                        OF_X[OF_Z_IND]; // TODO: no q?
      }

      //Z_expect_GT_angle = -OF_X[OF_V_IND]*cos(eulers->phi)*cos(eulers->phi)/OF_X[OF_Z_IND];

      if (OF_USE_GYROS) {
        Z_expected[OF_RATE_IND] = OF_X[OF_ANGLE_DOT_IND]; // TODO: is this even in the right direction?
      }
    } else {
      Z_expected[OF_LAT_FLOW_IND] = -OF_X[OF_V_IND] * cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND]
                                    + OF_X[OF_Z_DOT_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND])
                                    + OF_X[OF_ANGLE_DOT_IND]; // TODO: We first had this rate term but not for the constant alt filter.
      // Simulation and data analysis from real flights shows that including it is better. CHECK IN REALITY!

      Z_expected[OF_DIV_FLOW_IND] = -OF_X[OF_V_IND] * sin(2 * OF_X[OF_ANGLE_IND]) / (2 * OF_X[OF_Z_IND])
                                    - OF_X[OF_Z_DOT_IND] * cos(OF_X[OF_ANGLE_IND]) * cos(OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];

      //Z_expect_GT_angle = -OF_X[OF_V_IND]*cos(eulers->phi)*cos(eulers->phi)/OF_X[OF_Z_IND]
      //             + OF_X[OF_Z_DOT_IND]*sin(2*eulers->phi)/(2*OF_X[OF_Z_IND]);
      //+ OF_X[OF_ANGLE_DOT_IND];
      if (N_MEAS_OF_KF == 3) {
        Z_expected[OF_RATE_IND] = OF_X[OF_ANGLE_DOT_IND]; // TODO: is this even in the right direction?
      }

      /*
            float Z_exp_no_rate = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
                + OF_X[OF_Z_DOT_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND]);
            float Z_exp_with_rate = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
                  + OF_X[OF_Z_DOT_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND])
                  + OF_X[OF_ANGLE_DOT_IND];
      */
      /*
      printf("Z_exp_no_rate = %f, Z_exp_with_rate = %f, measured = %f, angle dot = %f, p = %f: ", Z_exp_no_rate, Z_exp_with_rate,
       ins_flow.optical_flow_x, OF_X[OF_ANGLE_DOT_IND], dt * (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f);
      if(fabs(ins_flow.optical_flow_x - Z_exp_no_rate) < fabs(ins_flow.optical_flow_x - Z_exp_with_rate)) {
      printf("NO RATE WINS!");
      }
      printf("\n");*/
    }

    //  i_k1 = Z - Z_expected;
    float innovation[N_MEAS_OF_KF][1];
    //print_ins_flow_state();
    innovation[OF_LAT_FLOW_IND][0] = ins_flow.optical_flow_x - Z_expected[OF_LAT_FLOW_IND];
    DEBUG_PRINT("Expected flow filter: %f, Expected flow ground truth = %f, Real flow x: %f, Real flow y: %f.\n",
                Z_expected[OF_LAT_FLOW_IND], Z_expect_GT_angle, ins_flow.optical_flow_x, ins_flow.optical_flow_y);
    innovation[OF_DIV_FLOW_IND][0] = ins_flow.divergence - Z_expected[OF_DIV_FLOW_IND];
    DEBUG_PRINT("Expected div: %f, Real div: %f.\n", Z_expected[OF_DIV_FLOW_IND], ins_flow.divergence);
    if (CONSTANT_ALT_FILTER && OF_TWO_DIM) {
      innovation[OF_LAT_FLOW_X_IND][0] = ins_flow.optical_flow_y - Z_expected[OF_LAT_FLOW_X_IND];
      DEBUG_PRINT("Expected flow in body X direction filter: %f, Real flow in corresponding y direction: %f, gyro = %f, expected velocity = %f, real velocity = %f, expected theta = %f, real theta = %f.\n",
                  Z_expected[OF_LAT_FLOW_X_IND], ins_flow.optical_flow_y, ins_flow.lp_gyro_pitch - ins_flow.lp_gyro_bias_pitch,
                  OF_X[OF_VX_IND], velocities->x, OF_X[OF_THETA_IND], eulers->theta);
    }
    if (OF_USE_GYROS) {
      float gyro_meas_roll;
      if (!PREDICT_GYROS) {
        gyro_meas_roll = (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI / 180.0f) / 74.0f;
      } else {
        // TODO: You can fake gyros here by estimating them as follows:
        // rate_p_filt_est = -1.8457e-04 * cmd_roll;
        // gyro_meas_roll = -1.8457e-04 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);
        // gyro_meas_roll = -2.0e-03 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);

        // gyro_meas_roll = 1e-04 * parameters[PAR_PRED_ROLL_1] * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);
        // gyro_meas_roll = parameters[PAR_PRED_ROLL_2] * gyro_meas_roll + 1E-3 * parameters[PAR_PRED_ROLL_3] * ins_flow.optical_flow_x;

        // only flow:
        gyro_meas_roll = 2E-3 * parameters[PAR_PRED_ROLL_3] * ins_flow.optical_flow_x;

        //printf("Predicted roll: %f, real measured roll: %f.\n", gyro_meas_roll, (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f);
      }

      innovation[OF_RATE_IND][0] = gyro_meas_roll - Z_expected[OF_RATE_IND];
      //innovation[OF_RATE_IND][0] = rates->p - Z_expected[OF_RATE_IND];
      DEBUG_PRINT("Expected rate: %f, Real rate: %f.\n", Z_expected[OF_RATE_IND], ins_flow.lp_gyro_roll);
    }

    MAKE_MATRIX_PTR(I, innovation, N_MEAS_OF_KF);
    DEBUG_PRINT("Innovation:");
    DEBUG_MAT_PRINT(N_MEAS_OF_KF, 1, I);

    // X_k1_k1 = X_k1_k + K_k1*(i_k1);
    float _KI[N_STATES_OF_KF][1];
    MAKE_MATRIX_PTR(KI, _KI, N_STATES_OF_KF);
    float_mat_mul(KI, K, I, N_STATES_OF_KF, N_MEAS_OF_KF, 1);

    DEBUG_PRINT("K*innovation:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, 1, KI);

    DEBUG_PRINT("PRE: v = %f, angle = %f\n", OF_X[OF_V_IND], OF_X[OF_ANGLE_IND]);
    for (int i = 0; i < N_STATES_OF_KF; i++) {
      OF_X[i] += KI[i][0];
    }
    DEBUG_PRINT("POST v: %f, angle = %f\n", OF_X[OF_V_IND], OF_X[OF_ANGLE_IND]);

    DEBUG_PRINT("Angles (deg): ahrs = %f, ekf = %f.\n", (180.0f / M_PI)*eulers->phi, (180.0f / M_PI)*OF_X[OF_ANGLE_IND]);

    DEBUG_PRINT("P before correction:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

    // P_k1_k1 = (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)' + K_k1*R*K_k1'; % Joseph form of the covariance update equation
    float _KJac[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KJac, _KJac, N_STATES_OF_KF);
    float_mat_mul(KJac, K, Jac, N_STATES_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);

    float _eye[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(eye, _eye, N_STATES_OF_KF);
    float_mat_diagonal_scal(eye, 1.0, N_STATES_OF_KF);
    DEBUG_PRINT("eye:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, eye);

    float _eKJac[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(eKJac, _eKJac, N_STATES_OF_KF);
    float_mat_diff(eKJac, eye, KJac, N_STATES_OF_KF, N_STATES_OF_KF);
    DEBUG_PRINT("eKJac:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, eKJac);

    float _eKJacT[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(eKJacT, _eKJacT, N_STATES_OF_KF);
    float_mat_transpose(eKJacT, eKJac, N_STATES_OF_KF, N_STATES_OF_KF);
    // (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)'
    float _P_pre[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(P_pre, _P_pre, N_STATES_OF_KF);
    float_mat_mul(P_pre, P, eKJacT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
    float_mat_mul(P, eKJac, P_pre, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
    DEBUG_PRINT("eKJac * P *eKJacT:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

    // K_k1*R*K_k1'
    // TODO: check all MAKE_MATRIX that they mention the number of ROWS!
    float _KT[N_MEAS_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KT, _KT, N_MEAS_OF_KF);
    float_mat_transpose(KT, K, N_STATES_OF_KF, N_MEAS_OF_KF);
    float _RKT[N_MEAS_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(RKT, _RKT, N_MEAS_OF_KF);
    float_mat_mul(RKT, R, KT, N_MEAS_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);
    float _KRKT[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KRKT, _KRKT, N_STATES_OF_KF);
    float_mat_mul(KRKT, K, RKT, N_STATES_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);
    DEBUG_PRINT("KRKT:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, KRKT);

    // summing the two parts:
    float_mat_sum(P, P, KRKT, N_STATES_OF_KF, N_STATES_OF_KF);

    DEBUG_PRINT("P corrected:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);
    float trace_P = 0.0f;
    for (int i = 0; i < N_STATES_OF_KF; i++) {
      trace_P += P[i][i];
    }
    DEBUG_PRINT("trace P = %f\n", trace_P);

    // indicate that the measurement has been used:
    ins_flow.new_flow_measurement = false;
  }

  // update the time:
  of_prev_time = of_time;

}

static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
  ahrs_icq_last_stamp = stamp;
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS_ICQ propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_propagate(gyro, dt);
    set_body_state_from_quat();

    // TODO: filter all gyro values
    // For now only filter the roll gyro:
    float current_rate = ((float)gyro->p); // TODO: is this correct? / INT32_RATE_FRAC;
    ins_flow.lp_gyro_roll = lp_factor * ins_flow.lp_gyro_roll + (1 - lp_factor) * current_rate;
    current_rate = ((float)gyro->q);
    ins_flow.lp_gyro_pitch = lp_factor * ins_flow.lp_gyro_pitch + (1 - lp_factor) * current_rate;
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS_ICQ propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_icq.status == AHRS_ICQ_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_icq_propagate(gyro, dt);
    set_body_state_from_quat();
  }
#endif
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id,
                     uint32_t __attribute__((unused)) stamp,
                     struct Int32Vect3 *accel)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_quat accel update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_update_accel(accel, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS int_cmpl_quat accel update.")
  PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  if (ahrs_icq.is_aligned) {
    const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
    ahrs_icq_update_accel(accel, dt);
    set_body_state_from_quat();
  }
#endif
}

/*
static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t __attribute__((unused)) stamp,
                   struct Int32Vect3 *mag)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_MAG_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_quat mag update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    //ahrs_icq_update_mag(mag, dt);
    //set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_MAG_CORRECT_FREQUENCY for AHRS int_cmpl_quat mag update.")
  PRINT_CONFIG_VAR(AHRS_MAG_CORRECT_FREQUENCY)
  if (ahrs_icq.is_aligned) {
    const float dt = 1. / (AHRS_MAG_CORRECT_FREQUENCY);
    ahrs_icq_update_mag(mag, dt);
    set_body_state_from_quat();
  }
#endif
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  //VECT3_ASSIGN(ahrs_icq.mag_h, MAG_BFP_OF_REAL(h->x), MAG_BFP_OF_REAL(h->y),
  //             MAG_BFP_OF_REAL(h->z));
}
*/

/** Rotate angles and rates from imu to body frame and set state */
static void set_body_state_from_quat(void)
{
  /* Compute LTP to BODY quaternion */
  struct Int32Quat ltp_to_body_quat = ahrs_icq.ltp_to_body_quat;
  //struct Int32Quat *body_to_imu_quat = orientationGetQuat_i(&ahrs_icq.body_to_imu);
  //int32_quat_comp_inv(&ltp_to_body_quat, &ahrs_icq.ltp_to_imu_quat, body_to_imu_quat);

  if (use_filter < USE_ANGLE) {
    // Use the orientation as is:
    stateSetNedToBodyQuat_i(&ltp_to_body_quat);
  } else {

    // get Euler angles:
    struct OrientationReps orient;
    orient.status = 1 << ORREP_QUAT_I;
    orient.quat_i = ltp_to_body_quat;
    struct FloatEulers *eulers = orientationGetEulers_f(&orient);

    // set roll angle with value from the filter:
    GT_phi = eulers->phi;
    eulers->phi = OF_X[OF_ANGLE_IND];
    if (OF_TWO_DIM) {
      GT_theta = eulers->theta;
      //printf("Real theta = %f, setting theta to %f.\n", eulers->theta, OF_X[OF_THETA_IND]);
      eulers->theta = OF_X[OF_THETA_IND];
    }

    // Transform the Euler representation to Int32Quat and set the state:
    struct OrientationReps orient_euler;
    orient_euler.status = 1 << ORREP_EULER_F;
    orient_euler.eulers_f = (*eulers);

    struct Int32Quat *quat_i_adapted = orientationGetQuat_i(&orient_euler);
    stateSetNedToBodyQuat_i(quat_i_adapted);
  }

  /* compute body rates */
  struct Int32Rates body_rate = ahrs_icq.body_rate;
  // struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&ahrs_icq.body_to_imu);
  // int32_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_icq.imu_rate);
  /* Set state */
  stateSetBodyRates_i(&body_rate);

}

static void ins_rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t num_act)
{
  ins_flow.RPM_num_act = num_act;
  for (int i = 0; i < num_act; i++) {
    ins_flow.RPM[i] = rpm[i];
  }
}


/* Update INS based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{

  if (gps_s->fix < GPS_FIX_3D) {
    return;
  }
  if (!ins_flow.ltp_initialized) {
    ins_reset_local_origin();
  }

  ahrs_icq_update_gps(gps_s);

  /* simply scale and copy pos/speed from gps */
  struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_flow.ltp_def, &gps_s->ecef_pos);
  INT32_VECT3_SCALE_2(ins_flow.ltp_pos, gps_pos_cm_ned,
                      INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);

  if (use_filter >= USE_HEIGHT) {
    //struct NedCoor_f* position = stateGetPositionNed_f();
    int32_t z_Ned_i_filter = - (int32_t)((OF_X[OF_Z_IND] * INT32_POS_OF_CM_NUM * 100) / INT32_POS_OF_CM_DEN);
    //printf("Z true: %f / %d, Z filter: %f / %d. (float / int32_t)\n", position->z, ins_flow.ltp_pos.z, OF_X[OF_Z_IND], z_Ned_i);
    ins_flow.ltp_pos.z = z_Ned_i_filter;
  }

  stateSetPositionNed_i(&ins_flow.ltp_pos);



  struct NedCoor_i gps_speed_cm_s_ned;
  ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_flow.ltp_def, &gps_s->ecef_vel);
  INT32_VECT3_SCALE_2(ins_flow.ltp_speed, gps_speed_cm_s_ned,
                      INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);

  if (use_filter >= USE_VELOCITY) {
    // get NED to body rotation matrix:
    struct FloatRMat *NTB = stateGetNedToBodyRMat_f();
    // get transpose (inverse):
    struct FloatRMat BTN;
    float_rmat_inv(&BTN, NTB);

    // the velocities from the filter are rotated from the body to the inertial frame:
    struct FloatVect3 NED_velocities, body_velocities;
    body_velocities.x = 0.0f; // filter does not determine this yet
    body_velocities.y = OF_X[OF_V_IND];
    body_velocities.z = 0.0f;
    /*
    if(CONSTANT_ALT_FILTER) {
    body_velocities.z = 0.0f;
    }
    else {
    body_velocities.z = OF_X[OF_Z_DOT_IND];
    }*/
    float_rmat_vmult(&NED_velocities, &BTN, &body_velocities);
    // TODO: also estimate vx, so that we can just use the rotated vector:
    // For now, we need to keep the x, and y body axes aligned with the global ones.
    // printf("Original speed y = %d, ", ins_flow.ltp_speed.y);
    ins_flow.ltp_speed.y = (int32_t)((NED_velocities.y * INT32_SPEED_OF_CM_S_NUM * 100) / INT32_SPEED_OF_CM_S_DEN);
    if (!CONSTANT_ALT_FILTER) { ins_flow.ltp_speed.z =  -(int32_t)((NED_velocities.z * INT32_SPEED_OF_CM_S_NUM * 100) / INT32_SPEED_OF_CM_S_DEN); }
    // printf("Changed speed y = %d (%f in float)\n", ins_flow.ltp_speed.y, NED_velocities.y);
  }

  stateSetSpeedNed_i(&ins_flow.ltp_speed);

  /*
  bool vel_ned_valid = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT);
  uint8_t nsats = gps_s->num_sv;
  */
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ahrs_icq.is_aligned) {
    if (ahrs_icq_align(lp_gyro, lp_accel, lp_mag)) {
      set_body_state_from_quat();
    }
  }
}


/* Save the Body to IMU information */
//// TODO: Is this code still necessary?
//static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
//                           struct FloatQuat *q_b2i_f)
//{
//  ahrs_icq_set_body_to_imu_quat(q_b2i_f);
//}
