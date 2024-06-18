/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
 * MAVLab Delft University of Technology
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

/** @file stabilization_attitude_quat_indi.c
 * @brief MAVLab Delft University of Technology
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "math/wls/wls_alloc.h"
#include <stdio.h>

// Factor that the estimated G matrix is allowed to deviate from initial one
#define INDI_ALLOWED_G_FACTOR 2.0

#ifdef STABILIZATION_INDI_FILT_CUTOFF_P
#define STABILIZATION_INDI_FILTER_ROLL_RATE TRUE
#else
#define STABILIZATION_INDI_FILT_CUTOFF_P 20.0
#endif

#ifdef STABILIZATION_INDI_FILT_CUTOFF_Q
#define STABILIZATION_INDI_FILTER_PITCH_RATE TRUE
#else
#define STABILIZATION_INDI_FILT_CUTOFF_Q 20.0
#endif

#ifdef STABILIZATION_INDI_FILT_CUTOFF_R
#define STABILIZATION_INDI_FILTER_YAW_RATE TRUE
#else
#define STABILIZATION_INDI_FILT_CUTOFF_R 20.0
#endif

// Default is WLS
#ifndef STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#define STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE FALSE
#endif

#ifndef STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
#define STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER FALSE
#endif

// Airspeed [m/s] at which the forward flight throttle limit is used instead of
// the hover throttle limit.
#ifndef STABILIZATION_INDI_THROTTLE_LIMIT_AIRSPEED_FWD
#define STABILIZATION_INDI_THROTTLE_LIMIT_AIRSPEED_FWD 8.0
#endif

#if INDI_OUTPUTS > 4
#ifndef STABILIZATION_INDI_G1_THRUST_X
#error "You must define STABILIZATION_INDI_G1_THRUST_X for your number of INDI_OUTPUTS"
#endif
#endif

#ifdef SetCommandsFromRC
#warning SetCommandsFromRC not used: STAB_INDI writes actuators directly
#endif

#ifdef SetAutoCommandsFromRC
#warning SetAutoCommandsFromRC not used: STAB_INDI writes actuators directly
#endif


#if !STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#if INDI_NUM_ACT > WLS_N_U
#error Matrix-WLS_N_U too small or not defined: define WLS_N_U >= INDI_NUM_ACT in airframe file
#endif
#if INDI_OUTPUTS > WLS_N_V
#error Matrix-WLS_N_V too small or not defined: define WLS_N_U >= INDI_OUTPUTS in airframe file
#endif
#endif

float u_min_stab_indi[INDI_NUM_ACT];
float u_max_stab_indi[INDI_NUM_ACT];
float u_pref_stab_indi[INDI_NUM_ACT];
float indi_v[INDI_OUTPUTS];
float *Bwls[INDI_OUTPUTS];
int num_iter = 0;

static void lms_estimation(void);
static void get_actuator_state(void);
static void calc_g1_element(float dx_error, int8_t i, int8_t j, float mu_extra);
static void calc_g2_element(float dx_error, int8_t j, float mu_extra);
#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
static void calc_g1g2_pseudo_inv(void);
#endif
static void bound_g_mat(void);

int32_t stabilization_att_indi_cmd[COMMANDS_NB];
struct Indi_gains indi_gains = {
  .att = {
    STABILIZATION_INDI_REF_ERR_P,
    STABILIZATION_INDI_REF_ERR_Q,
    STABILIZATION_INDI_REF_ERR_R
  },
  .rate = {
    STABILIZATION_INDI_REF_RATE_P,
    STABILIZATION_INDI_REF_RATE_Q,
    STABILIZATION_INDI_REF_RATE_R
  },
};

#if STABILIZATION_INDI_USE_ADAPTIVE
bool indi_use_adaptive = true;
#else
bool indi_use_adaptive = false;
#endif

#ifdef STABILIZATION_INDI_ACT_RATE_LIMIT
float act_rate_limit[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_RATE_LIMIT;
#endif

#ifdef STABILIZATION_INDI_ACT_IS_SERVO
bool act_is_servo[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_IS_SERVO;
#else
bool act_is_servo[INDI_NUM_ACT] = {0};
#endif

#ifdef STABILIZATION_INDI_ACT_IS_THRUSTER_X
bool act_is_thruster_x[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_IS_THRUSTER_X;
#else
bool act_is_thruster_x[INDI_NUM_ACT] = {0};
#endif

bool act_is_thruster_z[INDI_NUM_ACT];

#ifdef STABILIZATION_INDI_ACT_PREF
// Preferred (neutral, least energy) actuator value
float act_pref[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_PREF;
#else
// Assume 0 is neutral
float act_pref[INDI_NUM_ACT] = {0.0};
#endif

#ifdef STABILIZATION_INDI_ACT_DYN
#warning STABILIZATION_INDI_ACT_DYN is deprecated, use STABILIZATION_INDI_ACT_FREQ instead.
#warning You now have to define the continuous time corner frequency in rad/s of the actuators.
#warning "Use -ln(1 - old_number) * PERIODIC_FREQUENCY to compute it from the old values."
float act_dyn_discrete[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_DYN;
#else
float act_first_order_cutoff[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_FREQ;
float act_dyn_discrete[INDI_NUM_ACT]; // will be computed from freq at init
#endif

#ifdef STABILIZATION_INDI_WLS_PRIORITIES
static float Wv[INDI_OUTPUTS] = STABILIZATION_INDI_WLS_PRIORITIES;
#else
//State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
#if INDI_OUTPUTS == 5
static float Wv[INDI_OUTPUTS] = {1000, 1000, 1, 100, 100};
#else
static float Wv[INDI_OUTPUTS] = {1000, 1000, 1, 100};
#endif
#endif

/**
 * Weighting of different actuators in the cost function
 */
#ifdef STABILIZATION_INDI_WLS_WU
float indi_Wu[INDI_NUM_ACT] = STABILIZATION_INDI_WLS_WU;
#else
float indi_Wu[INDI_NUM_ACT] = {[0 ... INDI_NUM_ACT - 1] = 1.0};
#endif

/**
 * Limit the maximum specific moment that can be compensated (units rad/s^2)
*/
#ifdef STABILIZATION_INDI_YAW_DISTURBANCE_LIMIT
float stablization_indi_yaw_dist_limit = STABILIZATION_INDI_YAW_DISTURBANCE_LIMIT;
#else // Put a rediculously high limit
float stablization_indi_yaw_dist_limit = 99999.f;
#endif

// variables needed for control
float actuator_state_filt_vect[INDI_NUM_ACT];
struct FloatRates angular_accel_ref = {0., 0., 0.};
struct FloatRates angular_rate_ref = {0., 0., 0.};
float angular_acceleration[3] = {0., 0., 0.};
float actuator_state[INDI_NUM_ACT];
float indi_u[INDI_NUM_ACT];

float q_filt = 0.0;
float r_filt = 0.0;

float stabilization_indi_filter_freq = 20.0; //Hz, for setting handler

// variables needed for estimation
float g1g2_trans_mult[INDI_OUTPUTS][INDI_OUTPUTS];
float g1g2inv[INDI_OUTPUTS][INDI_OUTPUTS];
float actuator_state_filt_vectd[INDI_NUM_ACT];
float actuator_state_filt_vectdd[INDI_NUM_ACT];
float estimation_rate_d[INDI_NUM_ACT];
float estimation_rate_dd[INDI_NUM_ACT];
float du_estimation[INDI_NUM_ACT];
float ddu_estimation[INDI_NUM_ACT];

// The learning rate per axis (roll, pitch, yaw, thrust)
float mu1[INDI_OUTPUTS] = {0.00001, 0.00001, 0.000003, 0.000002};
// The learning rate for the propeller inertia (scaled by 512 wrt mu1)
float mu2 = 0.002;

// other variables
float act_obs[INDI_NUM_ACT];

// Number of actuators used to provide thrust
int32_t num_thrusters;
int32_t num_thrusters_x;

static struct Int32Eulers stab_att_sp_euler;
static struct Int32Quat   stab_att_sp_quat;

// Register actuator feedback if we rely on RPM information
#if STABILIZATION_INDI_RPM_FEEDBACK
#ifndef STABILIZATION_INDI_ACT_FEEDBACK_ID
#define STABILIZATION_INDI_ACT_FEEDBACK_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(STABILIZATION_INDI_ACT_FEEDBACK_ID)

abi_event act_feedback_ev;
static void act_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback, uint8_t num_act);
PRINT_CONFIG_MSG("STABILIZATION_INDI_RPM_FEEDBACK")
#endif

float g1g2_pseudo_inv[INDI_NUM_ACT][INDI_OUTPUTS];
float g2[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
#ifdef STABILIZATION_INDI_G1
float g1[INDI_OUTPUTS][INDI_NUM_ACT] = STABILIZATION_INDI_G1;
#else // old defines TODO remove
#if INDI_OUTPUTS == 5
float g1[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                        STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW,
                                        STABILIZATION_INDI_G1_THRUST, STABILIZATION_INDI_G1_THRUST_X
                                       };
#else
float g1[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                        STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                       };
#endif
#endif

float g1g2[INDI_OUTPUTS][INDI_NUM_ACT];
float g1_est[INDI_OUTPUTS][INDI_NUM_ACT];
float g2_est[INDI_NUM_ACT];
float g1_init[INDI_OUTPUTS][INDI_NUM_ACT];
float g2_init[INDI_NUM_ACT];

Butterworth2LowPass actuator_lowpass_filters[INDI_NUM_ACT];
Butterworth2LowPass estimation_input_lowpass_filters[INDI_NUM_ACT];
Butterworth2LowPass measurement_lowpass_filters[3];
Butterworth2LowPass estimation_output_lowpass_filters[3];
Butterworth2LowPass acceleration_lowpass_filter;
Butterworth2LowPass acceleration_body_x_filter;
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
Butterworth2LowPass rates_filt_so[3];
#else
static struct FirstOrderLowPass rates_filt_fo[3];
#endif
struct FloatVect3 body_accel_f;

void init_filters(void);
void sum_g1_g2(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_eff_mat_g_indi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0;
  pprz_msg_send_EFF_MAT_G(trans, dev, AC_ID,
                                   1, &zero,
                                   1, &zero,
                                   1, &zero,
                      INDI_NUM_ACT, g1g2[0],
                      INDI_NUM_ACT, g1g2[1],
                      INDI_NUM_ACT, g1g2[2],
                      INDI_NUM_ACT, g1g2[3],
                      INDI_NUM_ACT, g2_est);
}

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &stab_att_sp_quat.qi,
                              &stab_att_sp_quat.qx,
                              &stab_att_sp_quat.qy,
                              &stab_att_sp_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}

static void send_att_full_indi(struct transport_tx *trans, struct link_device *dev)
{
  float zero = 0.0;
  struct FloatRates *body_rates = stateGetBodyRates_f();
  struct FloatEulers att, att_sp;
#if GUIDANCE_INDI_HYBRID
  float_eulers_of_quat_zxy(&att, stateGetNedToBodyQuat_f());
  struct FloatQuat stab_att_sp_quat_f;
  QUAT_FLOAT_OF_BFP(stab_att_sp_quat_f, stab_att_sp_quat);
  float_eulers_of_quat_zxy(&att_sp, &stab_att_sp_quat_f);
#else
  att = *stateGetNedToBodyEulers_f();
  EULERS_FLOAT_OF_BFP(att_sp, stab_att_sp_euler);
#endif
  pprz_msg_send_STAB_ATTITUDE(trans, dev, AC_ID,
                                      &att.phi, &att.theta, &att.psi,           // att
                                      &att_sp.phi, &att_sp.theta, &att_sp.psi,  // att.ref
                                      &body_rates->p,           // rate
                                      &body_rates->q,
                                      &body_rates->r,
                                      &angular_rate_ref.p,      // rate.ref
                                      &angular_rate_ref.q,
                                      &angular_rate_ref.r,
                                      &angular_acceleration[0], // ang.acc
                                      &angular_acceleration[1],
                                      &angular_acceleration[2],
                                      &angular_accel_ref.p,     // ang.acc.ref
                                      &angular_accel_ref.q,
                                      &angular_accel_ref.r,
                                      1, &zero,                 // inputs
                                      INDI_NUM_ACT, indi_u);    // out
}
#endif

/**
 * Function that initializes important values upon engaging INDI
 */
void stabilization_indi_init(void)
{
  // Initialize filters
  init_filters();

  int8_t i;
// If the deprecated STABILIZATION_INDI_ACT_DYN is used, convert it to the new FREQUENCY format
#ifdef STABILIZATION_INDI_ACT_FREQ
  // Initialize the array of pointers to the rows of g1g2
  for (i = 0; i < INDI_NUM_ACT; i++) {
    act_dyn_discrete[i] = 1-exp(-act_first_order_cutoff[i]/PERIODIC_FREQUENCY);
  }
#endif

#if STABILIZATION_INDI_RPM_FEEDBACK
  AbiBindMsgACT_FEEDBACK(STABILIZATION_INDI_ACT_FEEDBACK_ID, &act_feedback_ev, act_feedback_cb);
#endif

  float_vect_zero(actuator_state_filt_vectd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vectdd, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_d, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_dd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vect, INDI_NUM_ACT);

  //Calculate G1G2
  sum_g1_g2();

  // Do not compute if not needed
#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
  //Calculate G1G2_PSEUDO_INVERSE
  calc_g1g2_pseudo_inv();
#endif

  // Initialize the array of pointers to the rows of g1g2
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // Initialize the estimator matrices
  float_vect_copy(g1_est[0], g1[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2_est, g2, INDI_NUM_ACT);
  // Remember the initial matrices
  float_vect_copy(g1_init[0], g1[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2_init, g2, INDI_NUM_ACT);

  // Assume all non-servos are delivering thrust
  num_thrusters = INDI_NUM_ACT;
  num_thrusters_x = 0;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    num_thrusters -= act_is_servo[i];
    num_thrusters -= act_is_thruster_x[i];

    num_thrusters_x += act_is_thruster_x[i];

    act_is_thruster_z[i] = !act_is_servo[i] && !act_is_thruster_x[i];
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_MAT_G, send_eff_mat_g_indi);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE, send_att_full_indi);
#endif
}

/**
 * Function that resets important values upon engaging INDI.
 *
 * Don't reset inputs and filters, because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 * FIXME: Ideally we should detect when coming from something that is not INDI
 */
void stabilization_indi_enter(void)
{
  float_vect_zero(du_estimation, INDI_NUM_ACT);
  float_vect_zero(ddu_estimation, INDI_NUM_ACT);
}

void stabilization_indi_update_filt_freq(float freq)
{
  stabilization_indi_filter_freq = freq;
  float tau = 1.0 / (2.0 * M_PI * freq);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
  init_butterworth_2_low_pass(&rates_filt_so[0], tau, sample_time, stateGetBodyRates_f()->p);
  init_butterworth_2_low_pass(&rates_filt_so[1], tau, sample_time, stateGetBodyRates_f()->q);
  init_butterworth_2_low_pass(&rates_filt_so[2], tau, sample_time, stateGetBodyRates_f()->r);
#else
  init_first_order_low_pass(&rates_filt_fo[0], tau, sample_time, stateGetBodyRates_f()->p);
  init_first_order_low_pass(&rates_filt_fo[1], tau, sample_time, stateGetBodyRates_f()->q);
  init_first_order_low_pass(&rates_filt_fo[2], tau, sample_time, stateGetBodyRates_f()->r);
#endif
}

/**
 * Function that resets the filters to zeros
 */
void init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF);
  float tau_est = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  // Filtering of the gyroscope
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&measurement_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], tau_est, sample_time, 0.0);
  }

  // Filtering of the actuators
  for (i = 0; i < INDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&actuator_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], tau_est, sample_time, 0.0);
  }

  // Filtering the bodyx acceleration with same cutoff as gyroscope
  init_butterworth_2_low_pass(&acceleration_body_x_filter, tau, sample_time, 0.0);

  // Filtering of the accel body z
  init_butterworth_2_low_pass(&acceleration_lowpass_filter, tau_est, sample_time, 0.0);

#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
  tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_P);
  init_butterworth_2_low_pass(&rates_filt_so[0], tau, sample_time, 0.0);
  tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_Q);
  init_butterworth_2_low_pass(&rates_filt_so[1], tau, sample_time, 0.0);
  tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_R);
  init_butterworth_2_low_pass(&rates_filt_so[2], tau, sample_time, 0.0);
#else
  // Init rate filter for feedback
  float time_constants[3] = {1.0 / (2 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_P), 1.0 / (2 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_Q), 1.0 / (2 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_R)};

  init_first_order_low_pass(&rates_filt_fo[0], time_constants[0], sample_time, stateGetBodyRates_f()->p);
  init_first_order_low_pass(&rates_filt_fo[1], time_constants[1], sample_time, stateGetBodyRates_f()->q);
  init_first_order_low_pass(&rates_filt_fo[2], time_constants[2], sample_time, stateGetBodyRates_f()->r);
#endif
}

/**
 * @param in_flight boolean that states if the UAV is in flight or not
 * @param sp rate setpoint
 * @param thrust thrust setpoint
 * @param cmd output command array
 *
 * Function that calculates the INDI commands
 */
void stabilization_indi_rate_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{

  // Propagate actuator filters
  get_actuator_state();
  float actuator_state_filt_vect_prev[INDI_NUM_ACT];
  for (int i = 0; i < INDI_NUM_ACT; i++) {
    update_butterworth_2_low_pass(&actuator_lowpass_filters[i], actuator_state[i]);
    update_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], actuator_state[i]);
    actuator_state_filt_vect[i] = actuator_lowpass_filters[i].o[0];
    actuator_state_filt_vect_prev[i] = actuator_lowpass_filters[i].o[1];

    // calculate derivatives for estimation
    float actuator_state_filt_vectd_prev = actuator_state_filt_vectd[i];
    actuator_state_filt_vectd[i] = (estimation_input_lowpass_filters[i].o[0] - estimation_input_lowpass_filters[i].o[1]) *
                                   PERIODIC_FREQUENCY;
    actuator_state_filt_vectdd[i] = (actuator_state_filt_vectd[i] - actuator_state_filt_vectd_prev) * PERIODIC_FREQUENCY;
  }

  // Use the last actuator state for this computation
  float g2_times_u_act_filt = float_vect_dot_product(g2, actuator_state_filt_vect_prev, INDI_NUM_ACT)/INDI_G_SCALING;

  // Predict angular acceleration u*B
  float angular_acc_prediction_filt[INDI_OUTPUTS];
  float_mat_vect_mul(angular_acc_prediction_filt, Bwls, actuator_state_filt_vect, INDI_OUTPUTS, INDI_NUM_ACT);
  angular_acc_prediction_filt[2] -= g2_times_u_act_filt;

  /* Propagate the filter on the gyroscopes */
  struct FloatRates *body_rates = stateGetBodyRates_f();
  float rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};

  // Get the acceleration in body axes
  struct Int32Vect3 *body_accel_i;
  body_accel_i = stateGetAccelBody_i();
  ACCELS_FLOAT_OF_BFP(body_accel_f, *body_accel_i);

  int8_t i;
  for (i = 0; i < 3; i++) {
    update_butterworth_2_low_pass(&measurement_lowpass_filters[i], rate_vect[i]);
    update_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], rate_vect[i]);

    update_butterworth_2_low_pass(&acceleration_body_x_filter, body_accel_f.x);

    //Calculate the angular acceleration via finite difference
    angular_acceleration[i] = (measurement_lowpass_filters[i].o[0]
                               - measurement_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;

    // Calculate derivatives for estimation
    float estimation_rate_d_prev = estimation_rate_d[i];
    estimation_rate_d[i] = (estimation_output_lowpass_filters[i].o[0] - estimation_output_lowpass_filters[i].o[1]) *
                           PERIODIC_FREQUENCY;
    estimation_rate_dd[i] = (estimation_rate_d[i] - estimation_rate_d_prev) * PERIODIC_FREQUENCY;
  }

  // subtract u*B from angular acceleration
  float angular_acc_disturbance_estimate[INDI_OUTPUTS];
  float_vect_diff(angular_acc_disturbance_estimate, angular_acceleration, angular_acc_prediction_filt, 3);

  if (in_flight) {
    // Limit the estimated disturbance in yaw for drones that are stable in sideslip
    BoundAbs(angular_acc_disturbance_estimate[2], stablization_indi_yaw_dist_limit);
  } else {
    // Not in flight, so don't estimate disturbance
    float_vect_zero(angular_acc_disturbance_estimate, INDI_OUTPUTS);
  }

  //The rates used for feedback are by default the measured rates.
  //If there is a lot of noise on the gyroscope, it might be good to use the filtered value for feedback.
  //Note that due to the delay, the PD controller may need relaxed gains.
  struct FloatRates rates_filt;
#if STABILIZATION_INDI_FILTER_ROLL_RATE
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
  rates_filt.p = update_butterworth_2_low_pass(&rates_filt_so[0], body_rates->p);
#else
  rates_filt.p = update_first_order_low_pass(&rates_filt_fo[0], body_rates->p);
#endif
#else
  rates_filt.p = body_rates->p;
#endif
#if STABILIZATION_INDI_FILTER_PITCH_RATE
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
  rates_filt.q = update_butterworth_2_low_pass(&rates_filt_so[1], body_rates->q);
#else
  rates_filt.q = update_first_order_low_pass(&rates_filt_fo[1], body_rates->q);
#endif
#else
  rates_filt.q = body_rates->q;
#endif
#if STABILIZATION_INDI_FILTER_YAW_RATE
#if STABILIZATION_INDI_FILTER_RATES_SECOND_ORDER
  rates_filt.r = update_butterworth_2_low_pass(&rates_filt_so[2], body_rates->r);
#else
  rates_filt.r = update_first_order_low_pass(&rates_filt_fo[2], body_rates->r);
#endif
#else
  rates_filt.r = body_rates->r;
#endif

  // calculate the virtual control (reference acceleration) based on a PD controller
  struct FloatRates rate_sp = stab_sp_to_rates_f(sp);
  angular_accel_ref.p = (rate_sp.p - rates_filt.p) * indi_gains.rate.p;
  angular_accel_ref.q = (rate_sp.q - rates_filt.q) * indi_gains.rate.q;
  angular_accel_ref.r = (rate_sp.r - rates_filt.r) * indi_gains.rate.r;

  // compute virtual thrust
  struct FloatVect3 v_thrust = { 0.f, 0.f, 0.f };
  if (thrust->type == THRUST_INCR_SP) {
    v_thrust.x = th_sp_to_incr_f(thrust, 0, THRUST_AXIS_X);
    v_thrust.y = th_sp_to_incr_f(thrust, 0, THRUST_AXIS_Y);
    v_thrust.z = th_sp_to_incr_f(thrust, 0, THRUST_AXIS_Z);

    // Compute estimated thrust
    struct FloatVect3 thrust_filt = { 0.f, 0.f, 0.f };
    for (i = 0; i < INDI_NUM_ACT; i++) {
      thrust_filt.z += Bwls[3][i]* actuator_lowpass_filters[i].o[0] * (int32_t) act_is_thruster_z[i];
#if INDI_OUTPUTS == 5
      thrust_filt.x += Bwls[4][i]* actuator_lowpass_filters[i].o[0] * (int32_t) act_is_thruster_x[i];
#endif
    }
    // Add the current estimated thrust to the increment
    VECT3_ADD(v_thrust, thrust_filt);
  } else {
    // build incremental thrust
    float th_cmd_z = (float)th_sp_to_thrust_i(thrust, 0, THRUST_AXIS_Z);
    for (i = 0; i < INDI_NUM_ACT; i++) {
      v_thrust.z += th_cmd_z * Bwls[3][i];
#if INDI_OUTPUTS == 5
      // TODO set X thrust from RC in the thrust input setpoint
      cmd[COMMAND_THRUST_X] = radio_control.values[RADIO_CONTROL_THRUST_X];
      v_thrust.x += cmd[COMMAND_THRUST_X] * Bwls[4][i];
#endif
    }
    v_thrust.y = 0.f;
  }

  // This term compensates for the spinup torque in the yaw axis
  float g2_times_u = float_vect_dot_product(g2, indi_u, INDI_NUM_ACT)/INDI_G_SCALING;

  if (in_flight) {
    // Limit the estimated disturbance in yaw for drones that are stable in sideslip
    BoundAbs(angular_acc_disturbance_estimate[2], stablization_indi_yaw_dist_limit);
  } else {
    // Not in flight, so don't estimate disturbance
    float_vect_zero(angular_acc_disturbance_estimate, INDI_OUTPUTS);
  }

  // The control objective in array format
  indi_v[0] = (angular_accel_ref.p - angular_acc_disturbance_estimate[0]);
  indi_v[1] = (angular_accel_ref.q - angular_acc_disturbance_estimate[1]);
  indi_v[2] = (angular_accel_ref.r - angular_acc_disturbance_estimate[2]) + g2_times_u;
  indi_v[3] = v_thrust.z;
#if INDI_OUTPUTS == 5
  indi_v[4] = v_thrust.x;
#endif

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
  // Calculate the increment for each actuator
  for (i = 0; i < INDI_NUM_ACT; i++) {
    indi_u[i] = (g1g2_pseudo_inv[i][0] * indi_v[0])
                + (g1g2_pseudo_inv[i][1] * indi_v[1])
                + (g1g2_pseudo_inv[i][2] * indi_v[2])
                + (g1g2_pseudo_inv[i][3] * indi_v[3]);
  }
#else
  stabilization_indi_set_wls_settings();

  // WLS Control Allocator
  num_iter =
    wls_alloc(indi_u, indi_v, u_min_stab_indi, u_max_stab_indi, Bwls, 0, 0, Wv, indi_Wu, u_pref_stab_indi, 10000, 10,
              INDI_NUM_ACT, INDI_OUTPUTS);
#endif

  // Bound the inputs to the actuators
  for (i = 0; i < INDI_NUM_ACT; i++) {
    if (act_is_servo[i]) {
      BoundAbs(indi_u[i], MAX_PPRZ);
    } else {
      if (autopilot_get_motors_on()) {
        Bound(indi_u[i], 0, MAX_PPRZ);
      } else {
        indi_u[i] = -MAX_PPRZ;
      }
    }
  }

  // Use online effectiveness estimation only when flying
  if (in_flight && indi_use_adaptive) {
    lms_estimation();
  }

  /*Commit the actuator command*/
  for (i = 0; i < INDI_NUM_ACT; i++) {
    actuators_pprz[i] = (int16_t) indi_u[i];
  }

  //update thrust command such that the current is correctly estimated
  cmd[COMMAND_THRUST] = 0;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    cmd[COMMAND_THRUST] += actuator_state[i] * (int32_t) act_is_thruster_z[i];
  }
  cmd[COMMAND_THRUST] /= num_thrusters;

}

/**
 * Function that sets the u_min, u_max and u_pref if function not elsewhere defined
 */
void WEAK stabilization_indi_set_wls_settings(void)
{
  // Calculate the min and max increments
  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    u_min_stab_indi[i] = -MAX_PPRZ * act_is_servo[i];
    u_max_stab_indi[i] = MAX_PPRZ;
    u_pref_stab_indi[i] = act_pref[i];

#ifdef GUIDANCE_INDI_MIN_THROTTLE
    float airspeed = stateGetAirspeed_f();
    //limit minimum thrust ap can give
    if (!act_is_servo[i]) {
      if ((guidance_h.mode == GUIDANCE_H_MODE_HOVER) || (guidance_h.mode == GUIDANCE_H_MODE_NAV)) {
        if (airspeed < STABILIZATION_INDI_THROTTLE_LIMIT_AIRSPEED_FWD) {
          u_min_stab_indi[i] = GUIDANCE_INDI_MIN_THROTTLE;
        } else {
          u_min_stab_indi[i] = GUIDANCE_INDI_MIN_THROTTLE_FWD;
        }
      }
    }
#endif
  }
}

/**
 * @param in_flight enable integrator only in flight
 * @param att_sp attitude stabilization setpoint
 * @param thrust thrust setpoint
 * @param[out] output command vector
 *
 * Function that should be called to run the INDI controller
 */
void stabilization_indi_attitude_run(bool in_flight, struct StabilizationSetpoint *att_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  stab_att_sp_euler = stab_sp_to_eulers_i(att_sp);  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_quat = stab_sp_to_quat_i(att_sp);     // quat attitude setpoint

  /* attitude error in float */
  struct FloatQuat att_err;
  struct FloatQuat *att_quat = stateGetNedToBodyQuat_f();
  struct FloatQuat quat_sp = stab_sp_to_quat_f(att_sp);

  float_quat_inv_comp_norm_shortest(&att_err, att_quat, &quat_sp);

  struct FloatVect3 att_fb;
#if TILT_TWIST_CTRL
  struct FloatQuat tilt;
  struct FloatQuat twist;
  float_quat_tilt_twist(&tilt, &twist, &att_err);
  att_fb.x = tilt.qx;
  att_fb.y = tilt.qy;
  att_fb.z = twist.qz;
#else
  att_fb.x = att_err.qx;
  att_fb.y = att_err.qy;
  att_fb.z = att_err.qz;
#endif

  // local variable to compute rate setpoints based on attitude error
  struct FloatRates rate_sp;
  // calculate the virtual control (reference acceleration) based on a PD controller
  rate_sp.p = indi_gains.att.p * att_fb.x / indi_gains.rate.p;
  rate_sp.q = indi_gains.att.q * att_fb.y / indi_gains.rate.q;
  rate_sp.r = indi_gains.att.r * att_fb.z / indi_gains.rate.r;

  // Add feed-forward rates to the attitude feedback part
  struct FloatRates ff_rates = stab_sp_to_rates_f(att_sp);
  RATES_ADD(rate_sp, ff_rates);

  // Store for telemetry
  angular_rate_ref.p = rate_sp.p;
  angular_rate_ref.q = rate_sp.q;
  angular_rate_ref.r = rate_sp.r;

  // Possibly we can use some bounding here
  /*BoundAbs(rate_sp.r, 5.0);*/

  /* compute the INDI command */
  struct StabilizationSetpoint sp = stab_sp_from_rates_f(&rate_sp);
  stabilization_indi_rate_run(in_flight, &sp, thrust, cmd);
}

/**
 * Function that tries to get actuator feedback.
 *
 * If this is not available it will use a first order filter to approximate the actuator state.
 * It is also possible to model rate limits (unit: PPRZ/loop cycle)
 */
void get_actuator_state(void)
{
#if STABILIZATION_INDI_RPM_FEEDBACK
  float_vect_copy(actuator_state, act_obs, INDI_NUM_ACT);
#else
  //actuator dynamics
  int8_t i;
  float UNUSED prev_actuator_state;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    prev_actuator_state = actuator_state[i];

    actuator_state[i] = actuator_state[i]
                        + act_dyn_discrete[i] * (indi_u[i] - actuator_state[i]);

#ifdef STABILIZATION_INDI_ACT_RATE_LIMIT
    if ((actuator_state[i] - prev_actuator_state) > act_rate_limit[i]) {
      actuator_state[i] = prev_actuator_state + act_rate_limit[i];
    } else if ((actuator_state[i] - prev_actuator_state) < -act_rate_limit[i]) {
      actuator_state[i] = prev_actuator_state - act_rate_limit[i];
    }
#endif
  }

#endif
}

/**
 * @param ddx_error error in output change
 * @param i row of the matrix element
 * @param j column of the matrix element
 * @param mu learning rate
 *
 * Function that calculates an element of the G1 matrix.
 * The elements are stored in a different matrix,
 * because the old matrix is necessary to caclulate more elements.
 */
void calc_g1_element(float ddx_error, int8_t i, int8_t j, float mu)
{
  g1_est[i][j] = g1_est[i][j] - du_estimation[j] * mu * ddx_error;
}

/**
 * @param ddx_error error in output change
 * @param j column of the matrix element
 * @param mu learning rate
 *
 * Function that calculates an element of the G2 matrix.
 * The elements are stored in a different matrix,
 * because the old matrix is necessary to caclulate more elements.
 */
void calc_g2_element(float ddx_error, int8_t j, float mu)
{
  g2_est[j] = g2_est[j] - ddu_estimation[j] * mu * ddx_error;
}

/**
 * Function that estimates the control effectiveness of each actuator online.
 * It is assumed that disturbances do not play a large role.
 * All elements of the G1 and G2 matrices are be estimated.
 */
void lms_estimation(void)
{

  // Get the acceleration in body axes
  struct Int32Vect3 *body_accel_i;
  body_accel_i = stateGetAccelBody_i();
  ACCELS_FLOAT_OF_BFP(body_accel_f, *body_accel_i);

  // Filter the acceleration in z axis
  update_butterworth_2_low_pass(&acceleration_lowpass_filter, body_accel_f.z);

  // Calculate the derivative of the acceleration via finite difference
  float indi_accel_d = (acceleration_lowpass_filter.o[0]
                        - acceleration_lowpass_filter.o[1]) * PERIODIC_FREQUENCY;

  // Use xml setting for adaptive mu for lms
  // Set default value if not defined
#ifndef STABILIZATION_INDI_ADAPTIVE_MU
  float adaptive_mu_lr = 0.001;
#else
  float adaptive_mu_lr = STABILIZATION_INDI_ADAPTIVE_MU;
#endif

  // scale the inputs to avoid numerical errors
  float_vect_smul(du_estimation, actuator_state_filt_vectd, adaptive_mu_lr, INDI_NUM_ACT);
  float_vect_smul(ddu_estimation, actuator_state_filt_vectdd, adaptive_mu_lr / PERIODIC_FREQUENCY, INDI_NUM_ACT);

  float ddx_estimation[INDI_OUTPUTS] = {estimation_rate_dd[0], estimation_rate_dd[1], estimation_rate_dd[2], indi_accel_d};

  //Estimation of G
  // TODO: only estimate when du_norm2 is large enough (enough input)
  /*float du_norm2 = du_estimation[0]*du_estimation[0] + du_estimation[1]*du_estimation[1] +du_estimation[2]*du_estimation[2] + du_estimation[3]*du_estimation[3];*/
  int8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    // Calculate the error between prediction and measurement
    float ddx_error = - ddx_estimation[i];
    int8_t j;
    for (j = 0; j < INDI_NUM_ACT; j++) {
      ddx_error += g1_est[i][j] * du_estimation[j];
      if (i == 2) {
        // Changing the momentum of the rotors gives a counter torque
        ddx_error += g2_est[j] * ddu_estimation[j];
      }
    }

    // when doing the yaw axis, also use G2
    if (i == 2) {
      for (j = 0; j < INDI_NUM_ACT; j++) {
        calc_g2_element(ddx_error, j, mu2);
      }
    } else if (i == 3) {
      // If the acceleration change is very large (rough landing), don't adapt
      if (fabs(indi_accel_d) > 60.0) {
        ddx_error = 0.0;
      }
    }

    // Calculate the row of the G1 matrix corresponding to this axis
    for (j = 0; j < INDI_NUM_ACT; j++) {
      calc_g1_element(ddx_error, i, j, mu1[i]);
    }
  }

  bound_g_mat();

  // Save the calculated matrix to G1 and G2
  // until thrust is included, first part of the array
  float_vect_copy(g1[0], g1_est[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2, g2_est, INDI_NUM_ACT);

  // Calculate sum of G1 and G2 for Bwls
  sum_g1_g2();

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
  // Calculate the inverse of (G1+G2)
  calc_g1g2_pseudo_inv();
#endif
}

/**
 * Function that sums g1 and g2 to obtain the g1g2 matrix
 * It also undoes the scaling that was done to make the values readable
 */
void sum_g1_g2(void)
{
  int8_t i;
  int8_t j;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    for (j = 0; j < INDI_NUM_ACT; j++) {
      if (i != 2) {
        g1g2[i][j] = g1[i][j] / INDI_G_SCALING;
      } else {
        g1g2[i][j] = (g1[i][j] + g2[j]) / INDI_G_SCALING;
      }
    }
  }
}

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
/**
 * Function that calculates the pseudo-inverse of (G1+G2).
 * Make sure to sum of G1 and G2 before running this!
 */
void calc_g1g2_pseudo_inv(void)
{
  //G1G2*transpose(G1G2)
  //calculate matrix multiplication of its transpose INDI_OUTPUTSxnum_act x num_actxINDI_OUTPUTS
  float element = 0;
  int8_t row;
  int8_t col;
  int8_t i;
  for (row = 0; row < INDI_OUTPUTS; row++) {
    for (col = 0; col < INDI_OUTPUTS; col++) {
      element = 0;
      for (i = 0; i < INDI_NUM_ACT; i++) {
        element = element + g1g2[row][i] * g1g2[col][i];
      }
      g1g2_trans_mult[row][col] = element;
    }
  }

  //there are numerical errors if the scaling is not right.
  float_vect_scale(g1g2_trans_mult[0], 1000.0, INDI_OUTPUTS * INDI_OUTPUTS);

  //inverse of 4x4 matrix
  float_mat_inv_4d(g1g2inv, g1g2_trans_mult);

  //scale back
  float_vect_scale(g1g2inv[0], 1000.0, INDI_OUTPUTS * INDI_OUTPUTS);

  //G1G2'*G1G2inv
  //calculate matrix multiplication INDI_NUM_ACTxINDI_OUTPUTS x INDI_OUTPUTSxINDI_OUTPUTS
  for (row = 0; row < INDI_NUM_ACT; row++) {
    for (col = 0; col < INDI_OUTPUTS; col++) {
      element = 0;
      for (i = 0; i < INDI_OUTPUTS; i++) {
        element = element + g1g2[i][row] * g1g2inv[col][i];
      }
      g1g2_pseudo_inv[row][col] = element;
    }
  }
}
#endif

#if STABILIZATION_INDI_RPM_FEEDBACK
static void act_feedback_cb(uint8_t sender_id UNUSED, struct act_feedback_t *feedback, uint8_t num_act)
{
  int8_t i;
  for (i = 0; i < num_act; i++) {
    // Sanity check that index is valid
    if (feedback[i].idx < INDI_NUM_ACT && feedback[i].set.rpm) {
      int8_t idx = feedback[i].idx;
      act_obs[idx] = (feedback[i].rpm - get_servo_min(idx));
      act_obs[idx] *= (MAX_PPRZ / (float)(get_servo_max(idx) - get_servo_min(idx)));
      Bound(act_obs[idx], 0, MAX_PPRZ);
    }
  }
}
#endif

static void bound_g_mat(void)
{
  int8_t i;
  int8_t j;
  for (j = 0; j < INDI_NUM_ACT; j++) {
    float max_limit;
    float min_limit;

    // Limit the values of the estimated G1 matrix
    for (i = 0; i < INDI_OUTPUTS; i++) {
      if (g1_init[i][j] > 0.0) {
        max_limit = g1_init[i][j] * INDI_ALLOWED_G_FACTOR;
        min_limit = g1_init[i][j] / INDI_ALLOWED_G_FACTOR;
      } else {
        max_limit = g1_init[i][j] / INDI_ALLOWED_G_FACTOR;
        min_limit = g1_init[i][j] * INDI_ALLOWED_G_FACTOR;
      }

      if (g1_est[i][j] > max_limit) {
        g1_est[i][j] = max_limit;
      }
      if (g1_est[i][j] < min_limit) {
        g1_est[i][j] = min_limit;
      }
    }

    // Do the same for the G2 matrix
    if (g2_init[j] > 0.0) {
      max_limit = g2_init[j] * INDI_ALLOWED_G_FACTOR;
      min_limit = g2_init[j] / INDI_ALLOWED_G_FACTOR;
    } else {
      max_limit = g2_init[j] / INDI_ALLOWED_G_FACTOR;
      min_limit = g2_init[j] * INDI_ALLOWED_G_FACTOR;
    }

    if (g2_est[j] > max_limit) {
      g2_est[j] = max_limit;
    }
    if (g2_est[j] < min_limit) {
      g2_est[j] = min_limit;
    }
  }
}
