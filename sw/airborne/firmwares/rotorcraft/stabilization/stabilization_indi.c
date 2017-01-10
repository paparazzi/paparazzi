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
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/radio_control.h"
#include "subsystems/actuators.h"
#include "subsystems/abi.h"
#include "filters/low_pass_filter.h"

static void lms_estimation(void);
static void get_actuator_state(void);
static void calc_g1_element(float dx_error, int8_t i, int8_t j, float mu_extra);
static void calc_g2_element(float dx_error, int8_t j, float mu_extra);
static void calc_g1g2_pseudo_inv(void);
static void bound_g_mat(void);

int32_t stabilization_att_indi_cmd[COMMANDS_NB];
struct ReferenceSystem reference_acceleration = {
  STABILIZATION_INDI_REF_ERR_P,
  STABILIZATION_INDI_REF_ERR_Q,
  STABILIZATION_INDI_REF_ERR_R,
  STABILIZATION_INDI_REF_RATE_P,
  STABILIZATION_INDI_REF_RATE_Q,
  STABILIZATION_INDI_REF_RATE_R,
};

//only 4 actuators supported for now
#define INDI_NUM_ACT 4
// outputs: roll, pitch, yaw, thrust
#define INDI_OUTPUTS 4
// Factor that the estimated G matrix is allowed to deviate from initial one
#define INDI_ALLOWED_G_FACTOR 2.0
// Scaling for the control effectiveness to make it readible
#define INDI_G_SCALING 1000.0

#if STABILIZATION_INDI_USE_ADAPTIVE
bool indi_use_adaptive = true;
#else
bool indi_use_adaptive = false;
#endif

// variables needed for control
float actuator_state_filt_vect[INDI_NUM_ACT];
struct FloatRates angular_accel_ref = {0., 0., 0.};
float angular_acceleration[3] = {0., 0., 0.};
float actuator_state[INDI_NUM_ACT];
float indi_u[INDI_NUM_ACT];
float indi_du[INDI_NUM_ACT];
float g2_times_du;

// variables needed for estimation
float g1g2_trans_mult[INDI_OUTPUTS][INDI_OUTPUTS];
float g1g2inv[INDI_OUTPUTS][INDI_OUTPUTS];
float actuator_state_filt_vectd[INDI_NUM_ACT];
float actuator_state_filt_vectdd[INDI_NUM_ACT];
float estimation_rate_d[INDI_NUM_ACT];
float estimation_rate_dd[INDI_NUM_ACT];
float du_estimation[INDI_NUM_ACT];
float ddu_estimation[INDI_NUM_ACT];
float mu1[4] = {0.00001, 0.00001, 0.000003, 0.000002};
float mu2 = 0.002;

// other variables
float act_obs[INDI_NUM_ACT];

struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;

abi_event rpm_ev;
static void rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t num_act);

abi_event thrust_ev;
static void thrust_cb(uint8_t sender_id, float thrust_increment);
float indi_thrust_increment;
bool indi_thrust_increment_set = false;

float g1g2_pseudo_inv[INDI_NUM_ACT][INDI_OUTPUTS];
float g2[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
  STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST};
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

struct FloatVect3 body_accel_f;

void init_filters(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_indi_g(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INDI_G(trans, dev, AC_ID, INDI_NUM_ACT, g1_est[0],
                                          INDI_NUM_ACT, g1_est[1],
                                          INDI_NUM_ACT, g1_est[2],
                                          INDI_NUM_ACT, g1_est[3],
                                          INDI_NUM_ACT, g2_est);
}
#endif

/**
 * Function that initializes important values upon engaging INDI
 */
void stabilization_indi_init(void)
{
  // Initialize filters
  init_filters();

  AbiBindMsgRPM(RPM_SENSOR_ID, &rpm_ev, rpm_cb);
  AbiBindMsgTHRUST(THRUST_INCREMENT_ID, &thrust_ev, thrust_cb);

  float_vect_zero(actuator_state_filt_vectd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vectdd, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_d, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_dd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vect, INDI_NUM_ACT);

  //Calculate G1G2_PSEUDO_INVERSE
  calc_g1g2_pseudo_inv();

  // Initialize the estimator matrices
  float_vect_copy(g1_est[0], g1[0], INDI_OUTPUTS*INDI_NUM_ACT);
  float_vect_copy(g2_est, g2, INDI_NUM_ACT);
  // Remember the initial matrices
  float_vect_copy(g1_init[0], g1[0], INDI_OUTPUTS*INDI_NUM_ACT);
  float_vect_copy(g2_init, g2, INDI_NUM_ACT);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INDI_G, send_indi_g);
#endif
}

/**
 * Function that resets important values upon engaging INDI
 */
void stabilization_indi_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  // reset filters
  init_filters();


  float_vect_zero(actuator_state, INDI_NUM_ACT);
  float_vect_zero(indi_u, INDI_NUM_ACT);
  float_vect_zero(indi_du, INDI_NUM_ACT);
  float_vect_zero(du_estimation, INDI_NUM_ACT);
  float_vect_zero(ddu_estimation, INDI_NUM_ACT);
}

/**
 * Function that resets the filters to zeros
 */
void init_filters(void) {
  // tau = 1/(2*pi*Fc)
  float tau = 1.0/(2.0*M_PI*STABILIZATION_INDI_FILT_CUTOFF);
  float tau_est = 1.0/(2.0*M_PI*STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
  float sample_time = 1.0/PERIODIC_FREQUENCY;
  // Filtering of the gyroscope
  int8_t i;
  for(i=0; i<3; i++) {
    init_butterworth_2_low_pass(&measurement_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], tau_est, sample_time, 0.0);
  }

  // Filtering of the actuators
  for(i=0; i<INDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&actuator_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], tau_est, sample_time, 0.0);
  }

  // Filtering of the accel body z
  init_butterworth_2_low_pass(&acceleration_lowpass_filter, tau_est, sample_time, 0.0);
}

/**
 * Function that calculates the failsafe setpoint
 */
void stabilization_indi_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

/**
 * @param rpy rpy from which to calculate quaternion setpoint
 *
 * Function that calculates the setpoint quaternion from rpy
 */
void stabilization_indi_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  memcpy(&stab_att_sp_euler, rpy, sizeof(struct Int32Eulers));

  quat_from_rpy_cmd_i(&stab_att_sp_quat, &stab_att_sp_euler);
}

/**
 * @param cmd 2D command in North East axes
 * @param heading Heading of the setpoint
 *
 * Function that calculates the setpoint quaternion from a command in earth axes
 */
void stabilization_indi_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

/**
 * @param att_err attitude error
 * @param rate_control boolean that states if we are in rate control or attitude control
 * @param in_flight boolean that states if the UAV is in flight or not
 *
 * Function that calculates the INDI commands
 */
static void stabilization_indi_calc_cmd(struct Int32Quat *att_err, bool rate_control, bool in_flight)
{
  //calculate the virtual control (reference acceleration) based on a PD controller
  angular_accel_ref.p = reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                        - reference_acceleration.rate_p * stateGetBodyRates_f()->p;
  angular_accel_ref.q = reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                        - reference_acceleration.rate_q * stateGetBodyRates_f()->q;
  angular_accel_ref.r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                        - reference_acceleration.rate_r * stateGetBodyRates_f()->r;

  g2_times_du = 0.0;
  int8_t i;
  for(i=0; i<INDI_NUM_ACT; i++) {
    g2_times_du += g2[i]*indi_du[i];
  }
  //G2 is scaled by INDI_G_SCALING to make it readable
  g2_times_du = g2_times_du/INDI_G_SCALING;

  // Calculate the increment for each actuator
  for(i=0; i<INDI_NUM_ACT; i++) {
    indi_du[i] = (g1g2_pseudo_inv[i][0] * (angular_accel_ref.p - angular_acceleration[0]))
               + (g1g2_pseudo_inv[i][1] * (angular_accel_ref.q - angular_acceleration[1]))
               + (g1g2_pseudo_inv[i][2] * (angular_accel_ref.r - angular_acceleration[2] + g2_times_du));
  }

  if(indi_thrust_increment_set){
    // The required body-z acceleration is calculated by the outer loop INDI controller
    for(i=0; i<INDI_NUM_ACT; i++) {
      indi_du[i] = indi_du[i] + g1g2_pseudo_inv[i][3]*indi_thrust_increment;
    }

    // Add the increments to the actuators
    float_vect_sum(indi_u, actuator_state_filt_vect, indi_du, INDI_NUM_ACT);
  } else
  {
    // Add the increments to the actuators without the thrust
    float_vect_sum(indi_u, actuator_state_filt_vect, indi_du, INDI_NUM_ACT);

    // Calculate the average of the actuators as a measure for the thrust
    float avg_u_in = 0;
    for(i=0; i<INDI_NUM_ACT; i++) {
      avg_u_in += indi_u[i];
    }
    avg_u_in /= INDI_NUM_ACT;

    // Make sure the thrust is bounded
    Bound(stabilization_cmd[COMMAND_THRUST],0, MAX_PPRZ);

    //avoid dividing by zero
    if(avg_u_in < 1.0) {
      avg_u_in = 1.0;
    }

    // Rescale the command to the actuators to get the desired thrust
    float indi_cmd_scaling = stabilization_cmd[COMMAND_THRUST] / avg_u_in;
    float_vect_smul(indi_u, indi_u, indi_cmd_scaling, INDI_NUM_ACT);
  }

  // Bound the inputs to the actuators
  for(i=0; i<INDI_NUM_ACT; i++) {
    Bound(indi_u[i], 0, MAX_PPRZ);
  }

  // Propagate actuator filters
  get_actuator_state();
  for(i=0; i<INDI_NUM_ACT; i++) {
    update_butterworth_2_low_pass(&actuator_lowpass_filters[i], actuator_state[i]);
    update_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], actuator_state[i]);
    actuator_state_filt_vect[i] = actuator_lowpass_filters[i].o[0];

    // calculate derivatives for estimation
    float actuator_state_filt_vectd_prev = actuator_state_filt_vectd[i];
    actuator_state_filt_vectd[i] = (estimation_input_lowpass_filters[i].o[0] - estimation_input_lowpass_filters[i].o[1])*PERIODIC_FREQUENCY;
    actuator_state_filt_vectdd[i] = (actuator_state_filt_vectd[i] - actuator_state_filt_vectd_prev)*PERIODIC_FREQUENCY;
  }

  //Don't increment if thrust is off
  if(!in_flight) {
    float_vect_zero(indi_u, INDI_NUM_ACT);
  }
  else if(indi_use_adaptive) {
    lms_estimation();
  }

  /*Commit the actuator command*/
  for(i=0; i<INDI_NUM_ACT; i++) {
    actuators_pprz[i] = (int16_t) indi_u[i];
  }
}

/**
 * @param enable_integrator
 * @param rate_control boolean that determines if we are in rate control or attitude control
 *
 * Function that should be called to run the INDI controller
 */
void stabilization_indi_run(bool in_flight, bool rate_control)
{

  /* Propagate the filter on the gyroscopes */
  struct FloatRates *body_rates = stateGetBodyRates_f();
  float rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};
  int8_t i;
  for(i=0; i<3; i++) {
    update_butterworth_2_low_pass(&measurement_lowpass_filters[i], rate_vect[i]);
    update_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], rate_vect[i]);

    //Calculate the angular acceleration via finite difference
    angular_acceleration[i] = (measurement_lowpass_filters[i].o[0]
      - measurement_lowpass_filters[i].o[1])*PERIODIC_FREQUENCY;

    // Calculate derivatives for estimation
    float estimation_rate_d_prev = estimation_rate_d[i];
    estimation_rate_d[i] = (estimation_output_lowpass_filters[i].o[0] - estimation_output_lowpass_filters[i].o[1]) *PERIODIC_FREQUENCY;
    estimation_rate_dd[i] = (estimation_rate_d[i] - estimation_rate_d_prev) * PERIODIC_FREQUENCY;
  }

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
  int32_quat_inv_comp(&att_err, att_quat, &stab_att_sp_quat);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /* compute the INDI command */
  stabilization_indi_calc_cmd(&att_err, rate_control, in_flight);

  // Set the stab_cmd to 42 to indicate that it is not used
  stabilization_cmd[COMMAND_ROLL] = 42;
  stabilization_cmd[COMMAND_PITCH] = 42;
  stabilization_cmd[COMMAND_YAW] = 42;

  // Reset thrust increment boolean
  indi_thrust_increment_set = false;
}

// This function reads rc commands
void stabilization_indi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif

  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

/**
 * Function that tries to get actuator feedback.
 * If this is not available it will use a first order filter to approximate the actuator state.
 */
void get_actuator_state(void) {
#if INDI_RPM_FEEDBACK
  float_vect_copy(actuator_state, act_obs, INDI_NUM_ACT);
#else
  //actuator dynamics
  int8_t i;
  for(i=0; i<INDI_NUM_ACT; i++) {
    actuator_state[i] = actuator_state[i]
      + STABILIZATION_INDI_ACT_DYN*( indi_u[i] - actuator_state[i]);
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
void calc_g1_element(float ddx_error, int8_t i, int8_t j, float mu) {
  g1_est[i][j] = g1_est[i][j] - du_estimation[j]*mu*ddx_error;
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
void calc_g2_element(float ddx_error, int8_t j, float mu) {
  g2_est[j] = g2_est[j] - ddu_estimation[j]*mu*ddx_error;
}

/**
 * Function that estimates the control effectiveness of each actuator online.
 * It is assumed that disturbances do not play a large role.
 * All elements of the G1 and G2 matrices are be estimated.
 */
void lms_estimation(void) {

  // Get the acceleration in body axes
  struct Int32Vect3 *body_accel_i;
  body_accel_i = stateGetAccelBody_i();
  ACCELS_FLOAT_OF_BFP(body_accel_f, *body_accel_i);

  // Filter the acceleration in z axis
  update_butterworth_2_low_pass(&acceleration_lowpass_filter, body_accel_f.z);

  // Calculate the derivative of the acceleration via finite difference
  float indi_accel_d = (acceleration_lowpass_filter.o[0]
      - acceleration_lowpass_filter.o[1])*PERIODIC_FREQUENCY;

  // scale the inputs to avoid numerical errors
  float_vect_smul(du_estimation, actuator_state_filt_vectd, 0.001, INDI_NUM_ACT);
  float_vect_smul(ddu_estimation, actuator_state_filt_vectdd, 0.001/PERIODIC_FREQUENCY, INDI_NUM_ACT);

  float ddx_estimation[INDI_OUTPUTS] = {estimation_rate_dd[0], estimation_rate_dd[1], estimation_rate_dd[2], indi_accel_d};

  //Estimation of G
  // TODO: only estimate when du_norm2 is large enough (enough input)
  /*float du_norm2 = du_estimation[0]*du_estimation[0] + du_estimation[1]*du_estimation[1] +du_estimation[2]*du_estimation[2] + du_estimation[3]*du_estimation[3];*/
  int8_t i;
  for(i=0; i<INDI_OUTPUTS; i++) {
    // Calculate the error between prediction and measurement
    float ddx_error = - ddx_estimation[i];
    int8_t j;
    for(j=0; j<INDI_NUM_ACT; j++) {
      ddx_error += g1_est[i][j]*du_estimation[j];
      if(i==2) {
        // Changing the momentum of the rotors gives a counter torque
        ddx_error += g2_est[j]*ddu_estimation[j];
      }
    }

    // when doing the yaw axis, also use G2
    if(i==2) {
      for(j=0; j<INDI_NUM_ACT; j++) {
        calc_g2_element(ddx_error,j,mu2);
      }
    } else if(i==3) {
      // If the acceleration change is very large (rough landing), don't adapt
      if(fabs(indi_accel_d) > 60.0) {
        ddx_error = 0.0;
      }
    }

    // Calculate the row of the G1 matrix corresponding to this axis
    for(j=0; j<INDI_NUM_ACT; j++) {
      calc_g1_element(ddx_error, i, j, mu1[i]);
    }
  }

  bound_g_mat();

  // Save the calculated matrix to G1 and G2
  // until thrust is included, first part of the array
  float_vect_copy(g1[0], g1_est[0], INDI_OUTPUTS*INDI_NUM_ACT);
  float_vect_copy(g2, g2_est, INDI_NUM_ACT);

  // Calculate the inverse of (G1+G2)
  calc_g1g2_pseudo_inv();
}

/**
 * Function that calculates the pseudo-inverse of (G1+G2).
 */
void calc_g1g2_pseudo_inv(void) {

  //sum of G1 and G2
  int8_t i;
  int8_t j;
  for(i=0; i<INDI_OUTPUTS; i++) {
    for(j=0; j<INDI_NUM_ACT; j++) {
      if(i!=2)
        g1g2[i][j] = g1[i][j]/INDI_G_SCALING;
      else
        g1g2[i][j] = (g1[i][j] + g2[j])/INDI_G_SCALING;
    }
  }

  //G1G2*transpose(G1G2)
  //calculate matrix multiplication of its transpose INDI_OUTPUTSxnum_act x num_actxINDI_OUTPUTS
  float element = 0;
  int8_t row;
  int8_t col;
  for(row=0; row<INDI_OUTPUTS; row++) {
    for(col=0; col<INDI_OUTPUTS; col++) {
      element = 0;
      for(i=0; i<INDI_NUM_ACT; i++) {
        element = element + g1g2[row][i]*g1g2[col][i];
      }
      g1g2_trans_mult[row][col] = element;
    }
  }

  //there are numerical errors if the scaling is not right.
  float_vect_scale(g1g2_trans_mult[0], 100.0, INDI_OUTPUTS*INDI_NUM_ACT);

  //inverse of 4x4 matrix
  float_mat_inv_4d(g1g2inv[0], g1g2_trans_mult[0]);

  //scale back
  float_vect_scale(g1g2inv[0], 100.0, INDI_OUTPUTS*INDI_NUM_ACT);

  //G1G2'*G1G2inv
  //calculate matrix multiplication INDI_NUM_ACTxINDI_OUTPUTS x INDI_OUTPUTSxINDI_OUTPUTS
  for(row=0; row<INDI_NUM_ACT; row++) {
    for(col=0; col<INDI_OUTPUTS; col++) {
      element = 0;
      for(i=0; i<INDI_OUTPUTS; i++) {
        element = element + g1g2[i][row]*g1g2inv[col][i];
      }
      g1g2_pseudo_inv[row][col] = element;
    }
  }
}

static void rpm_cb(uint8_t __attribute__((unused)) sender_id, uint16_t *rpm, uint8_t num_act)
{
  int8_t i;
  for(i=0; i<num_act; i++) {
    act_obs[i] = (rpm[i] - get_servo_min(i));
    act_obs[i] *= (MAX_PPRZ / (float)(get_servo_max(i)-get_servo_min(i)));
  }
}

/**
 * ABI callback that obtains the thrust increment from guidance INDI
 */
static void thrust_cb(uint8_t sender_id, float thrust_increment)
{
  indi_thrust_increment = thrust_increment;
  indi_thrust_increment_set = true;
}

static void bound_g_mat(void) {
  int8_t i;
  int8_t j;
  for(j=0; j<INDI_NUM_ACT; j++) {
    float max_limit;
    float min_limit;

    // Limit the values of the estimated G1 matrix
    for(i=0; i<INDI_OUTPUTS; i++) {
      if(g1_init[i][j] > 0.0) {
        max_limit = g1_init[i][j]*INDI_ALLOWED_G_FACTOR;
        min_limit = g1_init[i][j]/INDI_ALLOWED_G_FACTOR;
      } else {
        max_limit = g1_init[i][j]/INDI_ALLOWED_G_FACTOR;
        min_limit = g1_init[i][j]*INDI_ALLOWED_G_FACTOR;
      }

      if(g1_est[i][j] > max_limit) {
        g1_est[i][j] = max_limit;
      }
      if(g1_est[i][j] < min_limit) {
        g1_est[i][j] = min_limit;
      }
    }

    // Do the same for the G2 matrix
    if(g2_init[j] > 0.0) {
      max_limit = g2_init[j]*INDI_ALLOWED_G_FACTOR;
      min_limit = g2_init[j]/INDI_ALLOWED_G_FACTOR;
    } else {
      max_limit = g2_init[j]/INDI_ALLOWED_G_FACTOR;
      min_limit = g2_init[j]*INDI_ALLOWED_G_FACTOR;
    }

    if(g2_est[j] > max_limit) {
      g2_est[j] = max_limit;
    }
    if(g2_est[j] < min_limit) {
      g2_est[j] = min_limit;
    }
  }
}
