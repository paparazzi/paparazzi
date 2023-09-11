/*
 * Copyright (C) 2015 Guido de Croon.
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/optical_flow_landing.h
 * @brief This module implements optical flow landings in which the divergence is kept constant.
 * When using a fixed gain for control, the covariance between thrust and divergence is tracked,
 * so that the drone knows when it has arrived close to the landing surface. Then, a final landing
 * procedure is triggered. It can also be set to adaptive gain control, where the goal is to continuously
 * gauge the distance to the landing surface. In this mode, the drone will oscillate all the way down to
 * the surface.
 *
 * de Croon, G.C.H.E. (2016). Monocular distance estimation with optical flow maneuvers and efference copies:
 * a stability-based strategy. Bioinspiration & biomimetics, 11(1), 016004.
 * <http://iopscience.iop.org/article/10.1088/1748-3190/11/1/016004>
 *
 * Based on the above theory, we have also developed a new strategy for landing that consists of two phases:
 * (1) while hovering, the drone determines the optimal gain by increasing the gain until oscillation
 * (2) the drone starts landing while exponentially decreasing the gain over time
 *
 * This strategy leads to smooth, high-performance constant divergence landings, as explained in the article:
 * H.W. Ho, G.C.H.E. de Croon, E. van Kampen, Q.P. Chu, and M. Mulder (submitted)
 * Adaptive Control Strategy for Constant Optical Flow Divergence Landing,
 * <https://arxiv.org/abs/1609.06767>
 *
 * The module also contains code for learning to map the visual appearance of the landing surface with
 * self-supervised learning (SSL) to the corresponding control gains. This leads to smooth optical flow
 * landings where the "height" expressed in the control gain is known and can be used to shut down the
 * motors. This has been published in the following paper:
 *
 * de Croon, Guido CHE, Christophe De Wagter, and Tobias Seidl. "Enhancing optical-flow-based control
 * by learning visual appearance cues for flying robots." Nature Machine Intelligence 3.1 (2021): 33-41.
 *
 * Finally, horizontal flow control has been added, so that the drone can hover without optitrack. In fact,
 * this module also formed the basis for the experiments in which the attitude is estimated with only optical
 * flow and gyro measurements (without accelerometers), published in Nature:
 *
 * De Croon, G.C.H.E., Dupeyroux, J.J., De Wagter, C., Chatterjee, A., Olejnik, D. A., & Ruffier, F. (2022).
 * Accommodating unobservability to control flight attitude with optic flow. Nature, 610(7932), 485-490.
 *
 */

#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "optical_flow_landing.h"

// SSL:
float *text_dists[MAX_SAMPLES_LEARNING];
float sonar_OF[MAX_SAMPLES_LEARNING];
float gains[MAX_SAMPLES_LEARNING];
float cov_divs_log[MAX_SAMPLES_LEARNING];
float *weights;

// for "communication" with file logger:
float cov_div;
float pstate;
float divergence_vision;


#include "generated/airframe.h"
#include "paparazzi.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"

// used for automated landing:
#include "autopilot.h"
#include "modules/nav/common_flight_plan.h"
#include "modules/datalink/telemetry.h"

// for measuring time
#include "mcu_periph/sys_time.h"

#include "math/pprz_stat.h"

/* Default sonar/agl to use */
#ifndef OFL_AGL_ID
#define OFL_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFL_AGL_ID)

/* Use optical flow estimates */
#ifndef OFL_OPTICAL_FLOW_ID
#define OFL_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFL_OPTICAL_FLOW_ID)

// Other default values:
#ifndef OFL_PGAIN
#define OFL_PGAIN 0.40
#endif

#ifndef OFL_IGAIN
#define OFL_IGAIN 0.01
#endif

#ifndef OFL_DGAIN
#define OFL_DGAIN 0.0
#endif

#ifndef OFL_PGAIN_HORIZONTAL_FACTOR
#define OFL_PGAIN_HORIZONTAL_FACTOR 0.05
#endif

#ifndef OFL_IGAIN_HORIZONTAL_FACTOR
#define OFL_IGAIN_HORIZONTAL_FACTOR 0.1
#endif

#ifndef OFL_ROLL_TRIM
#define OFL_ROLL_TRIM 0.0f
#endif

#ifndef OFL_PITCH_TRIM
#define OFL_PITCH_TRIM 0.0f
#endif

#ifndef OFL_VISION_METHOD
#define OFL_VISION_METHOD 1
#endif

#ifndef OFL_CONTROL_METHOD
#define OFL_CONTROL_METHOD 0
#endif

#ifndef OFL_COV_METHOD
#define OFL_COV_METHOD 0
#endif

// number of time steps used for calculating the covariance (oscillations)
#ifndef OFL_COV_WINDOW_SIZE
#define OFL_COV_WINDOW_SIZE 30
#endif

#ifndef OFL_COV_LANDING_LIMIT
#define OFL_COV_LANDING_LIMIT 2.2
#endif

#ifndef OFL_COV_SETPOINT
#define OFL_COV_SETPOINT -0.0075
#endif

#ifndef OFL_LP_CONST
#define OFL_LP_CONST 0.02
#endif

#ifndef OFL_P_LAND_THRESHOLD
#define OFL_P_LAND_THRESHOLD 0.15
#endif

#ifndef OFL_ELC_OSCILLATE
#define OFL_ELC_OSCILLATE true
#endif

#ifndef OFL_CLOSE_TO_EDGE
#define OFL_CLOSE_TO_EDGE 0.025
#endif


#ifndef OFL_PGAIN_ADAPTIVE
#define OFL_PGAIN_ADAPTIVE 0.50
#endif

#ifndef OFL_IGAIN_ADAPTIVE
#define OFL_IGAIN_ADAPTIVE 0.50
#endif

#ifndef OFL_DGAIN_ADAPTIVE
#define OFL_DGAIN_ADAPTIVE 0.50
#endif

#ifndef OFL_OMEGA_LR
#define OFL_OMEGA_LR 0.0
#endif

#ifndef OFL_OMEGA_FB
#define OFL_OMEGA_FB 0.0
#endif

#ifndef OFL_ACTIVE_MOTION
#define OFL_ACTIVE_MOTION 0
#endif

// Normally, horizontal control is done via sending angle commands to INDI, so 0 (false)
// When this is 1 (true),a change in angle will be commanded instead.
#define HORIZONTAL_RATE_CONTROL 0

// Normally, ACTIVE_RATES = 0, and the estimated angle is immediately used for controlling the error to 0
// If ACTIVE_RATES = 1, a sine is actively added to the commanded rates, so that there is often a rate
#define ACTIVE_RATES 0

// Constants
// minimum value of the P-gain for divergence control
// adaptive control / exponential gain control will not be able to go lower
#define MINIMUM_GAIN 0.1

// for exponential gain landing, gain increase per second during the first (hover) phase:
#define INCREASE_GAIN_PER_SECOND 0.10

// variables retained between module calls

// horizontal loop:
float optical_flow_x;
float optical_flow_y;
float lp_flow_x;
float lp_flow_y;
float sum_roll_error;
float sum_pitch_error;

//float divergence_vision;
float divergence_vision_dt;
float normalized_thrust;
//float cov_div;
//float pstate, pused;
float pused;
float istate;
float dstate;
float vision_time,  prev_vision_time;
bool landing;
float previous_cov_err;
int32_t thrust_set;
float divergence_setpoint;

// *********************************
// include and define stuff for SSL:
// *********************************
#define RECURSIVE_LEARNING 0
#include <stdio.h>
#include "modules/computer_vision/textons.h"
// it should now be set to 10 to match the number of textons on the stereoboard... this is extremely ugly.
#define n_ts 10
float texton_distribution_stereoboard[n_ts];
// On Bebop 2:
#define TEXTON_DISTRIBUTION_PATH /data/ftp/internal_000
// for RLS, recursive least squares:
float **P_RLS;
// forgetting factor:
float lambda;
static FILE *distribution_logger = NULL;
static FILE *weights_file = NULL;
unsigned int n_read_samples;
// paparazzi files for doing svd etc.:
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_simple_matrix.h"
float module_active_time_sec;
float module_enter_time;
float previous_divergence_setpoint;

// for ramping
int ramp;
float delta_setpoint;
float ramp_start_time;
float start_setpoint_ramp;

// for the exponentially decreasing gain strategy:
int32_t elc_phase;
uint32_t elc_time_start;
float elc_p_gain_start, elc_i_gain_start,  elc_d_gain_start;
int32_t count_covdiv;
float lp_cov_div;

static abi_event agl_ev; ///< The altitude ABI event
static abi_event optical_flow_ev;

// struct containing most relevant parameters
struct OpticalFlowLanding of_landing_ctrl;

// sending the divergence message to the ground station:
static void send_divergence(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIVERGENCE(trans, dev, AC_ID,
                           &(of_landing_ctrl.divergence), &divergence_vision_dt, &normalized_thrust,
                           &cov_div, &pstate, &pused, &(of_landing_ctrl.agl));
}

/// Function definitions
/// Callback function of the ground altitude
void vertical_ctrl_agl_cb(uint8_t sender_id, uint32_t stamp, float distance);
// Callback function of the optical flow estimate:
void vertical_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x,
                                   int32_t flow_y, int32_t flow_der_x, int32_t flow_der_y, float quality, float size_divergence);

// common functions for different landing strategies:
static void set_cov_div(int32_t thrust);
static int32_t PID_divergence_control(float divergence_setpoint, float P, float I, float D, float dt);
static void update_errors(float error, float dt);
static uint32_t final_landing_procedure(void);

// resetting all variables to be called for instance when starting up / re-entering module
static void reset_all_vars(void);

float thrust_history[OFL_COV_WINDOW_SIZE];
float divergence_history[OFL_COV_WINDOW_SIZE];
float past_divergence_history[OFL_COV_WINDOW_SIZE];
uint32_t ind_hist;
uint8_t cov_array_filled;

void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool in_flight);

/**
 * Initialize the optical flow landing module
 */
void vertical_ctrl_module_init(void)
{
  // filling the of_landing_ctrl struct with default values:
  of_landing_ctrl.use_bias = true;
  of_landing_ctrl.agl = 0.0f;
  of_landing_ctrl.agl_lp = 0.0f;
  of_landing_ctrl.vel = 0.0f;
  of_landing_ctrl.divergence_setpoint = 0.0f; // For exponential gain landing, pick a negative value
  of_landing_ctrl.cov_set_point = OFL_COV_SETPOINT;
  of_landing_ctrl.cov_limit = fabsf(OFL_COV_LANDING_LIMIT);
  of_landing_ctrl.lp_const = OFL_LP_CONST;
  Bound(of_landing_ctrl.lp_const, 0.001f, 1.f);
  of_landing_ctrl.pgain = OFL_PGAIN;
  of_landing_ctrl.igain = OFL_IGAIN;
  of_landing_ctrl.dgain = OFL_DGAIN;

  of_landing_ctrl.pgain_horizontal_factor = OFL_PGAIN_HORIZONTAL_FACTOR;
  of_landing_ctrl.igain_horizontal_factor = OFL_IGAIN_HORIZONTAL_FACTOR;
  of_landing_ctrl.pitch_trim = OFL_PITCH_TRIM;
  of_landing_ctrl.roll_trim = OFL_ROLL_TRIM;

  of_landing_ctrl.divergence = 0.;
  of_landing_ctrl.previous_err = 0.;
  of_landing_ctrl.sum_err = 0.0f;
  of_landing_ctrl.d_err = 0.0f;
  of_landing_ctrl.nominal_thrust = (float)guidance_v.nominal_throttle / MAX_PPRZ; // copy this value from guidance
  of_landing_ctrl.VISION_METHOD = OFL_VISION_METHOD;
  of_landing_ctrl.CONTROL_METHOD = OFL_CONTROL_METHOD;
  of_landing_ctrl.COV_METHOD = OFL_COV_METHOD;
  of_landing_ctrl.delay_steps = 15;
  of_landing_ctrl.window_size = OFL_COV_WINDOW_SIZE;
  of_landing_ctrl.pgain_adaptive = OFL_PGAIN_ADAPTIVE;
  of_landing_ctrl.igain_adaptive = OFL_IGAIN_ADAPTIVE;
  of_landing_ctrl.dgain_adaptive = OFL_DGAIN_ADAPTIVE;
  of_landing_ctrl.reduction_factor_elc =
    0.25f; // TODO: it used to be 0.80 for the oscillation landings... make a separate factor for the predictions.
  // for exponential gain landing, after detecting oscillations, the gain is multiplied with this factor
  of_landing_ctrl.lp_cov_div_factor =
    0.99f; // low pass filtering cov div so that the drone is really oscillating when triggering the descent
  of_landing_ctrl.t_transition = 2.f;
  // if the gain reaches this value during an exponential landing, the drone makes the final landing.
  of_landing_ctrl.p_land_threshold = OFL_P_LAND_THRESHOLD;
  of_landing_ctrl.elc_oscillate = OFL_ELC_OSCILLATE;
  of_landing_ctrl.close_to_edge = OFL_CLOSE_TO_EDGE;
  of_landing_ctrl.lp_factor_prediction = 0.95;

  of_landing_ctrl.omega_FB = OFL_OMEGA_FB;
  of_landing_ctrl.omega_LR = OFL_OMEGA_LR;
  of_landing_ctrl.active_motion = OFL_ACTIVE_MOTION;

  int i;
  if (of_landing_ctrl.use_bias) {
    weights = (float *)calloc(n_textons + 1, sizeof(float));
    for (i = 0; i <= n_textons; i++) {
      weights[i] = 0.0f;
    }
  } else {
    weights = (float *)calloc(n_textons, sizeof(float));
    for (i = 0; i < n_textons; i++) {
      weights[i] = 0.0f;
    }
  }


#ifdef RLS_TEXTONS
  // RLS:
  P_RLS = (float **)calloc((n_textons + 1), sizeof(float *));
  for (i = 0; i < n_textons + 1; i++) {
    P_RLS[i] = (float *)calloc((n_textons + 1), sizeof(float));
  }
  int j;
  for (i = 0; i < n_textons + 1; i++) {
    for (j = 0; j < n_textons + 1; j++) {
      if (i == j) {
        P_RLS[i][j] = 1.0f;
      } else {
        P_RLS[i][j] = 0.0f;
      }
    }
  }
#endif

  reset_all_vars();

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OFL_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);
  // Subscribe to the optical flow estimator:
  // register telemetry:
  AbiBindMsgOPTICAL_FLOW(OFL_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DIVERGENCE, send_divergence);

  module_enter_time = get_sys_time_float();

  of_landing_ctrl.ramp_duration = 0.50f; // ramp duration in seconds
  // ramping:
  ramp = 0;
  delta_setpoint = 0.0f;
  ramp_start_time = 0.0f;
  start_setpoint_ramp = 0.0f;

  lp_flow_x = 0.0f;
  lp_flow_y = 0.0f;
}

/**
 * Reset all variables:
 */
static void reset_all_vars(void)
{
  optical_flow_x = 0.0;
  optical_flow_y = 0.0;
  sum_roll_error = 0.0;
  sum_pitch_error = 0.0;

  of_landing_ctrl.agl_lp = of_landing_ctrl.agl = stateGetPositionEnu_f()->z;

  thrust_set = of_landing_ctrl.nominal_thrust * MAX_PPRZ;

  cov_div = 0.;
  normalized_thrust = of_landing_ctrl.nominal_thrust * 100;
  previous_cov_err = 0.;
  divergence_vision = 0.;
  divergence_vision_dt = 0.;
  divergence_setpoint = 0;
  previous_divergence_setpoint = divergence_setpoint;

  vision_time = get_sys_time_float();
  prev_vision_time = vision_time;

  ind_hist = 0;
  cov_array_filled = 0;
  uint32_t i;
  for (i = 0; i < OFL_COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }

  landing = false;

  elc_phase = 0;
  elc_time_start = 0;
  count_covdiv = 0;
  lp_cov_div = 0.0f;

  pstate = of_landing_ctrl.pgain;
  pused = pstate;
  istate = of_landing_ctrl.igain;
  dstate = of_landing_ctrl.dgain;

  of_landing_ctrl.load_weights = true;
  of_landing_ctrl.divergence = 0.;
  of_landing_ctrl.previous_err = 0.;
  of_landing_ctrl.sum_err = 0.;
  of_landing_ctrl.d_err = 0.;

  // ramping:
  ramp = 0;
  delta_setpoint = 0.0f;
  ramp_start_time = 0.0f;
  start_setpoint_ramp = 0.0f;

  lp_flow_x = 0.0f;
  lp_flow_y = 0.0f;
}

/**
 * Run the optical flow landing module
 */
void vertical_ctrl_module_run(bool in_flight)
{
  float div_factor; // factor that maps divergence in pixels as received from vision to 1 / frame

  float dt = vision_time - prev_vision_time;

  // check if new measurement received
  if (dt <= 1e-5f) {
    return;
  }

  module_active_time_sec = get_sys_time_float() - module_enter_time;

  Bound(of_landing_ctrl.lp_const, 0.001f, 1.f);
  float lp_factor = dt / of_landing_ctrl.lp_const;
  Bound(lp_factor, 0.f, 1.f);

  if (of_landing_ctrl.load_weights) {
    printf("LOADING WEIGHTS!\n");
    load_weights();
    of_landing_ctrl.load_weights = false;
  }

  if (!in_flight) {

    // When not flying and in mode module:
    // Reset integrators, landing phases, etc.
    // reset_all_vars(); // commented out to allow us to study the observation variables in-hand, i.e., without flying


    // SSL: only learn if not flying - due to use of resources:
    if (of_landing_ctrl.learn_gains) {
      printf("LEARNING WEIGHTS!\n");
      float start = get_sys_time_float();
      // learn the weights from the file filled with training examples:
      learn_from_file();

      float end = get_sys_time_float();
      printf("Time to learn: %f\n", end - start);
      // reset the learn_gains variable to false:
      of_landing_ctrl.learn_gains = false;
      // dt is smaller than it actually should be...
    }
  }

  /***********
   * VISION
   ***********/
  float new_flow_x = 0.0;
  float new_flow_y = 0.0;
  if (of_landing_ctrl.VISION_METHOD == 0) {
    // SIMULATED DIVERGENCE:

    // USE OPTITRACK HEIGHT
    of_landing_ctrl.agl = stateGetPositionEnu_f()->z;

    if (fabsf(of_landing_ctrl.agl - of_landing_ctrl.agl_lp) > 1.0f) {
      // ignore outliers:
      of_landing_ctrl.agl = of_landing_ctrl.agl_lp;
    }
    // calculate the new low-pass height and the velocity
    of_landing_ctrl.agl_lp += (of_landing_ctrl.agl - of_landing_ctrl.agl_lp) * lp_factor;

    // only calculate velocity and divergence if dt is large enough:
    of_landing_ctrl.vel = stateGetSpeedEnu_f()->z;

    // calculate the fake divergence:
    if (of_landing_ctrl.agl_lp > 1e-5f) {
      of_landing_ctrl.divergence = of_landing_ctrl.vel / of_landing_ctrl.agl_lp;
      // TODO: this time scaling should be done in optical flow module
      divergence_vision_dt = (divergence_vision / dt);
      if (fabsf(divergence_vision_dt) > 1e-5f) {
        div_factor = of_landing_ctrl.divergence / divergence_vision_dt;
      }
    }
  } else {
    // USE REAL VISION OUTPUTS:
    // TODO: this div_factor depends on the subpixel-factor (automatically adapt?)
    // TODO: this factor is camera specific and should be implemented in the optical
    // flow calculator module not here. Additionally, the time scaling should also
    // be done in the calculator module
    // div_factor = -1.28f; // magic number comprising field of view etc.
    div_factor = -2.25f; // in the sim
    float new_divergence = (divergence_vision * div_factor) / dt;
    new_flow_x = optical_flow_x / dt;
    new_flow_y = optical_flow_y / dt;

    // deal with (unlikely) fast changes in divergence:
    static const float max_div_dt = 0.20f;
    if (fabsf(new_divergence - of_landing_ctrl.divergence) > max_div_dt) {
      if (new_divergence < of_landing_ctrl.divergence) { new_divergence = of_landing_ctrl.divergence - max_div_dt; }
      else { new_divergence = of_landing_ctrl.divergence + max_div_dt; }
    }

    // low-pass filter the divergence:
    of_landing_ctrl.divergence += (new_divergence - of_landing_ctrl.divergence) * lp_factor;
    prev_vision_time = vision_time;
  }

  /***********
  * CONTROL
  ***********/
  // landing indicates whether the drone is already performing a final landing procedure (flare):
  if (!landing) {

    /*
    // First seconds, don't do anything crazy:
    if (module_active_time_sec < 2.5f) {
      int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
      thrust_set = nominal_throttle;
      stabilization_cmd[COMMAND_THRUST] = thrust;
      // keep track of histories and set the covariance
      set_cov_div(thrust_set);
      cov_div = 0.0f; // however, cov div is set to 0 to prevent strange issues at module startup.
      return;
    }
    */

    if (of_landing_ctrl.CONTROL_METHOD == 0) {
      // FIXED GAIN CONTROL, cov_limit for landing:

      // use the divergence for control:
      thrust_set = PID_divergence_control(of_landing_ctrl.divergence_setpoint, of_landing_ctrl.pgain, of_landing_ctrl.igain,
                                          of_landing_ctrl.dgain, dt);

      // trigger the landing if the cov div is too high:
      if (fabsf(cov_div) > of_landing_ctrl.cov_limit) {
        thrust_set = final_landing_procedure();
      }
    } else if (of_landing_ctrl.CONTROL_METHOD == 1) {
      // ADAPTIVE GAIN CONTROL:
      // TODO: i-gain and d-gain are currently not adapted

      // adapt the gains according to the error in covariance:
      float error_cov = of_landing_ctrl.cov_set_point - cov_div;
      // limit the error_cov, which could else become very large:
      if (error_cov > fabsf(of_landing_ctrl.cov_set_point)) { error_cov = fabsf(of_landing_ctrl.cov_set_point); }
      pstate -= (of_landing_ctrl.igain_adaptive * pstate) * error_cov;
      if (pstate < MINIMUM_GAIN) { pstate = MINIMUM_GAIN; }
      pused = pstate - (of_landing_ctrl.pgain_adaptive * pstate) * error_cov;
      // make sure pused does not become too small, nor grows too fast:
      if (pused < MINIMUM_GAIN) { pused = MINIMUM_GAIN; }
      if (of_landing_ctrl.COV_METHOD == 1 && error_cov > 0.001) {
        pused = 0.5 * pused;
      }

      // use the divergence for control:
      thrust_set = PID_divergence_control(of_landing_ctrl.divergence_setpoint, pused, of_landing_ctrl.igain,
                                          of_landing_ctrl.dgain, dt);

      // SSL: if close enough, store texton inputs:
      printf("abs error cov = %f, close = %f\n", fabs(error_cov), of_landing_ctrl.close_to_edge);
      if (fabs(error_cov) < of_landing_ctrl.close_to_edge) {
        save_texton_distribution();
      }

      // when to make the final landing:
      if (pstate < of_landing_ctrl.p_land_threshold) {
        thrust_set = final_landing_procedure();
      }

    } else if (of_landing_ctrl.CONTROL_METHOD == 2) {
      // EXPONENTIAL GAIN CONTROL:
      static const float phase_0_set_point = 0.0f;
      if (elc_phase == 0) {
        // increase the gain till you start oscillating:

        // if not yet oscillating, increase the gains:
        if (of_landing_ctrl.elc_oscillate && cov_div > of_landing_ctrl.cov_set_point) {
          pstate += dt * INCREASE_GAIN_PER_SECOND;
          float gain_factor = pstate / pused;
          istate *= gain_factor;
          dstate *= gain_factor;
          pused = pstate;
        }

        // use the divergence for control:
        thrust_set = PID_divergence_control(phase_0_set_point, pused, istate, dstate, dt);

        // low pass filter cov div and remove outliers:
        if (fabsf(lp_cov_div - cov_div) < of_landing_ctrl.cov_limit) {
          lp_cov_div = of_landing_ctrl.lp_cov_div_factor * lp_cov_div + (1 - of_landing_ctrl.lp_cov_div_factor) * cov_div;
        }
        // if oscillating, maintain a counter to see if it endures:
        if (lp_cov_div <= of_landing_ctrl.cov_set_point) {
          count_covdiv++;
        } else {
          count_covdiv = 0;
          elc_time_start = get_sys_time_float();
        }
        // if the drone has been oscillating long enough, start landing:
        if (!of_landing_ctrl.elc_oscillate ||
            (count_covdiv > 0 && (get_sys_time_float() - elc_time_start) >= of_landing_ctrl.t_transition)) {
          // next phase:
          elc_phase = 1;
          elc_time_start = get_sys_time_float();

          // we don't want to oscillate, so reduce the gain:
          elc_p_gain_start = of_landing_ctrl.reduction_factor_elc * pstate;
          elc_i_gain_start = of_landing_ctrl.reduction_factor_elc * istate;
          elc_d_gain_start = of_landing_ctrl.reduction_factor_elc * dstate;
          count_covdiv = 0;
          of_landing_ctrl.sum_err = 0.0f;
        }
      } else if (elc_phase == 1) {
        // control divergence to 0 with the reduced gain:
        pstate = elc_p_gain_start;
        pused = pstate;
        istate = elc_i_gain_start;
        dstate = elc_d_gain_start;

        float t_interval = get_sys_time_float() - elc_time_start;
        // this should not happen, but just to be sure to prevent too high gain values:
        if (t_interval < 0) { t_interval = 0.0f; }

        // use the divergence for control:
        thrust_set = PID_divergence_control(phase_0_set_point, pused, istate, dstate, dt);

        // if we have been trying to hover stably again for 2 seconds and we move in the same way as the desired divergence, switch to landing:
        if (t_interval >= 2.0f && of_landing_ctrl.divergence * of_landing_ctrl.divergence_setpoint >= 0.0f) {
          // next phase:
          elc_phase = 2;
          elc_time_start = get_sys_time_float();
          count_covdiv = 0;
        }
      } else if (elc_phase == 2) {
        // land while exponentially decreasing the gain:
        float t_interval = get_sys_time_float() - elc_time_start;

        // this should not happen, but just to be sure to prevent too high gain values:
        if (t_interval < 0) { t_interval = 0.0f; }

        // determine the P-gain, exponentially decaying:
        float gain_scaling = expf(of_landing_ctrl.divergence_setpoint * t_interval);
        pstate = elc_p_gain_start * gain_scaling;
        istate = elc_i_gain_start * gain_scaling;
        dstate = elc_d_gain_start * gain_scaling;
        pused = pstate;

        // 2 [1/s] ramp to setpoint
        /*if (fabsf(of_landing_ctrl.divergence_setpoint - divergence_setpoint) > 2.*dt){
          divergence_setpoint += 2*dt * of_landing_ctrl.divergence_setpoint / fabsf(of_landing_ctrl.divergence_setpoint);
        } else {
          divergence_setpoint = of_landing_ctrl.divergence_setpoint;
        }*/

        // use the divergence for control:
        thrust_set = PID_divergence_control(of_landing_ctrl.divergence_setpoint, pused, istate, dstate, dt);

        // when to make the final landing:
        if (pstate < of_landing_ctrl.p_land_threshold) {
          elc_phase = 3;
        }
      } else {
        thrust_set = final_landing_procedure();
      }
    } else if (of_landing_ctrl.CONTROL_METHOD == 3) {

      // SSL LANDING: use learned weights for setting the gain on the way down:

      // adapt the p-gain with a low-pass filter to the gain predicted by image appearance:
      pstate = predict_gain(texton_distribution);
      // low-pass filtered, downscaled pstate predictions:
      // TODO: just as with divergence, also take dt into account for low-pass filtering:
      of_landing_ctrl.pgain = of_landing_ctrl.lp_factor_prediction * of_landing_ctrl.pgain +
                              (1.0f - of_landing_ctrl.lp_factor_prediction) * of_landing_ctrl.reduction_factor_elc * pstate;
      pused = of_landing_ctrl.pgain;
      // make sure pused does not become too small, nor grows too fast:
      if (of_landing_ctrl.pgain < MINIMUM_GAIN) { of_landing_ctrl.pgain = MINIMUM_GAIN; }
      // have the i and d gain depend on the p gain:
      istate = 0.025 * of_landing_ctrl.pgain;
      dstate = 0.0f;
      printf("of_landing_ctrl.pgain = %f, divergence = %f, phase = %d\n", of_landing_ctrl.pgain, of_landing_ctrl.divergence,
             elc_phase);

      if (elc_phase == 0) {
        // Let the low-pass filter settle first with 0 divergence:
        float phase_0_set_point = 0.0f;
        previous_divergence_setpoint = 0.0f;

        // use the divergence for control:
        thrust_set = PID_divergence_control(phase_0_set_point, pused, istate, dstate, dt);

        // if the low-pass filter of the p-gain has settled and the drone is moving in the right direction:
        if (module_active_time_sec >= 3.0f && of_landing_ctrl.divergence * of_landing_ctrl.divergence_setpoint >= 0.0f) {
          // next phase:
          elc_phase = 1;
        }
      } else {
        // use the chosen divergence setpoint and the measured divergence for control:
        thrust_set = PID_divergence_control(of_landing_ctrl.divergence_setpoint, pused, istate, dstate, dt);
      }
      if (pused < of_landing_ctrl.p_land_threshold) {
        thrust_set = final_landing_procedure();
      }
    } else  if (of_landing_ctrl.CONTROL_METHOD == 4) {

      // Mixed SSL + EXPONENTIAL LANDING: use learned weights for setting the gain on the way down:

      if (elc_phase == 0) {

        float phase_0_set_point = 0.0f;
        previous_divergence_setpoint = 0.0f;
        // adapt the p-gain with a low-pass filter to the gain predicted by image appearance:
        pstate = predict_gain(texton_distribution);
        // of_landing_ctrl.pgain = lp_factor_prediction * of_landing_ctrl.pgain + (1.0f - lp_factor_prediction) * of_landing_ctrl.stable_gain_factor * pstate;
        of_landing_ctrl.pgain = of_landing_ctrl.lp_factor_prediction * of_landing_ctrl.pgain +
                                (1.0f - of_landing_ctrl.lp_factor_prediction) * of_landing_ctrl.reduction_factor_elc * pstate;
        pused = of_landing_ctrl.pgain;
        // make sure pused does not become too small, nor grows too fast:
        if (of_landing_ctrl.pgain < MINIMUM_GAIN) { of_landing_ctrl.pgain = MINIMUM_GAIN; }
        // have the i and d gain depend on the p gain:
        istate = 0.025 * of_landing_ctrl.pgain;
        dstate = 0.0f;
        printf("of_landing_ctrl.pgain = %f, divergence = %f\n", of_landing_ctrl.pgain, of_landing_ctrl.divergence);

        // use the divergence for control:
        thrust_set = PID_divergence_control(phase_0_set_point, pused, istate, dstate, dt);

        // if the low-pass filter of the p-gain has settled and the drone is moving in the right direction:
        if (module_active_time_sec >= 3.0f && of_landing_ctrl.divergence * of_landing_ctrl.divergence_setpoint >= 0.0f) {
          // next phase:
          elc_phase = 1;
          elc_time_start = get_sys_time_float();

          // we don't have to reduce the gain with of_landing_ctrl.reduction_factor_elc, as this is done above for the p-gain already:
          elc_p_gain_start = pused;
          elc_i_gain_start = istate;
          elc_d_gain_start = dstate;
          count_covdiv = 0;
          of_landing_ctrl.sum_err = 0.0f;
        }
      } else if (elc_phase == 1) {
        // land while exponentially decreasing the gain:
        float new_time = get_sys_time_float();
        float t_interval = new_time - elc_time_start;

        // this should not happen, but just to be sure to prevent too high gain values:
        if (t_interval < 0) { t_interval = 0.0f; }

        // determine the P-gain, weighing between an exponentially decaying one and the predicted one based on textons:
        float gain_scaling = exp(of_landing_ctrl.divergence_setpoint * t_interval);
        float prediction = predict_gain(texton_distribution); // low-pass it in of_landing_ctrl.pgain?
        float weight_prediction = 0.5;

        if (gain_scaling <= 1.0f) {
          pstate = weight_prediction * of_landing_ctrl.reduction_factor_elc * prediction + (1 - weight_prediction) *
                   elc_p_gain_start * gain_scaling;
          istate = 0.025 * weight_prediction * of_landing_ctrl.reduction_factor_elc * prediction +
                   (1 - weight_prediction) * elc_i_gain_start * gain_scaling;
          dstate = 0.0f;
        } else {
          // should never come here:
          pstate = of_landing_ctrl.reduction_factor_elc * prediction;
          istate = 0.025 * of_landing_ctrl.reduction_factor_elc * prediction;
          dstate = 0.0f;
        }
        pused = pstate;

        // use the divergence for control:
        thrust_set = PID_divergence_control(of_landing_ctrl.divergence_setpoint, pused, istate, dstate, dt);

        // when to make the final landing:
        if (pstate < of_landing_ctrl.p_land_threshold) {
          elc_phase = 2;
        }
      } else {
        thrust_set = final_landing_procedure();
      }
    } else  if (of_landing_ctrl.CONTROL_METHOD == 5) {

      // SSL raw, no landing procedure, ideal for hovering:

      pstate = predict_gain(texton_distribution);
      // printf("prediction = %f\n", pstate);
      of_landing_ctrl.pgain = of_landing_ctrl.lp_factor_prediction * of_landing_ctrl.pgain +
                              (1.0f - of_landing_ctrl.lp_factor_prediction) * of_landing_ctrl.reduction_factor_elc * pstate;
      pused = of_landing_ctrl.pgain;
      // make sure pused does not become too small, nor grows too fast:
      if (of_landing_ctrl.pgain < MINIMUM_GAIN) { of_landing_ctrl.pgain = MINIMUM_GAIN; }
      // have the i and d gain depend on the p gain:
      istate = 0.025 * of_landing_ctrl.pgain;
      dstate = 0.0f;
      printf("of_landing_ctrl.pgain = %f\n", of_landing_ctrl.pgain);

      // use the divergence for control:
      thrust_set = PID_divergence_control(of_landing_ctrl.divergence_setpoint, pused, istate, dstate, dt);

      if (pstate < of_landing_ctrl.p_land_threshold) {
        thrust_set = final_landing_procedure();
      }
    }

    if (in_flight) {
      Bound(thrust_set, 0.25 * of_landing_ctrl.nominal_thrust * MAX_PPRZ, MAX_PPRZ);
      stabilization_cmd[COMMAND_THRUST] = thrust_set;
    }
  }

  /*********************/
  // Horizontal control:
  /*********************/

  struct FloatEulers *attitude = stateGetNedToBodyEulers_f();

  // negative command is flying forward, positive back.
  lp_flow_x = of_landing_ctrl.lp_factor_prediction * lp_flow_x + (1.0f - of_landing_ctrl.lp_factor_prediction) *
              optical_flow_x;
  lp_flow_y = of_landing_ctrl.lp_factor_prediction * lp_flow_y + (1.0f - of_landing_ctrl.lp_factor_prediction) *
              optical_flow_y;


  if (of_landing_ctrl.active_motion == 1) {
    // Active motion through varying ventral flow commands
    float period_factor = 0.2;
    float sine_threshold = 0.75;
    float omega_set_point = 20;

    if (sinf(period_factor * module_active_time_sec) > sine_threshold) {
      of_landing_ctrl.omega_LR = omega_set_point;
    } else if (sinf(period_factor * module_active_time_sec) < -sine_threshold) {
      of_landing_ctrl.omega_LR = -omega_set_point;
    } else {
      of_landing_ctrl.omega_LR = 0.0f;
    }
  } else if (of_landing_ctrl.active_motion == 2) {
    // Active motion through varying roll trim commands
    float period_factor = 0.2;
    float sine_threshold = 0.9;
    float trim_set_point = 2.0f;

    if (sinf(period_factor * module_active_time_sec) > sine_threshold) {
      of_landing_ctrl.roll_trim = trim_set_point;
    } else if (sinf(period_factor * module_active_time_sec) < -sine_threshold) {
      of_landing_ctrl.roll_trim = -trim_set_point;
    } else {
      of_landing_ctrl.roll_trim = 0.0f;
    }
  }

  float error_pitch = new_flow_y - of_landing_ctrl.omega_FB;
  float error_roll = new_flow_x - of_landing_ctrl.omega_LR;
  sum_pitch_error += error_pitch;
  sum_roll_error += error_roll;
  float P_hor = of_landing_ctrl.pgain_horizontal_factor * of_landing_ctrl.pgain;
  float I_hor = of_landing_ctrl.igain_horizontal_factor * of_landing_ctrl.igain;
  float pitch_cmd;
  float roll_cmd;
  float add_active_sine = 0.0f;

  if (ACTIVE_RATES) {
    float sine_period = 0.05;
    float sine_amplitude = RadOfDeg(1.0);
    float sine_time = get_sys_time_float();
    add_active_sine = sine_amplitude * sinf((1.0f / sine_period) * sine_time * 2 * M_PI);
    printf("add_active_sine = %f\n", add_active_sine);
  }
  if (!HORIZONTAL_RATE_CONTROL) {
    // normal operation:
    pitch_cmd = RadOfDeg(of_landing_ctrl.pitch_trim + error_pitch *  P_hor +  I_hor * sum_pitch_error);
    // add_active_sine = 0.0f in normal operation:
    roll_cmd = RadOfDeg(of_landing_ctrl.roll_trim + error_roll *  P_hor +  I_hor * sum_roll_error) + add_active_sine;
  }
  BoundAbs(pitch_cmd, RadOfDeg(10.0));
  BoundAbs(roll_cmd, RadOfDeg(10.0));
  float psi_cmd =
    attitude->psi; // control of psi in simulation is still a bit enthusiastic! Increase the associated control effectiveness.
  struct Int32Eulers rpy = { .phi = (int32_t)ANGLE_BFP_OF_REAL(roll_cmd),
           .theta = (int32_t)ANGLE_BFP_OF_REAL(pitch_cmd), .psi = (int32_t)ANGLE_BFP_OF_REAL(psi_cmd)
  };

  // set the desired roll pitch and yaw:
  stabilization_indi_set_rpy_setpoint_i(&rpy);
  // execute attitude stabilization:
  stabilization_attitude_run(in_flight);
}

/**
 * Execute a final landing procedure
 */
uint32_t final_landing_procedure()
{
  // land with 85% nominal thrust:
  uint32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
  uint32_t thrust = 0.85 * nominal_throttle;
  Bound(thrust, 0.6 * nominal_throttle, 0.9 * MAX_PPRZ);
  landing = true;

  return thrust;
}

/**
 * Set the covariance of the divergence and the thrust / past divergence
 * This funciton should only be called once per time step
 * @param[in] thrust: the current thrust value
 */
void set_cov_div(int32_t thrust)
{
  // histories and cov detection:
  divergence_history[ind_hist] = of_landing_ctrl.divergence;

  normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
  thrust_history[ind_hist] = normalized_thrust;

  int ind_past = ind_hist - of_landing_ctrl.delay_steps;
  while (ind_past < 0) { ind_past += of_landing_ctrl.window_size; }
  past_divergence_history[ind_hist] = divergence_history[ind_past];

  // determine the covariance for landing detection:
  // only take covariance into account if there are enough samples in the histories:
  if (of_landing_ctrl.COV_METHOD == 0 && cov_array_filled > 0) {
    // TODO: step in landing set point causes an incorrectly perceived covariance
    cov_div = covariance_f(thrust_history, divergence_history, of_landing_ctrl.window_size);
  } else if (of_landing_ctrl.COV_METHOD == 1 && cov_array_filled > 1) {
    // todo: delay steps should be invariant to the run frequency
    cov_div = covariance_f(past_divergence_history, divergence_history, of_landing_ctrl.window_size);
  }

  if (cov_array_filled < 2 && ind_hist + 1 == of_landing_ctrl.window_size) {
    cov_array_filled++;
  }
  ind_hist = (ind_hist + 1) % of_landing_ctrl.window_size;
}

/**
 * Determine and set the thrust for constant divergence control
 * @param[out] thrust
 * @param[in] divergence_set_point: The desired divergence
 * @param[in] P: P-gain
 * @param[in] I: I-gain
 * @param[in] D: D-gain
 * @param[in] dt: time difference since last update
 */
int32_t PID_divergence_control(float setpoint, float P, float I, float D, float dt)
{

  if (previous_divergence_setpoint != setpoint) {
    ramp = 1;
    delta_setpoint = setpoint - previous_divergence_setpoint;
    start_setpoint_ramp = previous_divergence_setpoint;
    ramp_start_time = get_sys_time_float();
  }

  // determine the error, possibly using a ramp:
  float err;
  if (!ramp) {
    err = setpoint - of_landing_ctrl.divergence;
  } else {
    // rt is in [0,1] and used to make the ramp:
    float ramp_time = get_sys_time_float();
    float rt = (ramp_time - ramp_start_time) / (of_landing_ctrl.ramp_duration);

    if (rt > 1.0f) {
      ramp = 0; // end of the ramp
      err = setpoint - of_landing_ctrl.divergence;
      // Send the setpoint back via the divergence message, so that we can see that this procedure works
      divergence_vision_dt = setpoint;
    } else {
      float ds = start_setpoint_ramp + rt * delta_setpoint;
      // Send the setpoint back via the divergence message, so that we can see that this procedure works
      divergence_vision_dt = ds;
      // determine the error:
      err = ds - of_landing_ctrl.divergence;
    }
  }

  // update the controller errors:
  update_errors(err, dt);

  // PID control:
  int32_t thrust = (of_landing_ctrl.nominal_thrust
                    + P * err
                    + I * of_landing_ctrl.sum_err
                    + D * of_landing_ctrl.d_err) * MAX_PPRZ;

  // bound thrust:
  Bound(thrust, 0.25 * of_landing_ctrl.nominal_thrust * MAX_PPRZ, MAX_PPRZ);

  // update covariance
  set_cov_div(thrust);

  previous_divergence_setpoint = setpoint;

  return thrust;
}

/**
 * Updates the integral and differential errors for PID control and sets the previous error
 * @param[in] err: the error of the divergence and divergence setpoint
 * @param[in] dt:  time difference since last update
 */
void update_errors(float err, float dt)
{
  float lp_factor = dt / of_landing_ctrl.lp_const;
  Bound(lp_factor, 0.f, 1.f);

  // maintain the controller errors:
  of_landing_ctrl.sum_err += err;
  of_landing_ctrl.d_err += (((err - of_landing_ctrl.previous_err) / dt) - of_landing_ctrl.d_err) * lp_factor;
  of_landing_ctrl.previous_err = err;
}

// Reading from "sensors":
void vertical_ctrl_agl_cb(uint8_t sender_id UNUSED, __attribute__((unused)) uint32_t stamp, float distance)
{
  of_landing_ctrl.agl = distance;
}

void vertical_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp,
                                   int32_t flow_x UNUSED, int32_t flow_y UNUSED,
                                   int32_t flow_der_x UNUSED, int32_t flow_der_y UNUSED,
                                   float quality UNUSED, float size_divergence)
{

  if (sender_id == FLOW_OPTICFLOW_ID + 0) { // 0 = bottom 1 = front
    optical_flow_x = ((float)flow_der_x) / 10.0;
    optical_flow_y = ((float)flow_der_y) / 10.0;
    divergence_vision = size_divergence;
    //printf("Reading %f, %f, %f\n", optical_flow_x, optical_flow_y, divergence_vision);
    vision_time = ((float)stamp) / 1e6;
  }
}

////////////////////////////////////////////////////////////////////
// Call our controller

void guidance_h_module_init(void)
{

}

void guidance_h_module_enter(void)
{

}

void guidance_h_module_run(bool in_flight)
{

}

void guidance_h_module_read_rc(void)
{

}

void guidance_v_module_init(void)
{
  vertical_ctrl_module_init();
}

/**
 * Entering the module (user switched to module)
 */
void guidance_v_module_enter(void)
{
  reset_all_vars();

  // adaptive estimation - assume hover condition when entering the module
  of_landing_ctrl.nominal_thrust = (float) stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
  thrust_set = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
}

void guidance_v_module_run(bool in_flight)
{
  vertical_ctrl_module_run(in_flight);
}

// SSL:
void save_texton_distribution(void)
{
  printf("Logging textons!\n");
  int i;

  // If not the same, append the target values (heights, gains) and texton values to a .dat file:
  char filename[512];
  sprintf(filename, "%s/Training_set_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  distribution_logger = fopen(filename, "a");
  if (distribution_logger == NULL) {
    perror(filename);
    //perror("Error while opening the file.\n");
  } else {
    printf("Logging at height %f, gain %f, cov_div %f\n", of_landing_ctrl.agl, pstate, cov_div);

    // save the information in a single row:
    fprintf(distribution_logger, "%f ", of_landing_ctrl.agl); // sonar measurement
    fprintf(distribution_logger, "%f ", pstate); // gain measurement
    fprintf(distribution_logger, "%f ", cov_div); // cov div measurement
    for (i = 0; i < n_textons - 1; i++) {
      fprintf(distribution_logger, "%f ", texton_distribution[i]);
    }
    fprintf(distribution_logger, "%f\n", texton_distribution[n_textons - 1]);
    fclose(distribution_logger);
  }
}

void load_texton_distribution(void)
{
  int i, j, read_result;
  char filename[512];

  sprintf(filename, "%s/Training_set_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  printf("Loading textons from %s\n", filename);

  distribution_logger = fopen(filename, "r");
  if (distribution_logger) {
    // Load the dictionary:
    n_read_samples = 0;
    // For now we read the samples sequentially:
    for (i = 0; i < MAX_SAMPLES_LEARNING; i++)
      //for(i = 0; i < 30; i++)
    {
      read_result = fscanf(distribution_logger, "%f ", &sonar_OF[n_read_samples]);
      if (read_result == EOF) { break; }
      read_result = fscanf(distribution_logger, "%f ", &gains[n_read_samples]);
      if (read_result == EOF) { break; }
      read_result = fscanf(distribution_logger, "%f ", &cov_divs_log[n_read_samples]);
      if (read_result == EOF) { break; }

      text_dists[n_read_samples] = (float *) calloc(n_textons, sizeof(float));
      for (j = 0; j < n_textons - 1; j++) {
        read_result = fscanf(distribution_logger, "%f ", &text_dists[n_read_samples][j]);
        if (read_result == EOF) { break; }
      }
      read_result = fscanf(distribution_logger, "%f\n", &text_dists[n_read_samples][n_textons - 1]);
      if (read_result == EOF) { break; }
      n_read_samples++;
    }
    fclose(distribution_logger);
  } else {
    printf("There was an error opening %s\n", filename);
  }
}

void learn_from_file(void)
{
  int i;
  float fit_error;

  // first load the texton distributions:
  printf("Loading distribution\n");
  load_texton_distribution();

  // then learn from it:
  printf("Learning!\n");
  if (!RECURSIVE_LEARNING) {
    fit_linear_model_OF(gains, text_dists, n_textons, n_read_samples, weights, &fit_error);
    // fit_linear_model_OF(sonar_OF, text_dists, n_textons, n_read_samples, weights, &fit_error);
  } else {
    recursive_least_squares_batch(gains, text_dists, n_textons, n_read_samples, weights, &fit_error);
  }

  printf("Saving!\n");
  // save the weights to a file:
  save_weights();

  printf("Learned! Fit error = %f\n", fit_error);

  // free learning distributions:
  for (i = 0; i < n_read_samples; i++) {
    free(text_dists[i]);
  }
}

/**
 * Recursively fit a linear model from samples to target values - batch mode, possibly for initialization.
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit // TODO: relevant for RLS?
 */
void recursive_least_squares_batch(float *targets, float **samples, uint8_t D, uint16_t count, float *params,
                                   float *fit_error)
{
  // TODO: potentially randomizing the sequence of the samples, as not to get a bias towards the later samples
  // local vars for iterating, random numbers:
  int sam, d;
  uint8_t D_1 = D + 1;
  float augmented_sample[D_1];
  // augmented sample is used if the bias is used:
  augmented_sample[D] = 1.0f;

  // Initialize the weights with 0.0f:
  for (d = 0; d < D_1; d++) {
    weights[d] = 0.0f;
  }

  // Reset the P matrix to an identity matrix:
  int i, j;
  for (i = 0; i < n_textons + 1; i++) {
    for (j = 0; j < n_textons + 1; j++) {
      if (i == j) {
        P_RLS[i][j] = 1.0f;
      } else {
        P_RLS[i][j] = 0.0f;
      }
    }
  }

  // give the samples one by one to the recursive procedure:
  for (sam = 0; sam < count; sam++) {
    if (!of_landing_ctrl.use_bias) {
      recursive_least_squares(targets[sam], samples[sam], D, params);
    } else {
      for (d = 0; d < D; d++) {
        augmented_sample[d] = samples[sam][d];
      }
      recursive_least_squares(targets[sam], augmented_sample, D_1, params);
    }
  }

  // give the samples one by one to the recursive procedure to determine the fit on the training set:
  float prediction;
  float sum_abs_err = 0;
  for (sam = 0; sam < count; sam++) {
    prediction = predict_gain(samples[sam]);
    sum_abs_err += fabs(prediction - targets[sam]);
  }
  (*fit_error) = sum_abs_err / count;
}

void recursive_least_squares(float target, float *sample, uint8_t length_sample, float *params)
{
  // MATLAB procedure:
  /*
    u = features(i,:);
    phi = u * P ;
          k(:, i) = phi' / (lamda + phi * u' );
    y(i)= weights(:,i)' * u';
    e(i) = targets(i) - y(i) ;
          weights(:,i+1) = weights(:,i) + k(:, i) * e(i) ;
          P = ( P - k(:, i) * phi ) / lamda;
  */

  float _u[1][length_sample];
  float _uT[length_sample][1];
  float _phi[1][length_sample];
  float _phiT[length_sample][1];
  float _k[length_sample][1];
  float _ke[length_sample][1];
  float prediction, error;
  float _u_P_uT[1][1];
  float scalar;
  float _O[length_sample][length_sample];
  int i;

  // u = features(i,:);
  for (i = 0; i < length_sample; i++) {
    _u[0][i] = sample[i];
    _uT[i][0] = sample[i];
  }
  MAKE_MATRIX_PTR(u, _u, 1);
  MAKE_MATRIX_PTR(uT, _uT, length_sample); // transpose
  MAKE_MATRIX_PTR(phi, _phi, 1);
  MAKE_MATRIX_PTR(phiT, _phiT, length_sample);
  MAKE_MATRIX_PTR(u_P_uT, _u_P_uT, 1); // actually a scalar
  MAKE_MATRIX_PTR(k, _k, length_sample);
  MAKE_MATRIX_PTR(ke, _ke, length_sample);

  MAKE_MATRIX_PTR(P, P_RLS, length_sample);
  MAKE_MATRIX_PTR(O, _O, length_sample);
  // phi = u * P ;
  // result of multiplication goes into phi
  MAT_MUL(1, length_sample, length_sample, phi, u, P);
  // k(:, i) = phi' / (lamda + phi * u' );
  for (i = 0; i < length_sample; i++) {
    phiT[i][0] = phi[0][i];
  }
  // scalar:
  MAT_MUL(1, length_sample, 1, u_P_uT, phi, uT);
  scalar = u_P_uT[0][0];
  float_mat_div_scalar(k, phiT, lambda + scalar, length_sample, 1);
  // y(i)= weights(:,i)' * u';
  prediction = predict_gain(sample);
  // e(i) = targets(i) - y(i) ;
  error = target - prediction;
  // weights(:,i+1) = weights(:,i) + k(:, i) * e(i) ;
  float_mat_mul_scalar(ke, k, error, length_sample, 1);
  for (i = 0; i < length_sample; i++) {
    weights[i] += ke[i][0];
  }
  // P = ( P - k(:, i) * phi ) / lamda;
  MAT_MUL(length_sample, 1, length_sample, O, k, phi);
  MAT_SUB(length_sample, length_sample, P, P, O);
  float_mat_div_scalar(P, P, lambda, length_sample, length_sample);
}

/**
 * Fit a linear model from samples to target values.
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit
 */
void fit_linear_model_OF(float *targets, float **samples, uint8_t D, uint16_t count, float *params, float *fit_error)
{

  // We will solve systems of the form A x = b,
  // where A = [nx(D+1)] matrix with entries [s1, ..., sD, 1] for each sample (1 is the bias)
  // and b = [nx1] vector with the target values.
  // x in the system are the parameters for the linear regression function.

  // local vars for iterating, random numbers:
  int sam, d;
  uint16_t n_samples = count;
  uint8_t D_1 = D + 1;
  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < D_1) ? D_1 : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;

  // initialize matrices and vectors for the full point set problem:
  // this is used for determining inliers
  float _AA[count][D_1];
  MAKE_MATRIX_PTR(AA, _AA, count);
  float _targets_all[count][1];
  MAKE_MATRIX_PTR(targets_all, _targets_all, count);

  for (sam = 0; sam < count; sam++) {
    for (d = 0; d < D; d++) {
      AA[sam][d] = samples[sam][d];
    }
    if (of_landing_ctrl.use_bias) {
      AA[sam][D] = 1.0f;
    } else {
      AA[sam][D] = 0.0f;
    }
    targets_all[sam][0] = targets[sam];
  }


  // decompose A in u, w, v with singular value decomposition A = u * w * vT.
  // u replaces A as output:
  float _parameters[D_1][1];
  MAKE_MATRIX_PTR(parameters, _parameters, D_1);
  float w[n_samples], _v[D_1][D_1];
  MAKE_MATRIX_PTR(v, _v, D_1);

  pprz_svd_float(AA, w, v, count, D_1);
  pprz_svd_solve_float(parameters, AA, w, v, targets_all, count, D_1, 1);

  // used to determine the error of a set of parameters on the whole set:
  float _bb[count][1];
  MAKE_MATRIX_PTR(bb, _bb, count);
  float _C[count][1];
  MAKE_MATRIX_PTR(C, _C, count);

  // error is determined on the entire set
  // bb = AA * parameters:
  MAT_MUL(count, D_1, 1, bb, AA, parameters);
  // subtract bu_all: C = 0 in case of perfect fit:
  MAT_SUB(count, 1, C, bb, targets_all);
  *fit_error = 0;
  for (sam = 0; sam < count; sam++) {
    *fit_error += fabs(C[sam][0]);
  }
  *fit_error /= count;

  for (d = 0; d < D_1; d++) {
    params[d] = parameters[d][0];
  }

}


float predict_gain(float *distribution)
{
  //printf("Start prediction!\n");
  int i;
  float sum;

  sum = 0.0f;
  for (i = 0; i < n_textons; i++) {
    sum += weights[i] * distribution[i];
  }
  if (of_landing_ctrl.use_bias) {
    sum += weights[n_textons];
  }
  // printf("Prediction = %f\n", sum);
  return sum;
}

void save_weights(void)
{
  // save the weights to a file:
  int i;
  char filename[512];
  sprintf(filename, "%s/Weights_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  weights_file = fopen(filename, "w");
  if (weights_file == NULL) {
    perror(filename);
  } else {
    // save the information in a single row:
    for (i = 0; i <= n_textons; i++) {
      fprintf(weights_file, "%f ", weights[i]);
    }
    fclose(weights_file);
  }
}

void load_weights(void)
{
  int i, read_result;
  char filename[512];
  sprintf(filename, "%s/Weights_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  weights_file = fopen(filename, "r");
  if (weights_file == NULL) {
    printf("No weights file!\n");
    perror(filename);
  } else {
    // load the weights, stored in a single row:
    for (i = 0; i <= n_textons; i++) {
      read_result = fscanf(weights_file, "%f ", &weights[i]);
      if (read_result == EOF) { break; }
    }
    fclose(weights_file);
  }
}
