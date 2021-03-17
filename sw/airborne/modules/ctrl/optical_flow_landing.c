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
 */

#include "optical_flow_landing.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"

// used for automated landing:
#include "autopilot.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/datalink/telemetry.h"

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

// Constants
// minimum value of the P-gain for divergence control
// adaptive control / exponential gain control will not be able to go lower
#define MINIMUM_GAIN 0.1

// for exponential gain landing, gain increase per second during the first (hover) phase:
#define INCREASE_GAIN_PER_SECOND 0.02

// variables retained between module calls
float divergence_vision;
float divergence_vision_dt;
float normalized_thrust;
float cov_div;
float pstate, pused;
float istate;
float dstate;
float vision_time,  prev_vision_time;
bool landing;
float previous_cov_err;
int32_t thrust_set;
float divergence_setpoint;

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
  of_landing_ctrl.divergence = 0.;
  of_landing_ctrl.previous_err = 0.;
  of_landing_ctrl.sum_err = 0.0f;
  of_landing_ctrl.d_err = 0.0f;
  of_landing_ctrl.nominal_thrust = (float)guidance_v_nominal_throttle / MAX_PPRZ; // copy this value from guidance
  of_landing_ctrl.VISION_METHOD = OFL_VISION_METHOD;
  of_landing_ctrl.CONTROL_METHOD = OFL_CONTROL_METHOD;
  of_landing_ctrl.COV_METHOD = OFL_COV_METHOD;
  of_landing_ctrl.delay_steps = 15;
  of_landing_ctrl.window_size = OFL_COV_WINDOW_SIZE;
  of_landing_ctrl.pgain_adaptive = OFL_PGAIN;
  of_landing_ctrl.igain_adaptive = OFL_IGAIN;
  of_landing_ctrl.dgain_adaptive = OFL_DGAIN;
  of_landing_ctrl.reduction_factor_elc =
    0.80f; // for exponential gain landing, after detecting oscillations, the gain is multiplied with this factor
  of_landing_ctrl.lp_cov_div_factor =
    0.99f; // low pass filtering cov div so that the drone is really oscillating when triggering the descent
  of_landing_ctrl.t_transition = 2.f;
  // if the gain reaches this value during an exponential landing, the drone makes the final landing.
  of_landing_ctrl.p_land_threshold = OFL_P_LAND_THRESHOLD;
  of_landing_ctrl.elc_oscillate = OFL_ELC_OSCILLATE;
  reset_all_vars();

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OFL_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);
  // Subscribe to the optical flow estimator:
  // register telemetry:
  AbiBindMsgOPTICAL_FLOW(OFL_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DIVERGENCE, send_divergence);
}

/**
 * Reset all variables:
 */
static void reset_all_vars(void)
{
  of_landing_ctrl.agl_lp = of_landing_ctrl.agl = stateGetPositionEnu_f()->z;

  thrust_set = of_landing_ctrl.nominal_thrust * MAX_PPRZ;

  cov_div = 0.;
  normalized_thrust = of_landing_ctrl.nominal_thrust * 100;
  previous_cov_err = 0.;
  divergence_vision = 0.;
  divergence_vision_dt = 0.;
  divergence_setpoint = 0;

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

  of_landing_ctrl.divergence = 0.;
  of_landing_ctrl.previous_err = 0.;
  of_landing_ctrl.sum_err = 0.;
  of_landing_ctrl.d_err = 0.;
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

  Bound(of_landing_ctrl.lp_const, 0.001f, 1.f);
  float lp_factor = dt / of_landing_ctrl.lp_const;
  Bound(lp_factor, 0.f, 1.f);

  /***********
   * VISION
   ***********/
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
    div_factor = -1.28f; // magic number comprising field of view etc.
    float new_divergence = (divergence_vision * div_factor) / dt;

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
    }

    if (in_flight) {
      Bound(thrust_set, 0.25 * of_landing_ctrl.nominal_thrust * MAX_PPRZ, MAX_PPRZ);
      stabilization_cmd[COMMAND_THRUST] = thrust_set;
    }

  }
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
  // determine the error:
  float err = setpoint - of_landing_ctrl.divergence;

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

void vertical_ctrl_optical_flow_cb(uint8_t sender_id UNUSED, uint32_t stamp,
                                   int32_t flow_x UNUSED, int32_t flow_y UNUSED,
                                   int32_t flow_der_x UNUSED, int32_t flow_der_y UNUSED, float quality UNUSED, float size_divergence)
{
  divergence_vision = size_divergence;
  vision_time = ((float)stamp) / 1e6;
}

////////////////////////////////////////////////////////////////////
// Call our controller
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
