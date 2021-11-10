/*
 * Copyright (C) 2018
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "optical_flow_hover.h"
#include "optical_flow_functions.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "modules/energy/electrical.h"
#include <stdio.h>

#include "subsystems/datalink/telemetry.h"
//
//// for measuring time
#include "mcu_periph/sys_time.h"
//
// Additional math functions
#include "math/pprz_stat.h"

/* Use optical flow estimates */
#ifndef OFH_OPTICAL_FLOW_ID
#define OFH_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFH_OPTICAL_FLOW_ID)

// Set the standard method (1) to apply the algorithm to all 3 axes at the same time
#ifndef OFH_HOVER_METHOD
#define OFH_HOVER_METHOD 1
#endif

// Set the standard method to assume the quadrotor is not (0) symmetrical
#ifndef XY_SYMMETRICAL
#define XY_SYMMETRICAL 0
#endif

// Set the standard method to use vision signal and the known input and signal (0) optionally one can use the autocovariance (1)
#ifndef OFH_COV_METHOD
#define OFH_COV_METHOD 0
#endif

// number of time steps used for calculating the covariance (oscillations) and the delay steps
#ifndef COV_WINDOW_SIZE
#define COV_WINDOW_SIZE (10*30)
#endif

// In case the autocovariance is used, select half the window size as the delay
#ifndef OF_COV_DELAY_STEPS
#define OF_COV_DELAY_STEPS COV_WINDOW_SIZE/2
#endif

// The low pass filter constant for updating the vision measurements in the algorithm
#ifndef OF_LP_CONST
#define OF_LP_CONST 0.5
#endif

// Whether the algorithm should be applied to the phi angle(1) or not(0)
#ifndef OFH_OSCPHI
#define OFH_OSCPHI 1
#endif

// Whether the algorithm should be applied to the theta angle(1) or not(0)
#ifndef OFH_OSCTHETA
#define OFH_OSCTHETA 1
#endif

// Use default PID gains
#ifndef OFH_PGAINZ
#define OFH_PGAINZ 0.4
#endif

#ifndef OFH_IGAINZ
#define OFH_IGAINZ 0.f
#endif

#ifndef OFH_DGAINZ
#define OFH_DGAINZ 0.0
#endif

// Default slope at which the gain is increased
#ifndef OFH_RAMPZ
#define OFH_RAMPZ 0.15
#endif

// Default reduction factor to stabilize the quad after oscillating
#ifndef OFH_REDUCTIONZ
#define OFH_REDUCTIONZ 0.45
#endif

// The threshold value for the covariance oscillation measurement
#ifndef OFH_COVDIV_SETPOINT
#define OFH_COVDIV_SETPOINT -0.02
#endif

// Use default PID gains
#ifndef OFH_PGAINX
#define OFH_PGAINX 0.f
#endif

#ifndef OFH_IGAINX
#define OFH_IGAINX 0.00002
#endif

#ifndef OFH_DGAINX
#define OFH_DGAINX 0.f
#endif

#ifndef OFH_PGAINY
#define OFH_PGAINY 0.f
#endif

#ifndef OFH_IGAINY
#define OFH_IGAINY 0.00002
#endif

#ifndef OFH_DGAINY
#define OFH_DGAINY 0.f
#endif

// Default slope at which the gain is increased
#ifndef OFH_RAMPXY
#define OFH_RAMPXY 0.0008
#endif

// Default reduction factor to stabilize the quad after oscillating
#ifndef OFH_REDUCTIONXY
#define OFH_REDUCTIONXY 0.3
#endif

// The threshold value for the covariance oscillation measurement
#ifndef OFH_COVFLOW_SETPOINT
#define OFH_COVFLOW_SETPOINT -500.f
#endif

// The default slopes for the height-gain relationships
#ifndef OFH_VER_SLOPE_A
#define OFH_VER_SLOPE_A 0.5
#endif

#ifndef OFH_VER_SLOPE_B
#define OFH_VER_SLOPE_B 0.25
#endif

#ifndef OFH_HOR_X_SLOPE_A
#define OFH_HOR_X_SLOPE_A 2.f
#endif

#ifndef OFH_HOR_X_SLOPE_B
#define OFH_HOR_X_SLOPE_B 0.5
#endif

#ifndef OFH_HOR_Y_SLOPE_A
#define OFH_HOR_Y_SLOPE_A OFH_HOR_X_SLOPE_A
#endif

#ifndef OFH_HOR_Y_SLOPE_B
#define OFH_HOR_Y_SLOPE_B OFH_HOR_X_SLOPE_B
#endif


struct DesiredInputs des_inputs;
struct FloatVect3 covariances;

// variables retained between module calls
float vision_time, prev_vision_timeXY, prev_vision_timeZ;

bool oscillatingX;
bool oscillatingY;
int32_t flowX;
int32_t flowY;
struct OFhistory historyX;
struct OFhistory historyY;

// Stabilizing commands
struct Int32Eulers ofh_sp_eu;

bool oscillatingZ;
float divergence_vision;

struct OFhistory historyZ;

// The optical flow ABI event
static abi_event optical_flow_ev;

// struct containing most relevant parameters
struct OpticalFlowHoverControl of_hover_ctrl_X;
struct OpticalFlowHoverControl of_hover_ctrl_Y;
struct OpticalFlowHoverControl of_hover_ctrl_Z;

// variables used in the GCS
bool oscphi;
bool osctheta;
bool derotated;
bool cov_method;
uint8_t hover_method;


/// Function definitions
// Callback function of the optical flow estimate:
void ofh_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int32_t flow_x, int32_t flow_y,
                         int32_t flow_der_x, int32_t flow_der_y, float quality, float size_div);

// resetting all variables to be called for instance when starting up / re-entering module
static void reset_horizontal_vars(void);
static void reset_vertical_vars(void);
void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool in_flight);
void horizontal_ctrl_module_init(void);
void horizontal_ctrl_module_run(bool in_flight);

// Compute OptiTrack stabilization for 1/2 axes
void computeOptiTrack(bool phi, bool theta, struct Int32Eulers *opti_sp_eu);
#ifndef GH_GAIN_SCALE
#define GH_GAIN_SCALE 2
#endif
#ifndef MAX_POS_ERR
#define MAX_POS_ERR POS_BFP_OF_REAL(16.)
#endif
#ifndef MAX_SPEED_ERR
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)
#endif
struct Int32Vect2 of_hover_pos_err;
struct Int32Vect2 of_hover_speed_err;
struct Int32Vect2 of_hover_ref_pos;
struct Int32Vect2 of_hover_trim_att_integrator;
struct Int32Vect2 of_hover_cmd_earth;


// sending the divergence message to the ground station:
static void send_optical_flow_hover(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_OPTICAL_FLOW_HOVER(trans, dev, AC_ID, &of_hover.flowX, &of_hover.flowY, &of_hover.divergence,
                                   &covariances.x, &covariances.y, &covariances.z, &of_hover_ctrl_X.PID.P, &of_hover_ctrl_Y.PID.P, &of_hover_ctrl_Z.PID.P,
                                   &of_hover_ctrl_X.PID.sum_err, &of_hover_ctrl_Y.PID.sum_err, &of_hover_ctrl_Z.PID.sum_err,
                                   &des_inputs.thrust, &des_inputs.phi, &des_inputs.theta);
}


// Init the optical flow hover module
void optical_flow_hover_init()
{
  of_hover_ctrl_X.setpoint = 0.0f;
  of_hover_ctrl_X.cov_setpoint = OFH_COVFLOW_SETPOINT;
  of_hover_ctrl_X.PID.P = OFH_PGAINX;
  of_hover_ctrl_X.PID.I = OFH_IGAINX;
  of_hover_ctrl_X.PID.D = OFH_DGAINX;
  of_hover_ctrl_X.ramp  = OFH_RAMPXY;
  of_hover_ctrl_X.reduction_factor = OFH_REDUCTIONXY;

  of_hover_ctrl_Y.setpoint = 0.0f;
  of_hover_ctrl_Y.cov_setpoint = OFH_COVFLOW_SETPOINT;
  of_hover_ctrl_Y.PID.P = OFH_PGAINY;
  of_hover_ctrl_Y.PID.I = OFH_IGAINY;
  of_hover_ctrl_Y.PID.D = OFH_DGAINY;
  of_hover_ctrl_Y.ramp  = OFH_RAMPXY;
  of_hover_ctrl_Y.reduction_factor = OFH_REDUCTIONXY;

  of_hover_ctrl_Z.setpoint = 0.0f;
  of_hover_ctrl_Z.cov_setpoint  = OFH_COVDIV_SETPOINT;
  of_hover_ctrl_Z.PID.P = OFH_PGAINZ;
  of_hover_ctrl_Z.PID.I = OFH_IGAINZ;
  of_hover_ctrl_Z.PID.D = OFH_DGAINZ;
  of_hover_ctrl_Z.ramp  = OFH_RAMPZ;
  of_hover_ctrl_Z.reduction_factor = OFH_REDUCTIONZ;

  oscphi = OFH_OSCPHI;
  osctheta = OFH_OSCTHETA;

  cov_method = OFH_COV_METHOD;
  hover_method = OFH_HOVER_METHOD;

  reset_horizontal_vars();
  reset_vertical_vars();

  // Subscribe to the optical flow estimator:
  AbiBindMsgOPTICAL_FLOW(OFH_OPTICAL_FLOW_ID, &optical_flow_ev, ofh_optical_flow_cb);

  // register telemetry:
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW_HOVER, send_optical_flow_hover);
}

/**
 * Initialize the vertical optical flow hover module
 */
void vertical_ctrl_module_init(void)
{
  // filling the of_hover_ctrl struct with default values:
  reset_vertical_vars();
}

/**
 * Initialize the horizontal optical flow hover module
 */
void horizontal_ctrl_module_init(void)
{
  // filling the of_hover_ctrl struct with default values:
  reset_horizontal_vars();
}

/**
 * Reset all horizontal variables:
 */
static void reset_horizontal_vars(void)
{
  // Take the current angles as neutral
  struct Int32Eulers tempangle;
  int32_eulers_of_quat(&tempangle, &stab_att_sp_quat);
  of_hover_ctrl_X.nominal_value = DegOfRad(FLOAT_OF_BFP(tempangle.phi, INT32_ANGLE_FRAC));
  of_hover_ctrl_Y.nominal_value = DegOfRad(FLOAT_OF_BFP(tempangle.theta, INT32_ANGLE_FRAC));

  des_inputs.phi = 0;
  des_inputs.theta = 0;

  if ((hover_method == 0) && (GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE)) {
    // Z - X - Y Order
    oscillatingX = 1;
    oscillatingY = 1;
    of_hover_ctrl_X.PID.P = OFH_PGAINX;
    of_hover_ctrl_Y.PID.P = OFH_PGAINX;
  } else if ((hover_method == 2) && (GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE)) {
    // Z Set XY
    oscillatingX = 1;
    oscillatingY = 1;
    of_hover_ctrl_X.PID.P = OFH_PGAINX;
    of_hover_ctrl_Y.PID.P = OFH_PGAINY;
    of_hover_ctrl_X.PID.I = OFH_IGAINX / 4; // Have a slighly lower I gain during Z
    of_hover_ctrl_Y.PID.I = OFH_IGAINY / 4; // Have a slighly lower I gain during Z
  } else {
    // if V is in NAV or hover_method = 1
    //All axes
    oscillatingX = 0;
    oscillatingY = 0;
    of_hover_ctrl_X.PID.P = OFH_PGAINX;
    of_hover_ctrl_Y.PID.P = OFH_PGAINX;
  }

  flowX = 0;
  flowY = 0;

  of_hover_ctrl_X.PID.sum_err = 0.0f;
  of_hover_ctrl_X.PID.d_err = 0.0f;
  of_hover_ctrl_X.PID.previous_err = 0.0f;

  of_hover_ctrl_Y.PID.sum_err = 0.0f;
  of_hover_ctrl_Y.PID.d_err = 0.0f;
  of_hover_ctrl_Y.PID.previous_err = 0.0f;

  ofh_sp_eu.phi = of_hover_ctrl_X.nominal_value;
  ofh_sp_eu.theta = of_hover_ctrl_Y.nominal_value;

  ind_histXY = 0;
  cov_array_filledXY = 0;

  covariances.x = 0.0f;
  covariances.y = 0.0f;

  for (uint16_t i = 0; i < COV_WINDOW_SIZE; i++) {
    historyX.OF[i] = 0.0f;
    historyY.OF[i] = 0.0f;
    historyX.past_OF[i] = 0.0f;
    historyY.past_OF[i] = 0.0f;
    historyX.input[i] = 0.0f;
    historyY.input[i] = 0.0f;
  }

  of_hover.flowX = 0;
  of_hover.flowY = 0;

  vision_time = get_sys_time_float();
  prev_vision_timeXY = vision_time;
}

/**
 * Reset all vertical variables:
 */
static void reset_vertical_vars(void)
{
  oscillatingZ = 0;

  divergence_vision = 0;
  of_hover.divergence = 0;

  for (uint16_t i = 0; i < COV_WINDOW_SIZE; i++) {
    historyZ.OF[i] = 0.0f;
    historyZ.past_OF[i] = 0.0f;
    historyZ.input[i] = 0.0f;
  }

  ind_histZ = 0;

  covariances.z = 0.0f;
  cov_array_filledZ = 0;

  of_hover_ctrl_Z.PID.P = OFH_PGAINZ;

  of_hover_ctrl_Z.PID.sum_err = 0.0f;
  of_hover_ctrl_Z.PID.d_err = 0.0f;
  of_hover_ctrl_Z.PID.previous_err = 0.0f;

  vision_time = get_sys_time_float();
  prev_vision_timeZ = vision_time;
}

// Read H RC
void guidance_h_module_read_rc(void) {}

/**
 * Run the horizontal optical flow hover module
 */
void horizontal_ctrl_module_run(bool in_flight)
{
  /***********
   * TIME
   ***********/
  float ventral_factor = -1.28f; // magic number comprising field of view etc.

  float dt = vision_time - prev_vision_timeXY;

  // check if new measurement received
  if (dt <= 1e-5f) {
    return;
  }

  /***********
   * VISION
   ***********/

  float lp_factor = dt / OF_LP_CONST;
  Bound(lp_factor, 0.f, 1.f);

  float new_flowX = (flowX * ventral_factor) / dt;
  float new_flowY = (flowY * ventral_factor) / dt;

  //TODO: deal with (unlikely) fast changes in Flow?

  // low-pass filter the divergence:
  of_hover.flowX += (new_flowX - of_hover.flowX) * lp_factor;
  of_hover.flowY += (new_flowY - of_hover.flowY) * lp_factor;


  /***********
   * CONTROL
   ***********/

  if (!oscillatingX) {
    // if not oscillating, increase gain
    of_hover_ctrl_X.PID.P += of_hover_ctrl_X.ramp * dt;
  }
  if (!oscillatingY) {
    // if not oscillating, increase gain
    of_hover_ctrl_Y.PID.P += of_hover_ctrl_Y.ramp * dt;
  }

  // set desired pitch en roll
  if (oscphi) {
    of_hover_ctrl_X.PID.err = of_hover_ctrl_X.setpoint - of_hover.flowX;
    des_inputs.phi = of_hover_ctrl_X.nominal_value + PID_flow_control(dt, &of_hover_ctrl_X);
  }
  if (osctheta) {
    of_hover_ctrl_Y.PID.err = of_hover_ctrl_Y.setpoint - of_hover.flowY;
    des_inputs.theta = of_hover_ctrl_Y.nominal_value + PID_flow_control(dt, &of_hover_ctrl_Y);
  }

  // update covariance
  set_cov_flow(cov_method, &historyX, &historyY, &des_inputs, &covariances);
  ofh_sp_eu.phi = BFP_OF_REAL(RadOfDeg(des_inputs.phi * oscphi), INT32_ANGLE_FRAC);
  ofh_sp_eu.theta = BFP_OF_REAL(RadOfDeg(des_inputs.theta * osctheta), INT32_ANGLE_FRAC);

  // Check for oscillations
  if ((covariances.x < of_hover_ctrl_X.cov_setpoint) && (!oscillatingX)) {
    oscillatingX = 1;

    if (hover_method == 0) {
      //Start the Y axis
      oscillatingY = 0;
      of_hover_ctrl_Y.PID.P = OFH_PGAINY;
    }
    of_hover_ctrl_X.PID.P = of_hover_ctrl_X.PID.P * of_hover_ctrl_X.reduction_factor;

    if (XY_SYMMETRICAL) {
      // also set Y
      oscillatingY = 1;
      of_hover_ctrl_Y.PID.P = of_hover_ctrl_X.PID.P;
    }
  }
  if ((covariances.y < of_hover_ctrl_Y.cov_setpoint) && (!oscillatingY)) {
    oscillatingY = 1;
    of_hover_ctrl_Y.PID.P = of_hover_ctrl_Y.PID.P * of_hover_ctrl_Y.reduction_factor;
  }

  // Compute 0, 1 or 2 horizontal axes with optitrack
  computeOptiTrack(!oscphi, !osctheta, &ofh_sp_eu);
  // Run the stabilization mode
  stabilization_attitude_set_rpy_setpoint_i(&ofh_sp_eu);

  prev_vision_timeXY = vision_time;
}

/**
 * Run the vertical optical flow hover module
 */
void vertical_ctrl_module_run(bool in_flight)
{
  /***********
   * TIME
   ***********/

  float div_factor; // factor that maps divergence in pixels as received from vision to 1 / frame

  float dt = vision_time - prev_vision_timeZ;

  // check if new measurement received
  if (dt <= 1e-5f) {
    return;
  }

  /***********
   * VISION
   ***********/

  float lp_factor = dt / OF_LP_CONST;
  Bound(lp_factor, 0.f, 1.f);

  // Vision
  div_factor = -1.28f; // magic number comprising field of view etc.
  float new_divergence = (divergence_vision * div_factor) / dt;

  // deal with (unlikely) fast changes in divergence:
  static const float max_div_dt = 0.20f;
  if (fabsf(new_divergence - of_hover.divergence) > max_div_dt) {
    if (new_divergence < of_hover.divergence) { new_divergence = of_hover.divergence - max_div_dt; }
    else { new_divergence = of_hover.divergence + max_div_dt; }
  }
  // low-pass filter the divergence:

  of_hover.divergence += (new_divergence - of_hover.divergence) * lp_factor;
  prev_vision_timeZ = vision_time;

  /***********
   * CONTROL
   ***********/
  if (!oscillatingZ) {
    // if not oscillating, increase gain
    of_hover_ctrl_Z.PID.P += of_hover_ctrl_Z.ramp * dt;
  }

  // use the divergence for control:
  of_hover_ctrl_Z.PID.err = of_hover_ctrl_Z.setpoint - of_hover.divergence;
  des_inputs.thrust = PID_divergence_control(dt, &of_hover_ctrl_Z);

  covariances.z = set_cov_div(cov_method, &historyZ, &des_inputs);

  // Check for oscillations
  if (covariances.z < of_hover_ctrl_Z.cov_setpoint && (!oscillatingZ)) {
    float estimatedHeight = of_hover_ctrl_Z.PID.P * OFH_VER_SLOPE_A + OFH_VER_SLOPE_B; // Vertical slope Z
    oscillatingZ = 1;
    of_hover_ctrl_Z.setpoint = 0.0f;
    of_hover_ctrl_Z.PID.P = of_hover_ctrl_Z.PID.P * of_hover_ctrl_Z.reduction_factor;

    if (hover_method == 0) {
      // Z- X - Y Order
      //    Start the X axis
      oscillatingX = 0;
      of_hover_ctrl_X.PID.P = OFH_PGAINX;
    } else if (hover_method == 2) {
      // Start XY axes with computed slope
      of_hover_ctrl_X.PID.sum_err = 0.0f;
      of_hover_ctrl_Y.PID.sum_err = 0.0f;
      of_hover_ctrl_X.PID.I = OFH_IGAINX;
      of_hover_ctrl_Y.PID.I = OFH_IGAINY;
      of_hover_ctrl_X.PID.P = OFH_REDUCTIONXY * (estimatedHeight * OFH_HOR_X_SLOPE_A - OFH_HOR_X_SLOPE_B); // Horizontal Slope X
      of_hover_ctrl_Y.PID.P = OFH_REDUCTIONXY * (estimatedHeight * OFH_HOR_Y_SLOPE_A - OFH_HOR_Y_SLOPE_B); //  Horizontal Slope Y
    } else {
      // hover_method = 1
      //All axes
    }
  }

  stabilization_cmd[COMMAND_THRUST] = des_inputs.thrust;
}

void ofh_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int32_t flow_x, int32_t flow_y,
                         int32_t flow_der_x, int32_t flow_der_y, float quality, float size_div)
{
  if (!derotated) {
    flowX = flow_x;
    flowY = flow_y;
  } else {
    flowX = flow_der_x;
    flowY = flow_der_y;
  }

  divergence_vision = size_div;

  vision_time = ((float)stamp) / 1e6;
}

////////////////////////////////////////////////////////////////////
// Call our vertical controller
void guidance_v_module_init(void)
{
  vertical_ctrl_module_init();
}

// Call our horizontal controller
void guidance_h_module_init(void)
{
  horizontal_ctrl_module_init();
}

/**
 * Entering the vertical module (user switched to module)
 */
void guidance_v_module_enter(void)
{
  reset_vertical_vars();

  // adaptive estimation - assume hover condition when entering the module
  of_hover_ctrl_Z.nominal_value = (float) stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
  des_inputs.thrust = (int32_t) of_hover_ctrl_Z.nominal_value * MAX_PPRZ;
}

/**
 * Entering the horizontal module (user switched to module)
 */
void guidance_h_module_enter(void)
{
  // Set current psi as heading
  ofh_sp_eu.psi = stateGetNedToBodyEulers_i()->psi;

  VECT2_COPY(of_hover_ref_pos, *stateGetPositionNed_i());
  reset_horizontal_vars();

}

// Run the veritcal controller
void guidance_v_module_run(bool in_flight)
{
  if (electrical.bat_low) {
    autopilot_static_set_mode(AP_MODE_NAV);
  } else {
    // your vertical controller goes here
    vertical_ctrl_module_run(in_flight);
  }
}

// Run the horizontal controller
void guidance_h_module_run(bool in_flight)
{

  if (electrical.bat_low) {
    autopilot_static_set_mode(AP_MODE_NAV);
  } else {
    horizontal_ctrl_module_run(in_flight);
    stabilization_attitude_run(in_flight);
  }
}

/**
 * Get the desired Euler angles for optitrack stabilization
 * @param[in] Boolean whether to Phi or not
 * @param[in] Boolean whether to Theta or not
 * @param[out] The desired Euler angles
 */
void computeOptiTrack(bool phi, bool theta, struct Int32Eulers *opti_sp_eu)
{

  bool optiVelOnly;
  optiVelOnly = 0;

  int32_t psi = stateGetNedToBodyEulers_i()->psi;

  struct NedCoor_i vel_from_GPS;
  struct NedCoor_i pos_from_GPS;

  vel_from_GPS = *stateGetSpeedNed_i();
  pos_from_GPS = *stateGetPositionNed_i();

  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                       BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

  /* compute position error    */
  VECT2_DIFF(of_hover_pos_err, of_hover_ref_pos, pos_from_GPS);
  /* saturate it               */
  VECT2_STRIM(of_hover_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  struct Int32Vect2 ref_speed;
  ref_speed.x = 0;
  ref_speed.y = 0;

  /* compute speed error    */
  VECT2_DIFF(of_hover_speed_err, ref_speed, vel_from_GPS);
  /* saturate it               */
  VECT2_STRIM(of_hover_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  if (optiVelOnly) {
    of_hover_pos_err.x = 0;
    of_hover_pos_err.y = 0;
  }

  /* run PID */
  of_hover_cmd_earth.x =
    ((GUIDANCE_H_PGAIN * of_hover_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((GUIDANCE_H_DGAIN * (of_hover_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  of_hover_cmd_earth.y =
    ((GUIDANCE_H_PGAIN * of_hover_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((GUIDANCE_H_DGAIN * (of_hover_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));

  /* trim max bank angle from PD */
  VECT2_STRIM(of_hover_cmd_earth, -traj_max_bank, traj_max_bank);

  of_hover_trim_att_integrator.x += (GUIDANCE_H_IGAIN * of_hover_cmd_earth.x);
  of_hover_trim_att_integrator.y += (GUIDANCE_H_IGAIN * of_hover_cmd_earth.y);
  /* saturate it  */
  VECT2_STRIM(of_hover_trim_att_integrator, -(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)),
              (traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)));

  /* add it to the command */
  of_hover_cmd_earth.x += (of_hover_trim_att_integrator.x >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
  of_hover_cmd_earth.y += (of_hover_trim_att_integrator.y >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));

  VECT2_STRIM(of_hover_cmd_earth, -total_max_bank, total_max_bank);

  // Compute Angle Setpoints - Taken from Stab_att_quat
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);

  if (phi) {
    opti_sp_eu->phi = (-s_psi * of_hover_cmd_earth.x + c_psi * of_hover_cmd_earth.y) >> INT32_TRIG_FRAC;
  }
  if (theta) {
    opti_sp_eu->theta = -(c_psi * of_hover_cmd_earth.x + s_psi * of_hover_cmd_earth.y) >> INT32_TRIG_FRAC;
  }
}
