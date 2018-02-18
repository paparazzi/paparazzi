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

#include "optical_flow_functions.h"
#include "paparazzi.h"
#include "math/pprz_stat.h"
#include "std.h"

// The maximum bank angle the algorithm can steer
#ifndef OFH_MAXBANK
#define OFH_MAXBANK 10.f
#endif

// The low pass filter constant for updating the vision measurements in the algorithm
#ifndef OF_LP_CONST
#define OF_LP_CONST 0.5 // Average between 0.4 Bebop value and 0.6 ARDrone value
#endif

// In case the autocovariance is used, select half the window size as the delay
#ifndef OF_COV_DELAY_STEPS
#define OF_COV_DELAY_STEPS COV_WINDOW_SIZE/2
#endif

uint32_t ind_histXY;
uint8_t cov_array_filledXY;
uint32_t ind_histZ;
uint8_t cov_array_filledZ;


/**
 * Set the covariance of the divergence and the thrust / past divergence
 * This funciton should only be called once per time step
 * @param[in] thrust: the current thrust value
 */
float set_cov_div(bool cov_method, struct OFhistory *history, struct DesiredInputs *inputs)
{
  float cov_div = 0;
  // histories and cov detection:

  history->OF[ind_histZ] = of_hover.divergence;

  float normalized_thrust = (float)(100.0 * inputs->thrust / MAX_PPRZ);
  history->input[ind_histZ] = normalized_thrust;

  int ind_past = ind_histZ - OF_COV_DELAY_STEPS;
  while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
  history->past_OF[ind_histZ] = history->OF[ind_past];

  // determine the covariance for hover detection:
  // only take covariance into account if there are enough samples in the histories:
  if (cov_method == 0 && cov_array_filledZ > 0) {
    // TODO: step in hover set point causes an incorrectly perceived covariance
    cov_div = covariance_f(history->input, history->OF, COV_WINDOW_SIZE);
  } else if (cov_method == 1 && cov_array_filledZ > 1) {
    // todo: delay steps should be invariant to the run frequency
    cov_div = covariance_f(history->past_OF, history->OF, COV_WINDOW_SIZE);
  }

  if (cov_array_filledZ < 2 && ind_histZ + 1 == COV_WINDOW_SIZE) {
    cov_array_filledZ++;
  }
  ind_histZ = (ind_histZ + 1) % COV_WINDOW_SIZE;

  return cov_div;
}


/**
 * Set the covariance of the flow and past flow / desired angle
 * This funciton should only be called once per time step
 */
void set_cov_flow(bool cov_method, struct OFhistory *historyX, struct OFhistory *historyY, struct DesiredInputs *inputs,
                  struct FloatVect3 *covs)
{
  // histories and cov detection:
  historyX->OF[ind_histXY] = of_hover.flowX;
  historyY->OF[ind_histXY] = of_hover.flowY;

  int ind_past = ind_histXY - OF_COV_DELAY_STEPS;
  while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
  historyX->past_OF[ind_histXY] = historyX->OF[ind_past];
  historyY->past_OF[ind_histXY] = historyY->OF[ind_past];
  float normalized_phi = (float)(100.0 * inputs->phi / OFH_MAXBANK);
  float normalized_theta = (float)(100.0 * inputs->theta / OFH_MAXBANK);
  historyX->input[ind_histXY] = normalized_phi;
  historyY->input[ind_histXY] = normalized_theta;

  //   determine the covariance for hover detection:
  //   only take covariance into account if there are enough samples in the histories:
  if (cov_method == 0 && cov_array_filledXY > 0) {
    //    // TODO: step in hover set point causes an incorrectly perceived covariance
    covs->x = covariance_f(historyX->input, historyX->OF, COV_WINDOW_SIZE);
    covs->y = covariance_f(historyY->input, historyY->OF, COV_WINDOW_SIZE);
  } else if (cov_method == 1 && cov_array_filledXY > 1) {
    if (cov_array_filledXY > 1) {
      // todo: delay steps should be invariant to the run frequency
      covs->x = covariance_f(historyX->past_OF, historyX->OF, COV_WINDOW_SIZE);
      covs->y = covariance_f(historyY->past_OF, historyY->OF, COV_WINDOW_SIZE);
    }
  }

  if (cov_array_filledXY < 2 && ind_histXY + 1 == COV_WINDOW_SIZE) {
    cov_array_filledXY++;
  }
  ind_histXY = (ind_histXY + 1) % COV_WINDOW_SIZE;
}


/**
 * Determine and set the desired angle for constant flow control
 * @param[out] desired angle
 * @param[in] dt: time difference since last update
 * @param[in] *of_hover_ctrl: OpticalFlowHoverControl structure
 */
float PID_flow_control(float dt, struct OpticalFlowHoverControl *of_hover_ctrl)
{
  // update the controller errors:
  float lp_factor = dt / OF_LP_CONST;
  Bound(lp_factor, 0.f, 1.f);

  // maintain the controller errors:
  of_hover_ctrl->PID.sum_err += of_hover_ctrl->PID.err;
  of_hover_ctrl->PID.d_err += (((of_hover_ctrl->PID.err - of_hover_ctrl->PID.previous_err) / dt) -
                                  of_hover_ctrl->PID.d_err) * lp_factor;
  of_hover_ctrl->PID.previous_err = of_hover_ctrl->PID.err;

  // compute the desired angle
  float des_angle = of_hover_ctrl->PID.P * of_hover_ctrl->PID.err + of_hover_ctrl->PID.I *
                    of_hover_ctrl->PID.sum_err + of_hover_ctrl->PID.D * of_hover_ctrl->PID.d_err;

  // Bound angle:
  Bound(des_angle, -OFH_MAXBANK, OFH_MAXBANK);

  return des_angle;
}

/**
 * Determine and set the thrust for constant divergence control
 * @param[out] thrust
 * @param[in] dt: time difference since last update
 * @param[in] *of_hover_ctrl: OpticalFlowHoverControl structure
 */
int32_t PID_divergence_control(float dt, struct OpticalFlowHoverControl *of_hover_ctrl)
{
  // update the controller errors:
  float lp_factor = dt / OF_LP_CONST;
  Bound(lp_factor, 0.f, 1.f);

  // maintain the controller errors:
  of_hover_ctrl->PID.sum_err += of_hover_ctrl->PID.err;
  of_hover_ctrl->PID.d_err += (((of_hover_ctrl->PID.err - of_hover_ctrl->PID.previous_err) / dt) -
                                  of_hover_ctrl->PID.d_err) * lp_factor;
  of_hover_ctrl->PID.previous_err = of_hover_ctrl->PID.err;

  // PID control:
  int32_t thrust = (of_hover_ctrl->nominal_value
                    + of_hover_ctrl->PID.P * of_hover_ctrl->PID.err
                    + of_hover_ctrl->PID.I * of_hover_ctrl->PID.sum_err
                    + of_hover_ctrl->PID.D * of_hover_ctrl->PID.d_err) * MAX_PPRZ;

  // bound thrust:
  Bound(thrust, 0.25 * of_hover_ctrl->nominal_value * MAX_PPRZ, MAX_PPRZ);

  return thrust;
}
