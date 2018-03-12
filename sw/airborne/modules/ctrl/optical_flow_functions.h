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

#ifndef OPTICAL_FLOW_FUNCTIONS_H_
#define OPTICAL_FLOW_FUNCTIONS_H_

#include "std.h"
#include "math/pprz_algebra_float.h"

struct GainsPID {
  float P;             ///< P-gain for control
  float I;             ///< I-gain for control
  float D;             ///< D-gain for control
  float err;                     ///< Current tracking error
  float previous_err;            ///< Previous tracking error
  float sum_err;                 ///< integration of the error for I-gain
  float d_err;                   ///< difference of error for the D-gain
};

struct OFhistory {
  float input[COV_WINDOW_SIZE];
  float OF[COV_WINDOW_SIZE];
  float past_OF[COV_WINDOW_SIZE];
};

struct OpticalFlowHoverControl {
  struct GainsPID PID;       ///< The struct with the PID gains

  float nominal_value;       ///< The nominal value of thrust, phi or theta depending on Z, Y, X

  float ramp;                ///< The ramp pused is increased with per dt

  float reduction_factor;    ///< Reduce the gain by this factor when oscillating

  float setpoint;            ///< setpoint for constant divergence/flow
  float cov_setpoint;        ///< for adaptive gain control, setpoint of the covariance (oscillations)
};

struct OpticalFlowHover {
  float divergence;        ///< Divergence estimate
  float flowX;             ///< Flow estimate in X direction
  float flowY;             ///< Flow estimate in Y direction
};

struct DesiredInputs {
  float phi;
  float theta;
  int32_t thrust;
};

extern uint32_t ind_histXY;
extern uint8_t cov_array_filledXY;
extern uint32_t ind_histZ;
extern uint8_t cov_array_filledZ;

struct OpticalFlowHover of_hover;

extern float set_cov_div(bool cov_method, struct OFhistory *history, struct DesiredInputs *inputs);
extern void set_cov_flow(bool cov_method, struct OFhistory *historyX, struct OFhistory *historyY,
                         struct DesiredInputs *inputs, struct FloatVect3 *covs);
extern float PID_flow_control(float dt, struct OpticalFlowHoverControl *of_hover_ctrl);
extern int32_t PID_divergence_control(float dt, struct OpticalFlowHoverControl *of_hover_ctrl);
#endif /* OPTICAL_FLOW_FUNCTIONS_H_ */
