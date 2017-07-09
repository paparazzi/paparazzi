/*
 * Copyright (C) 2016
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
 * @file modules/ctrl/optical_flow_landing.c
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

#ifndef OPTICAL_FLOW_LANDING_H_
#define OPTICAL_FLOW_LANDING_H_

#include "std.h"

struct OpticalFlowLanding {
  float agl;                    ///< agl = height from sonar (only used when using "fake" divergence)
  float agl_lp;                 ///< low-pass version of agl
  float lp_const;               ///< low-pass filter constant
  float vel;                    ///< vertical velocity as determined with sonar (only used when using "fake" divergence)
  float divergence_setpoint;    ///< setpoint for constant divergence approach
  float pgain;                  ///< P-gain for constant divergence control (from divergence error to thrust)
  float igain;                  ///< I-gain for constant divergence control
  float dgain;                  ///< D-gain for constant divergence control
  float divergence;             ///< Divergence estimate
  float previous_err;           ///< Previous divergence tracking error
  float sum_err;                ///< integration of the error for I-gain
  float d_err;                  ///< difference of error for the D-gain
  float nominal_thrust;         ///< nominal thrust around which the PID-control operates
  uint32_t VISION_METHOD;       ///< whether to use vision (1) or Optitrack / sonar (0)
  uint32_t CONTROL_METHOD;      ///< type of divergence control: 0 = fixed gain, 1 = adaptive gain
  float cov_set_point;          ///< for adaptive gain control, setpoint of the covariance (oscillations)
  float cov_limit;              ///< for fixed gain control, what is the cov limit triggering the landing
  float pgain_adaptive;         ///< P-gain for adaptive gain control
  float igain_adaptive;         ///< I-gain for adaptive gain control
  float dgain_adaptive;         ///< D-gain for adaptive gain control
  uint32_t COV_METHOD;          ///< method to calculate the covariance: between thrust and div (0) or div and div past (1)
  uint32_t delay_steps;         ///< number of delay steps for div past
  uint32_t window_size;         ///< number of time steps in "window" used for getting the covariance
  float reduction_factor_elc;   ///< reduction factor - after oscillation, how much to reduce the gain...
  float lp_cov_div_factor;      ///< low-pass factor for the covariance of divergence in order to trigger the second landing phase in the exponential strategy.
  float t_transition;           ///< how many seconds the drone has to be oscillating in order to transition to the hover phase with reduced gain
  float p_land_threshold;       ///< if during the exponential landing the gain reaches this value, the final landing procedure is triggered
  bool elc_oscillate;           ///< Whether or not to oscillate at beginning of elc to find optimum gain
};

extern struct OpticalFlowLanding of_landing_ctrl;

// Without optitrack set to: GUIDANCE_H_MODE_ATTITUDE
// With optitrack set to: GUIDANCE_H_MODE_HOVER / NAV
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_NAV

// Own guidance_v
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif /* OPTICAL_FLOW_LANDING_H_ */
