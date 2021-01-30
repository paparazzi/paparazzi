/*
 * Copyright (C) 2015 The Paparazzi Community
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
 * @file modules/computer_vision/opticflow/inter_thread_data.h
 * @brief Inter-thread data structures.
 *
 * Data structures used to for inter-thread communication via Unix Domain sockets.
 */


#ifndef _INTER_THREAD_DATA_H
#define _INTER_THREAD_DATA_H

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"


/* The result calculated from the opticflow */
struct opticflow_result_t {
  float fps;              ///< Frames per second of the optical flow calculation
  uint16_t corner_cnt;    ///< The amount of coners found by FAST9
  uint16_t tracked_cnt;   ///< The amount of tracked corners

  // Camera frame with the origin in the top left corner of the image
  int16_t flow_x;         ///< Flow in x direction from the camera (in subpixels) with X positive to the right
  int16_t flow_y;         ///< Flow in y direction from the camera (in subpixels) with Y positive to the bottom
  int16_t flow_der_x;     ///< The derotated flow calculation in the x direction (in subpixels)
  int16_t flow_der_y;     ///< The derotated flow calculation in the y direction (in subpixels)

  struct FloatVect3 vel_cam;      ///< The velocity in camera frame (m/s)
  struct FloatVect3 vel_body;     ///< The velocity in body frame (m/s) with X positive to the front of the aircraft, Y positive to the right and Z positive downwards to the ground

  float div_size;         ///< Divergence as determined with the size_divergence script

  float surface_roughness; ///< Surface roughness as determined with a linear optical flow fit
  float divergence;       ///< Divergence as determined with a linear flow fit
  uint8_t camera_id;      ///< Camera id as passed to cv_add_to_device

  float noise_measurement;  ///< noise of measurement, for state filter
};

#endif


