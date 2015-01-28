/*
 * Copyright (C) 2014 Hann Woei Ho
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
 * @file modules/computer_vision/opticflow/visual_estimator.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#include <stdio.h>
#include <stdlib.h>

// Own Header
#include "visual_estimator.h"

// Computer Vision
#include "opticflow/optic_flow_int.h"
#include "opticflow/fast9/fastRosten.h"

// for FPS
#include "modules/computer_vision/opticflow_module.h"

// Paparazzi Data
#include "state.h"

// Downlink
#include "subsystems/datalink/downlink.h"

// Timer
#include <sys/time.h>

// Image size set at init
unsigned int imgWidth, imgHeight;

// Local variables
unsigned char *prev_frame, *gray_frame, *prev_gray_frame;
int old_img_init;
float OFx, OFy, dx_sum, dy_sum;

// ARDrone Vertical Camera Parameters
#define FOV_H 0.67020643276
#define FOV_W 0.89360857702
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053

// Corner Detection
int *x, *y;
int count = 0;
int max_count = 25;
#define MAX_COUNT 100

// Corner Tracking
int *new_x, *new_y, *status, *dx, *dy;
int error_opticflow;
int flow_count = 0;
int remove_point;
int c;
int borderx = 24, bordery = 24;

// Remove bad corners
float distance2, min_distance, min_distance2;

// Flow Derotation
#define FLOW_DEROTATION
float curr_pitch, curr_roll, curr_yaw, prev_pitch, prev_roll;
float cam_h, diff_roll, diff_pitch, OFx_trans, OFy_trans;

// Lateral Velocity Computation
float Velx, Vely;

// Compute body velocities
struct FloatVect3 V_body;

// Called by plugin
void my_plugin_init(unsigned int w, unsigned int h)
{
  // Initialize variables
  imgWidth = w;
  imgHeight = h;
  gray_frame = (unsigned char *) calloc(imgWidth * imgHeight, sizeof(unsigned char));
  prev_frame = (unsigned char *) calloc(imgWidth * imgHeight * 2, sizeof(unsigned char));
  prev_gray_frame = (unsigned char *) calloc(imgWidth * imgHeight, sizeof(unsigned char));
  x = (int *) calloc(MAX_COUNT, sizeof(int));
  new_x = (int *) calloc(MAX_COUNT, sizeof(int));
  y = (int *) calloc(MAX_COUNT, sizeof(int));
  new_y = (int *) calloc(MAX_COUNT, sizeof(int));
  status = (int *) calloc(MAX_COUNT, sizeof(int));
  dx = (int *) calloc(MAX_COUNT, sizeof(int));
  dy = (int *) calloc(MAX_COUNT, sizeof(int));
  old_img_init = 1;
  OFx = 0.0;
  OFy = 0.0;
  dx_sum = 0.0;
  dy_sum = 0.0;
  diff_roll = 0.0;
  diff_pitch = 0.0;
  cam_h = 0.0;
  prev_pitch = 0.0;
  prev_roll = 0.0;
  curr_pitch = 0.0;
  curr_roll = 0.0;
  curr_yaw = 0.0;
  OFx_trans = 0.0;
  OFy_trans = 0.0;
  Velx = 0.0;
  Vely = 0.0;
}

void my_plugin_run(unsigned char *frame)
{
  if (old_img_init == 1) {
    memcpy(prev_frame, frame, imgHeight * imgWidth * 2);
    CvtYUYV2Gray(prev_gray_frame, prev_frame, imgWidth, imgHeight);
    old_img_init = 0;
  }

  // ***********************************************************************************************************************
  // Additional information from other sensors
  // ***********************************************************************************************************************

  // Compute body velocities from ENU
  struct FloatVect3 *vel_ned = (struct FloatVect3*)stateGetSpeedNed_f();
  struct FloatQuat *q_n2b = stateGetNedToBodyQuat_f();
  float_quat_vmult(&V_body, q_n2b, vel_ned);

  // ***********************************************************************************************************************
  // Corner detection
  // ***********************************************************************************************************************

  // FAST corner detection
  int fast_threshold = 20;
  xyFAST *pnts_fast;
  pnts_fast = fast9_detect((const byte *)prev_gray_frame, imgWidth, imgHeight, imgWidth, fast_threshold, &count);

  if (count > MAX_COUNT) { count = MAX_COUNT; }
  for (int i = 0; i < count; i++) {
    x[i] = pnts_fast[i].x;
    y[i] = pnts_fast[i].y;
  }
  free(pnts_fast);

  // Remove neighbouring corners
  min_distance = 3;
  min_distance2 = min_distance * min_distance;
  int *labelmin;
  labelmin = (int *) calloc(MAX_COUNT, sizeof(int));
  for (int i = 0; i < count; i++) {
    for (int j = i + 1; j < count; j++) {
      // distance squared:
      distance2 = (x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]);
      if (distance2 < min_distance2) {
        labelmin[i] = 1;
      }
    }
  }

  int count_fil = count;
  for (int i = count - 1; i >= 0; i--) {
    remove_point = 0;

    if (labelmin[i]) {
      remove_point = 1;
    }

    if (remove_point) {
      for (c = i; c < count_fil - 1; c++) {
        x[c] = x[c + 1];
        y[c] = y[c + 1];
      }
      count_fil--;
    }
  }

  if (count_fil > max_count) { count_fil = max_count; }
  count = count_fil;
  free(labelmin);

  // **********************************************************************************************************************
  // Corner Tracking
  // **********************************************************************************************************************
  CvtYUYV2Gray(gray_frame, frame, imgWidth, imgHeight);

  error_opticflow = opticFlowLK(gray_frame, prev_gray_frame, x, y, count_fil, imgWidth, imgHeight, new_x, new_y, status,
                                5, 100);

  flow_count = count_fil;
  for (int i = count_fil - 1; i >= 0; i--) {
    remove_point = 1;

    if (status[i] && !(new_x[i] < borderx || new_x[i] > (imgWidth - 1 - borderx) ||
                       new_y[i] < bordery || new_y[i] > (imgHeight - 1 - bordery))) {
      remove_point = 0;
    }

    if (remove_point) {
      for (c = i; c < flow_count - 1; c++) {
        x[c] = x[c + 1];
        y[c] = y[c + 1];
        new_x[c] = new_x[c + 1];
        new_y[c] = new_y[c + 1];
      }
      flow_count--;
    }
  }

  dx_sum = 0.0;
  dy_sum = 0.0;

  // Optical Flow Computation
  for (int i = 0; i < flow_count; i++) {
    dx[i] = new_x[i] - x[i];
    dy[i] = new_y[i] - y[i];
  }

  // Median Filter
  if (flow_count) {
    quick_sort_int(dx, flow_count); // 11
    quick_sort_int(dy, flow_count); // 11

    dx_sum = (float) dx[flow_count / 2];
    dy_sum = (float) dy[flow_count / 2];
  } else {
    dx_sum = 0.0;
    dy_sum = 0.0;
  }

  // Flow Derotation
  curr_pitch = stateGetNedToBodyEulers_f()->theta;
  curr_roll = stateGetNedToBodyEulers_f()->phi;
  curr_yaw = stateGetNedToBodyEulers_f()->psi;

  diff_pitch = (curr_pitch - prev_pitch) * imgHeight / FOV_H;
  diff_roll = (curr_roll - prev_roll) * imgWidth / FOV_W;

  prev_pitch = curr_pitch;
  prev_roll = curr_roll;

#ifdef FLOW_DEROTATION
  if (flow_count) {
    OFx_trans = dx_sum - diff_roll;
    OFy_trans = dy_sum - diff_pitch;

    if ((OFx_trans <= 0) != (dx_sum <= 0)) {
      OFx_trans = 0;
      OFy_trans = 0;
    }
  } else {
    OFx_trans = dx_sum;
    OFy_trans = dy_sum;
  }
#else
  OFx_trans = dx_sum;
  OFy_trans = dy_sum;
#endif

  // Average Filter
  OFfilter(&OFx, &OFy, OFx_trans, OFy_trans, flow_count, 1);

  // Velocity Computation
#if USE_SONAR
  cam_h = 1; //ins_impl.sonar_z;
#else
  cam_h = 1;
#endif

  if (flow_count) {
    Velx = OFy * cam_h * FPS / Fy_ARdrone + 0.05;
    Vely = -OFx * cam_h * FPS / Fx_ARdrone - 0.1;
  } else {
    Velx = 0.0;
    Vely = 0.0;
  }

  // **********************************************************************************************************************
  // Next Loop Preparation
  // **********************************************************************************************************************

  memcpy(prev_frame, frame, imgHeight * imgWidth * 2);
  memcpy(prev_gray_frame, gray_frame, imgHeight * imgWidth);

  // **********************************************************************************************************************
  // Downlink Message
  // **********************************************************************************************************************
  DOWNLINK_SEND_OF_HOVER(DefaultChannel, DefaultDevice, &FPS, &dx_sum, &dy_sum, &OFx, &OFy, &diff_roll, &diff_pitch,
                         &Velx, &Vely, &V_body.x, &V_body.y, &cam_h, &count);
}

