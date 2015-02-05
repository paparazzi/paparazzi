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

// Warning: all this code is called form the Vision-Thread: do not access any autopilot data in here.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "visual_estimator.h"

// Computer Vision
#include "opticflow/optic_flow_int.h"
#include "opticflow/fast9/fastRosten.h"

// for FPS
#include "modules/computer_vision/cv/framerate.h"

// Image size set at init
unsigned int imgWidth, imgHeight;

// Local variables
unsigned char *prev_frame, *gray_frame, *prev_gray_frame;
int old_img_init;

// ARDrone Vertical Camera Parameters
#define FOV_H 0.67020643276
#define FOV_W 0.89360857702
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053

// Corner Detection
int *x, *y;
int max_count = 25;
#define MAX_COUNT 100

// Corner Tracking
int *new_x, *new_y, *status, *dx, *dy;
int error_opticflow;
int remove_point;
int c;
int borderx = 24, bordery = 24;

// Remove bad corners
float distance2, min_distance, min_distance2;

// Flow Derotation
#define FLOW_DEROTATION
float curr_pitch, curr_roll, prev_pitch, prev_roll;
float OFx_trans, OFy_trans;



// Called by plugin
void opticflow_plugin_init(unsigned int w, unsigned int h, struct CVresults *results)
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
  results->OFx = 0.0;
  results->OFy = 0.0;
  results->dx_sum = 0.0;
  results->dy_sum = 0.0;
  results->diff_roll = 0.0;
  results->diff_pitch = 0.0;
  results->cam_h = 0.0;
  prev_pitch = 0.0;
  prev_roll = 0.0;
  curr_pitch = 0.0;
  curr_roll = 0.0;
  OFx_trans = 0.0;
  OFy_trans = 0.0;
  results->Velx = 0.0;
  results->Vely = 0.0;
  results->flow_count = 0;
  results->cnt = 0;
  results->status = 0;
  results->count = 0;

  framerate_init();
}

void opticflow_plugin_run(unsigned char *frame, struct PPRZinfo* info, struct CVresults *results)
{
  framerate_run();

  if (old_img_init == 1) {
    memcpy(prev_frame, frame, imgHeight * imgWidth * 2);
    CvtYUYV2Gray(prev_gray_frame, prev_frame, imgWidth, imgHeight);
    old_img_init = 0;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection
  int fast_threshold = 20;
  xyFAST *pnts_fast;
  pnts_fast = fast9_detect((const byte *)prev_gray_frame, imgWidth, imgHeight, imgWidth,
                           fast_threshold, &results->count);

  if (results->count > MAX_COUNT) { results->count = MAX_COUNT; }
  for (int i = 0; i < results->count; i++) {
    x[i] = pnts_fast[i].x;
    y[i] = pnts_fast[i].y;
  }
  free(pnts_fast);

  // Remove neighbouring corners
  min_distance = 3;
  min_distance2 = min_distance * min_distance;
  int *labelmin;
  labelmin = (int *) calloc(MAX_COUNT, sizeof(int));
  for (int i = 0; i < results->count; i++) {
    for (int j = i + 1; j < results->count; j++) {
      // distance squared:
      distance2 = (x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]);
      if (distance2 < min_distance2) {
        labelmin[i] = 1;
      }
    }
  }

  int count_fil = results->count;
  for (int i = results->count - 1; i >= 0; i--) {
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
  results->count = count_fil;
  free(labelmin);

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************
  CvtYUYV2Gray(gray_frame, frame, imgWidth, imgHeight);

  error_opticflow = opticFlowLK(gray_frame, prev_gray_frame, x, y, count_fil, imgWidth,
                                imgHeight, new_x, new_y, status, 5, 100);

  results->flow_count = count_fil;
  for (int i = count_fil - 1; i >= 0; i--) {
    remove_point = 1;

    if (status[i] && !(new_x[i] < borderx || new_x[i] > (imgWidth - 1 - borderx) ||
                       new_y[i] < bordery || new_y[i] > (imgHeight - 1 - bordery))) {
      remove_point = 0;
    }

    if (remove_point) {
      for (c = i; c < results->flow_count - 1; c++) {
        x[c] = x[c + 1];
        y[c] = y[c + 1];
        new_x[c] = new_x[c + 1];
        new_y[c] = new_y[c + 1];
      }
      results->flow_count--;
    }
  }

  results->dx_sum = 0.0;
  results->dy_sum = 0.0;

  // Optical Flow Computation
  for (int i = 0; i < results->flow_count; i++) {
    dx[i] = new_x[i] - x[i];
    dy[i] = new_y[i] - y[i];
  }

  // Median Filter
  if (results->flow_count) {
    quick_sort_int(dx, results->flow_count); // 11
    quick_sort_int(dy, results->flow_count); // 11

    results->dx_sum = (float) dx[results->flow_count / 2];
    results->dy_sum = (float) dy[results->flow_count / 2];
  } else {
    results->dx_sum = 0.0;
    results->dy_sum = 0.0;
  }

  // Flow Derotation
  curr_pitch = info->theta;
  curr_roll = info->phi;

  results->diff_pitch = (curr_pitch - prev_pitch) * imgHeight / FOV_H;
  results->diff_roll = (curr_roll - prev_roll) * imgWidth / FOV_W;

  prev_pitch = curr_pitch;
  prev_roll = curr_roll;

#ifdef FLOW_DEROTATION
  if (results->flow_count) {
    OFx_trans = results->dx_sum - results->diff_roll;
    OFy_trans = results->dy_sum - results->diff_pitch;

    if ((OFx_trans <= 0) != (results->dx_sum <= 0)) {
      OFx_trans = 0;
      OFy_trans = 0;
    }
  } else {
    OFx_trans = results->dx_sum;
    OFy_trans = results->dy_sum;
  }
#else
  OFx_trans = dx_sum;
  OFy_trans = dy_sum;
#endif

  // Average Filter
  OFfilter(&results->OFx, &results->OFy, OFx_trans, OFy_trans, results->flow_count, 1);

  // Velocity Computation
  if (info->agl < 0) {
    results->cam_h = 1;
  }
  else {
    results->cam_h = info->agl;
  }

  results->FPS = framerate_get();

  if (results->flow_count) {
    results->Velx = results->OFy * results->cam_h * results->FPS / Fy_ARdrone + 0.05;
    results->Vely = -results->OFx * results->cam_h * results->FPS / Fx_ARdrone - 0.1;
  } else {
    results->Velx = 0.0;
    results->Vely = 0.0;
  }

  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************

  memcpy(prev_frame, frame, imgHeight * imgWidth * 2);
  memcpy(prev_gray_frame, gray_frame, imgHeight * imgWidth);

}

