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
 * @brief Estimate velocity from optic flow.
 *
 * Using sensors from vertical camera and IMU of Parrot AR.Drone 2.0.
 *
 * Warning: all this code is called form the Vision-Thread: do not access any autopilot data in here.
 */

#include "std.h"

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


// Local variables
struct visual_estimator_struct
{
  // Image size
  unsigned int imgWidth;
  unsigned int imgHeight;

  // Images
  uint8_t *prev_frame;
  uint8_t *gray_frame;
  uint8_t *prev_gray_frame;

  // Initialization
  int old_img_init;

  // Store previous
  float prev_pitch;
  float prev_roll;
} visual_estimator;

// ARDrone Vertical Camera Parameters
#define FOV_H 0.67020643276
#define FOV_W 0.89360857702
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053

// Corner Detection
#define MAX_COUNT 100

// Flow Derotation
#define FLOW_DEROTATION


// Called by plugin
void opticflow_plugin_init(unsigned int w, unsigned int h, struct CVresults *results)
{
  // Initialize variables
  visual_estimator.imgWidth = w;
  visual_estimator.imgHeight = h;

  visual_estimator.gray_frame = (unsigned char *) calloc(w * h, sizeof(uint8_t));
  visual_estimator.prev_frame = (unsigned char *) calloc(w * h * 2, sizeof(uint8_t));
  visual_estimator.prev_gray_frame = (unsigned char *) calloc(w * h, sizeof(uint8_t));

  visual_estimator.old_img_init = 1;
  visual_estimator.prev_pitch = 0.0;
  visual_estimator.prev_roll = 0.0;

  results->OFx = 0.0;
  results->OFy = 0.0;
  results->dx_sum = 0.0;
  results->dy_sum = 0.0;
  results->diff_roll = 0.0;
  results->diff_pitch = 0.0;
  results->cam_h = 0.0;
  results->Velx = 0.0;
  results->Vely = 0.0;
  results->flow_count = 0;
  results->cnt = 0;
  results->count = 0;

  framerate_init();
}

void opticflow_plugin_run(unsigned char *frame, struct PPRZinfo* info, struct CVresults *results)
{
  // Corner Tracking
  // Working Variables
  int max_count = 25;
  int borderx = 24, bordery = 24;
  int x[MAX_COUNT], y[MAX_COUNT];
  int new_x[MAX_COUNT], new_y[MAX_COUNT];
  int status[MAX_COUNT];
  int dx[MAX_COUNT], dy[MAX_COUNT];
  int w = visual_estimator.imgWidth;
  int h = visual_estimator.imgHeight;

  // Framerate Measuring
  results->FPS = framerate_run();

  if (visual_estimator.old_img_init == 1) {
    memcpy(visual_estimator.prev_frame, frame, w * h * 2);
    CvtYUYV2Gray(visual_estimator.prev_gray_frame, visual_estimator.prev_frame, w, h);
    visual_estimator.old_img_init = 0;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection
  int fast_threshold = 20;
  xyFAST *pnts_fast;
  pnts_fast = fast9_detect((const byte *)visual_estimator.prev_gray_frame, w, h, w,
                           fast_threshold, &results->count);
  if (results->count > MAX_COUNT) { results->count = MAX_COUNT; }
  for (int i = 0; i < results->count; i++) {
    x[i] = pnts_fast[i].x;
    y[i] = pnts_fast[i].y;
  }
  free(pnts_fast);

  // Remove neighboring corners
  const float min_distance = 3;
  float min_distance2 = min_distance * min_distance;
  int labelmin[MAX_COUNT];
  for (int i = 0; i < results->count; i++) {
    for (int j = i + 1; j < results->count; j++) {
      // distance squared:
      float distance2 = (x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]);
      if (distance2 < min_distance2) {
        labelmin[i] = 1;
      }
    }
  }

  int count_fil = results->count;
  for (int i = results->count - 1; i >= 0; i--) {
    int remove_point = 0;

    if (labelmin[i]) {
      remove_point = 1;
    }

    if (remove_point) {
      for (int c = i; c < count_fil - 1; c++) {
        x[c] = x[c + 1];
        y[c] = y[c + 1];
      }
      count_fil--;
    }
  }

  if (count_fil > max_count) { count_fil = max_count; }
  results->count = count_fil;

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************
  CvtYUYV2Gray(visual_estimator.gray_frame, frame, w, h);

  opticFlowLK(visual_estimator.gray_frame, visual_estimator.prev_gray_frame, x, y,
              count_fil, w, h, new_x, new_y, status, 5, 100);

  results->flow_count = count_fil;
  for (int i = count_fil - 1; i >= 0; i--) {
    int remove_point = 1;

    if (status[i] && !(new_x[i] < borderx || new_x[i] > (w - 1 - borderx) ||
                       new_y[i] < bordery || new_y[i] > (h - 1 - bordery))) {
      remove_point = 0;
    }

    if (remove_point) {
      for (int c = i; c < results->flow_count - 1; c++) {
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
  results->diff_pitch = (info->theta - visual_estimator.prev_pitch) * h / FOV_H;
  results->diff_roll = (info->phi - visual_estimator.prev_roll) * w / FOV_W;
  visual_estimator.prev_pitch = info->theta;
  visual_estimator.prev_roll = info->phi;

  float OFx_trans, OFy_trans;
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
  OFx_trans = results->dx_sum;
  OFy_trans = results->dy_sum;
#endif

  // Average Filter
  OFfilter(&results->OFx, &results->OFy, OFx_trans, OFy_trans, results->flow_count, 1);

  // Velocity Computation
  if (info->agl < 0.01) {
    results->cam_h = 0.01;
  }
  else {
    results->cam_h = info->agl;
  }

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

  memcpy(visual_estimator.prev_frame, frame, w * h * 2);
  memcpy(visual_estimator.prev_gray_frame, visual_estimator.gray_frame, w * h);

}
