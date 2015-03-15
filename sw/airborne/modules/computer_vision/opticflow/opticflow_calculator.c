/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/opticflow/opticflow_calculator.c
 * @brief Estimate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */

#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "opticflow_calculator.h"

// Computer Vision
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"

// ARDrone Vertical Camera Parameters
#define FOV_H 0.67020643276
#define FOV_W 0.89360857702
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053

// Corner Detection
#define MAX_COUNT 100

// Flow Derotation
#define FLOW_DEROTATION

/* Usefull for calculating FPS */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);

/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 * @param[in] *w The image width
 * @param[in] *h The image height
 */
void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h)
{
  /* Set the width/height of the image */
  opticflow->img_w = w;
  opticflow->img_h = h;

  /* Create the image buffers */
  image_create(&opticflow->img_gray, w, h, IMAGE_GRAYSCALE);
  image_create(&opticflow->prev_img_gray, w, h, IMAGE_GRAYSCALE);

  /* Set the previous values */
  opticflow->got_first_img = FALSE;
  opticflow->prev_pitch = 0.0;
  opticflow->prev_roll = 0.0;
}

/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
void opticflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img, struct opticflow_result_t *result)
{
  // Update FPS for information
  result->fps = 1 / (timeval_diff(&opticflow->prev_timestamp, &img->ts) / 1000.);
  memcpy(&opticflow->prev_timestamp, &img->ts, sizeof(struct timeval));

  // Convert image to grayscale
  image_to_grayscale(img, &opticflow->img_gray);

  // Copy to previous image if not set
  if (!opticflow->got_first_img) {
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    opticflow->got_first_img = TRUE;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection (TODO: non fixed threashold)
  struct point_t *fast9_points = fast9_detect(img, 20, 5, &result->corner_cnt);

  /*image_to_grayscale(img, img);
  uint8_t *im = (uint8_t *)img->buf;
  for(int i = 0; i < result->corner_cnt; i++) {
    uint32_t idx = 2*fast9_points[i].y*opticflow->img_w + fast9_points[i].x*2;
    im[idx] = 255;
    idx = idx+1 % (opticflow->img_w*opticflow->img_h*2);
    im[idx] = 255;
  }*/


  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

  struct point_t *new_points = malloc(sizeof(struct point_t) * result->corner_cnt);
  bool_t *tracked_points = malloc(sizeof(bool_t) * result->corner_cnt);
  opticFlowLK(&opticflow->img_gray, &opticflow->prev_img_gray, fast9_points, result->corner_cnt,
    new_points, tracked_points, 5, 100, 2);

  // Remove points if we lost tracking
 /* for (int i = count_fil - 1; i >= 0; i--) {
    if (!status[i] || new_x[i] < borderx || new_x[i] > (opticflow->img_w - 1 - borderx) ||
                       new_y[i] < bordery || new_y[i] > (opticflow->img_h - 1 - bordery)) {
      for (int c = i; c < result->flow_count - 1; c++) {
        x[c] = x[c + 1];
        y[c] = y[c + 1];
        new_x[c] = new_x[c + 1];
        new_y[c] = new_y[c + 1];
      }
      result->flow_count--;
    }
  }*/

  /*result->dx_sum = 0.0;
  result->dy_sum = 0.0;

  // Optical Flow Computation
  for (int i = 0; i < result->flow_count; i++) {
    dx[i] = new_x[i] - x[i];
    dy[i] = new_y[i] - y[i];
  }

  // Median Filter
  if (result->flow_count) {
    quick_sort_int(dx, result->flow_count); // 11
    quick_sort_int(dy, result->flow_count); // 11

    result->dx_sum = (float) dx[result->flow_count / 2];
    result->dy_sum = (float) dy[result->flow_count / 2];
  } else {
    result->dx_sum = 0.0;
    result->dy_sum = 0.0;
  }*/

  // Flow Derotation
  /*result->diff_pitch = (state->theta - opticflow->prev_pitch) * opticflow->img_h / FOV_H;
  result->diff_roll = (state->phi - opticflow->prev_roll) * opticflow->img_w / FOV_W;
  opticflow->prev_pitch = state->theta;
  opticflow->prev_roll = state->phi;

  float OFx_trans, OFy_trans;
#ifdef FLOW_DEROTATION
  if (result->flow_count) {
    OFx_trans = result->dx_sum - result->diff_roll;
    OFy_trans = result->dy_sum - result->diff_pitch;

    if ((OFx_trans <= 0) != (result->dx_sum <= 0)) {
      OFx_trans = 0;
      OFy_trans = 0;
    }
  } else {
    OFx_trans = result->dx_sum;
    OFy_trans = result->dy_sum;
  }
#else
  OFx_trans = result->dx_sum;
  OFy_trans = result->dy_sum;
#endif

  // Average Filter
  OFfilter(&result->OFx, &result->OFy, OFx_trans, OFy_trans, result->flow_count, 1);

  // Velocity Computation
  if (state->agl < 0.01) {
    result->cam_h = 0.01;
  }
  else {
    result->cam_h = state->agl;
  }

  if (result->flow_count) {
    result->Velx = result->OFy * result->cam_h * result->fps / Fy_ARdrone + 0.05;
    result->Vely = -result->OFx * result->cam_h * result->fps / Fx_ARdrone - 0.1;
  } else {
    result->Velx = 0.0;
    result->Vely = 0.0;
  }

  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************
  */
  free(fast9_points);
  free(new_points);
  free(tracked_points);
  image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
}

/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec=(finishtime->tv_sec-starttime->tv_sec)*1000;
  msec+=(finishtime->tv_usec-starttime->tv_usec)/1000;
  return msec;
}
