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

/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);
static int cmp_flow(const void *a, const void *b);

/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 * @param[in] *w The image width
 * @param[in] *h The image height
 */
void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h)
{
  /* Create the image buffers */
  image_create(&opticflow->img_gray, w, h, IMAGE_GRAYSCALE);
  image_create(&opticflow->prev_img_gray, w, h, IMAGE_GRAYSCALE);

  /* Set the previous values */
  opticflow->got_first_img = FALSE;
  opticflow->prev_phi = 0.0;
  opticflow->prev_theta = 0.0;
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
  static uint8_t threshold = 20;
  struct point_t *corners = fast9_detect(img, threshold, 10, 20, 20, &result->corner_cnt);

  // Adaptive threshold
  if(result->corner_cnt < 40 && threshold > 5)
    threshold--;
  else if(result->corner_cnt > 50 && threshold < 60)
    threshold++;

#if OPTICFLOW_DEBUG && OPTICFLOW_SHOW_CORNERS
  image_show_points(img, corners, result->corner_cnt);
#endif

  // Check if we found some corners to track
  if(result->corner_cnt < 1) {
    free(corners);
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    return;
  }

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

#define MAX_TRACK_CORNERS 25
#define HALF_WINDOW_SIZE 5
#define SUBPIXEL_FACTOR 10
#define MAX_ITERATIONS 10
#define THRESHOLD_VEC 2
  // Execute a Lucas Kanade optical flow
  result->tracked_cnt = result->corner_cnt;
  struct flow_t *vectors = opticFlowLK(&opticflow->img_gray, &opticflow->prev_img_gray, corners, &result->tracked_cnt,
    HALF_WINDOW_SIZE, SUBPIXEL_FACTOR, MAX_ITERATIONS, THRESHOLD_VEC, MAX_TRACK_CORNERS);

#if OPTICFLOW_DEBUG && OPTICFLOW_SHOW_FLOW
  image_show_flow(img, vectors, result->tracked_cnt, SUBPIXEL_FACTOR);
#endif

  // Get the median flow
  qsort(vectors, result->tracked_cnt, sizeof(struct flow_t), cmp_flow);
  if(result->tracked_cnt == 0) {
    // We got no flow
    result->flow_x = 0;
    result->flow_y = 0;
  } else if(result->tracked_cnt > 3) {
    // Take the average of the 3 median points
    result->flow_x = vectors[result->tracked_cnt/2 -1].flow_x;
    result->flow_y = vectors[result->tracked_cnt/2 -1].flow_y;
    result->flow_x += vectors[result->tracked_cnt/2].flow_x;
    result->flow_y += vectors[result->tracked_cnt/2].flow_y;
    result->flow_x += vectors[result->tracked_cnt/2 +1].flow_x;
    result->flow_y += vectors[result->tracked_cnt/2 +1].flow_y;
    result->flow_x /= 3;
    result->flow_y /= 3;
  } else {
    // Take the median point
    result->flow_x = vectors[result->tracked_cnt/2].flow_x;
    result->flow_y = vectors[result->tracked_cnt/2].flow_y;
  }

  // Flow Derotation
  float diff_flow_x = (state->phi - opticflow->prev_phi) * img->w / FOV_W;
  float diff_flow_y = (state->theta - opticflow->prev_theta) * img->h / FOV_H;
  result->flow_der_x = result->flow_x - diff_flow_x * SUBPIXEL_FACTOR;
  result->flow_der_y = result->flow_y - diff_flow_y * SUBPIXEL_FACTOR;
  opticflow->prev_phi = state->phi;
  opticflow->prev_theta = state->theta;

  // Velocity calculation
  result->vel_x = -result->flow_der_x * result->fps / SUBPIXEL_FACTOR * img->w / Fx_ARdrone;
  result->vel_y =  result->flow_der_y * result->fps / SUBPIXEL_FACTOR * img->h / Fy_ARdrone;

  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************
  free(corners);
  free(vectors);
  image_switch(&opticflow->img_gray, &opticflow->prev_img_gray);
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

/**
 * Compare two flow vectors based on flow distance
 * Used for sorting.
 * @param[in] *a The first flow vector (should be vect flow_t)
 * @param[in] *b The second flow vector (should be vect flow_t)
 * @return Negative if b has more flow than a, 0 if the same and positive if a has more flow than b
 */
static int cmp_flow(const void *a, const void *b)
{
  const struct flow_t *a_p = (const struct flow_t *)a;
  const struct flow_t *b_p = (const struct flow_t *)b;
  return (a_p->flow_x*a_p->flow_x + a_p->flow_y*a_p->flow_y) - (b_p->flow_x*b_p->flow_x + b_p->flow_y*b_p->flow_y);
}
