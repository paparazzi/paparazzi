/*
 * Copyright (C) 2016 Kimberly McGuire <k.n.mcguire@tudelft.nl
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
 * @file modules/computer_vision/lib/vision/edge_flow.h
 * @brief calculate optical flow with EdgeFlow
 *
 * Edge-histogram matching, implementation by K. N. McGuire
 * Publication: Local Histogram Matching for Efficient Optical Flow Computation Applied to Velocity Estimation on Pocket Drones
 * by K.N. McGuire et al. (2016), ICRA 2016
 */

#ifndef EDGE_FLOW_H_
#define EDGE_FLOW_H_


#include "std.h"
#include "opticflow/inter_thread_data.h"
#include "lib/vision/image.h"
#include "lib/v4l/v4l2.h"
#include "opticflow/opticflow_calculator.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


#ifndef MAX_HORIZON
#define MAX_HORIZON 2
#else
#if MAX_HORIZON < 2
#define MAX_HORIZON 2
#endif
#endif
#ifndef DISP_RANGE_MAX
#define DISP_RANGE_MAX 50
#endif
#ifndef MAX_WINDOW_SIZE
#define MAX_WINDOW_SIZE 20
#endif
#ifndef OPTICFLOW_FOV_W
#define OPTICFLOW_FOV_W 0.89360857702
#endif
#ifndef OPTICFLOW_FOV_H
#define OPTICFLOW_FOV_H 0.67020643276
#endif

struct edge_hist_t {
  int32_t *x;
  int32_t *y;
  struct timeval frame_time;
  struct FloatRates rates;
};

struct edgeflow_displacement_t {
  int32_t *x;
  int32_t *y;
};

struct edge_flow_t {
  int32_t flow_x;
  int32_t div_x;
  int32_t flow_y;
  int32_t div_y;
};


// Local functions of the EDGEFLOW algorithm
void draw_edgeflow_img(struct image_t *img, struct edge_flow_t edgeflow, int32_t *edge_hist_x_prev
                       , int32_t *edge_hist_x);
void calc_previous_frame_nr(struct opticflow_result_t *result, struct opticflow_t *opticflow, uint8_t current_frame_nr,
                            uint8_t *previous_frame_offset, uint8_t *previous_frame_nr);
void calculate_edge_histogram(struct image_t *img, int32_t edge_histogram[],
                              char direction, uint16_t edge_threshold);
void calculate_edge_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
                                 uint16_t size,
                                 uint8_t window, uint8_t disp_range, int32_t der_shift);

// Local assisting functions (only used here)
// TODO: find a way to incorperate/find these functions in paparazzi
uint32_t timeval_diff2(struct timeval *starttime, struct timeval *finishtime);
uint32_t getMinimum(uint32_t *a, uint32_t n);
void line_fit(int32_t *displacement, int32_t *divergence, int32_t *flow, uint32_t size, uint32_t border,
              uint16_t RES);
uint32_t getAmountPeaks(int32_t *edgehist, uint32_t median, int32_t size);


#endif /* EDGE_FLOW_H_ */
