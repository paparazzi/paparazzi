/*
 * edge_flow.h
 *
 *  Created on: Feb 22, 2016
 *      Author: knmcguire
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
#define MAX_HORIZON 10
#endif
#ifndef DISP_RANGE_MAX
#define DISP_RANGE_MAX 50
#endif
#ifndef MAX_WINDOW_SIZE
#define MAX_WINDOW_SIZE 20
#endif
#ifndef IMAGE_HEIGHT
#define IMAGE_HEIGHT 240
#endif
#ifndef IMAGE_WIDTH
#define IMAGE_WIDTH  360
#endif
#ifndef OPTICFLOW_FOV_W
#define OPTICFLOW_FOV_W 0.89360857702
#endif
#ifndef OPTICFLOW_FOV_H
#define OPTICFLOW_FOV_H 0.67020643276
#endif

struct edge_hist_t {
  int32_t x[IMAGE_WIDTH];
  int32_t y[IMAGE_HEIGHT];
  struct timeval frame_time;
  float roll;
  float pitch;
};

struct edgeflow_displacement_t {
  int32_t x[IMAGE_WIDTH];
  int32_t y[IMAGE_HEIGHT];
};

struct edge_flow_t {
  int32_t flow_x;
  int32_t div_x;
  int32_t flow_y;
  int32_t div_y;
};

void edgeflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img,
                         struct opticflow_result_t *result);
// Local functions of the EDGEFLOW algorithm
void draw_edgeflow_img(struct image_t *img, struct edge_flow_t edgeflow, struct edgeflow_displacement_t displacement,
                       int32_t *edge_hist_x);
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
