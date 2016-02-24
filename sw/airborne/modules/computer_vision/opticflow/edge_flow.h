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
#define DISP_RANGE_MAX 20
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

#endif /* EDGE_FLOW_H_ */
