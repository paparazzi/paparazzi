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


#define MAX_HORIZON 10
#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH 320
#define DISP_RANGE_MAX 20

struct edge_hist_t {
	int32_t horizontal[IMAGE_WIDTH];
	int32_t vertical[IMAGE_HEIGHT];
	int32_t frame_time;
	int16_t roll;
	int16_t pitch;
};
void test_function(struct image_t *image,struct image_t *image_gray);
void edgeflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img,
                          struct opticflow_result_t *result);
void calculate_edge_histogram(struct image_t *img, int32_t edge_histogram[],
		char direction, uint16_t edge_threshold);

#endif /* EDGE_FLOW_H_ */
