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

#include "mcu_periph/sys_time.h"

#define MAX_HORIZON 10
#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH 320
#define DISP_RANGE_MAX 20
#ifndef OPTICFLOW_FOV_W
#define OPTICFLOW_FOV_W 0.89360857702
#endif

#ifndef OPTICFLOW_FOV_H
#define OPTICFLOW_FOV_H 0.67020643276
#endif

struct edge_hist_t {
	int32_t horizontal[IMAGE_WIDTH];
	int32_t vertical[IMAGE_HEIGHT];
	struct timeval frame_time;
	float roll;
	float pitch;
};

struct edgeflow_displacement_t {
	int32_t horizontal[IMAGE_WIDTH];
	int32_t vertical[IMAGE_HEIGHT];
};

struct edge_flow_t {
	int32_t horizontal_flow;
	int32_t horizontal_div;
	int32_t vertical_flow;
	int32_t vertical_div;
};


void line_fit(int32_t *displacement, int32_t *divergence, int32_t *flow, uint32_t size, uint32_t border,
		uint16_t RES);
void test_function(struct image_t *image,struct image_t *image_gray);
void edgeflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img,
                          struct opticflow_result_t *result);
void calculate_edge_histogram(struct image_t *img, int32_t edge_histogram[],
		char direction, uint16_t edge_threshold);
void calculate_edge_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
		uint16_t size, uint8_t window, uint8_t disp_range, int32_t der_shift);
uint32_t getMinimum(uint32_t *a, uint32_t n);
void draw_edgeflow_img(struct image_t *img, struct edge_flow_t edgeflow, struct edgeflow_displacement_t displacement, int32_t *edge_hist_x);

#endif /* EDGE_FLOW_H_ */
