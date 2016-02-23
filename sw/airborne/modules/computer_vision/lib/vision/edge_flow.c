/*
 * edge_flow.c
 *
 *  Created on: Feb 22, 2016
 *      Author: knmcguire
 */
#include <lib/vision/edge_flow.h>


void test_function(struct image_t *img,struct image_t *img_gray)
{
	image_to_grayscale(img, img_gray);

}

void edgeflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img,
		struct opticflow_result_t *result)
{

	static struct edge_hist_t edge_hist[MAX_HORIZON];
    static uint8_t current_frame_nr = 1;

	image_to_grayscale(img, &opticflow->img_gray);
	// Copy to previous image if not set
	if (!opticflow->got_first_img) {
		image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
		opticflow->got_first_img = TRUE;
	}
	int32_t *edge_hist_x = edge_hist[current_frame_nr].horizontal;
	int32_t *edge_hist_y = edge_hist[current_frame_nr].vertical;

	calculate_edge_histogram(img, edge_hist_x, 'x',0);
	calculate_edge_histogram(img, edge_hist_y, 'y',0);


	uint8_t previous_frame_x = (current_frame_nr - 5 + MAX_HORIZON) %
			MAX_HORIZON; // wrap index
	uint8_t previous_frame_y = (current_frame_nr - 5 + MAX_HORIZON) %
			MAX_HORIZON; // wrap index

	uint16_t i;
	struct point_t point1;
	struct point_t point2;
	struct point_t point1_prev;
	struct point_t point2_prev;


	for(i = 120; i<240;i++)
	{
		point1.y = -(uint16_t)edge_hist_x[i]/100 + img->h/3;
		point1.x = i;
		point2.y = -(uint16_t)edge_hist_x[i+1]/100 + img->h/3;
		point2.x = i+1;

		point1_prev.y = -(uint16_t)edge_hist[previous_frame_x].horizontal[i]/100 + img->h*2/3;
		point1_prev.x = i;
		point2_prev.y = -(uint16_t)edge_hist[previous_frame_x].horizontal[i+1]/100 + img->h*2/3;
		point2_prev.x = i+1;

		image_draw_line(img, &point1,&point2);
		image_draw_line(img, &point1_prev,&point2_prev);

	}

	current_frame_nr = (current_frame_nr + 1) % MAX_HORIZON;
}



// calculate_edge_histogram calculates the image gradient of the images and makes a edge feature histogram
void calculate_edge_histogram(struct image_t *img, int32_t edge_histogram[],
		char direction, uint16_t edge_threshold)
{
	  uint8_t *img_buf = (uint8_t *)img->buf;

	// TODO use arm_conv_q31()
	int32_t sobel_sum = 0;
	int32_t Sobel[3] = { -1, 0, 1};

	uint32_t y = 0, x = 0;
	int32_t c = 0;

	uint32_t idx = 0;

	uint16_t image_width = img->w;
	uint16_t image_height = img->h;
	uint32_t interlace;
	if(img->type == IMAGE_GRAYSCALE)
		interlace = 1;
	else {
		if(img->type == IMAGE_YUV422)
			interlace = 2;
		else
			while (1);   // hang to show user something isn't right
	}


	// compute edge histogram
	if (direction == 'x') {
		// set values that are not visited
		edge_histogram[0] = edge_histogram[image_width - 1] = 0;
		for (x = 1; x < image_width - 1; x++) {
			edge_histogram[x] = 0;
			for (y = 0; y < image_height; y++) {
				sobel_sum = 0;

				for (c = -1; c <= 1; c++) {
					idx = interlace * (image_width * y + (x + c)); // 2 for interlace

					sobel_sum += Sobel[c + 1] * (int32_t)img_buf[idx+1];
				}
				sobel_sum = abs(sobel_sum);
				if (sobel_sum > edge_threshold) {
					edge_histogram[x] += sobel_sum;
				}
			}
		}
	} else if (direction == 'y') {
		// set values that are not visited
		edge_histogram[0] = edge_histogram[image_height - 1] = 0;
		for (y = 1; y < image_height - 1; y++) {
			edge_histogram[y] = 0;
			for (x = 0; x < image_width; x++) {
				sobel_sum = 0;

				for (c = -1; c <= 1; c++) {
					idx = interlace * (image_width * (y + c) + x); // 2 for interlace

					sobel_sum += Sobel[c + 1] * (int32_t)img_buf[idx+1];
				}
				sobel_sum = abs(sobel_sum);
				if (sobel_sum > edge_threshold) {
					edge_histogram[y] += sobel_sum;
				}
			}
		}
	} else
		while (1);  // hang to show user something isn't right
}


