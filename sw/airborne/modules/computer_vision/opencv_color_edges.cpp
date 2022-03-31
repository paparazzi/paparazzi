/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

// CHANGES IN INPUTS AND FUNCTION NAMES result in changes in the conf files and function files "opencv_color_edges"
#include <opencv_color_edges.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
extern "C" {
#include "modules/computer_vision/lib/vision/image.h"
}


using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

RNG rng(12345);

void opencv_color_edges(struct obstacle *new_obstacle,struct image_t *img,
		bool GRAY_SCALE,
		bool BLUR_IMAGE,int BLUR_SIZE_IMAGE,
		bool BLUR_EDGES,int BLUR_SIZE_EDGES,
		bool BORDERS,int BORDER_MARGIN,
		bool Y_UP_filter,int y_up_del,
		bool Y_DOWN_filter,int y_down_del,
		int thresholdmin,int thresholdmax,
		int kernal_size,
		int max_number_obsticals,
		bool draw,
		int downsize_factor,
		double min_obs_size,double max_obs_size)
{
	// Time process
	clock_t t;
	t = clock();

	// Downsample image
	struct image_t img_small;
	image_create(&img_small,img->w/downsize_factor,img->h/downsize_factor,IMAGE_YUV422);
	image_yuv422_downsample(img,&img_small,downsize_factor);

	// ADD IF statements to gray scale and blur
	// ----------------------------------------------
	// Create a new image, using the original bebop image.
	Mat M(img_small.h, img_small.w, CV_8UC2, img_small.buf); // original
	Mat image, contour_image,store_image,thresh_image,blur_image,edge_image; //, thresh_image,blur_image,edge_image

	// convert UYVY in paparazzi to YUV in opencv
	cvtColor(M, store_image, CV_YUV2RGB_Y422);

	// Gray scale image
	if (GRAY_SCALE == true) {
		cvtColor(store_image, thresh_image, CV_RGB2GRAY);
	}
	else {
		cvtColor(store_image,thresh_image, CV_RGB2YUV);
	};

	// Blur image
	if (BLUR_IMAGE == true) {
		blur(thresh_image, blur_image, Size(BLUR_SIZE_IMAGE, BLUR_SIZE_IMAGE));
		thresh_image = blur_image;
	};

	// edge detect (output 0 or 255 image)
	Canny(thresh_image, edge_image, thresholdmin,thresholdmax,kernal_size);

	// Add borders to edge image with margin to avoid detection of entire image
	if (BORDERS == true) {
		for (int i = (0 + BORDER_MARGIN); i < (edge_image.rows  - BORDER_MARGIN); i++){
			edge_image.at<uchar>(i,0) = i;
			edge_image.at<uchar>(i,edge_image.cols-1) = i;
		};

		for (int i = (0 + BORDER_MARGIN); i < (edge_image.cols - BORDER_MARGIN); i++){
			edge_image.at<uchar>(0,i) = 1;
			edge_image.at<uchar>(edge_image.rows-1,i) = 1;
		};

	}

	// blur edges for better contour detection
	if (BLUR_EDGES == true) {
		blur(edge_image, edge_image, Size(BLUR_SIZE_EDGES, BLUR_SIZE_EDGES));
	};

	// Find contours
	contour_image = edge_image;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(contour_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	if (contours.size()>0)
	{

		// Find Largest Contour
		int largest_contour_index[max_number_obsticals] ;
		int largest_area[max_number_obsticals] ;
		double a;
		int Change;
		Rect rect;

		for (int i = 0; i < max_number_obsticals; i++) {
			largest_area[i] = 0;
		}

		// iterate through each contour.
		for (int i = 0; i < contours.size(); i++) {
		//  Find the area of contour
		a = contourArea(contours[i], false);
		if (a > min_obs_size*contour_image.cols*contour_image.rows && a < min_obs_size*contour_image.cols*contour_image.rows){
			Change = 0;
			for (int j = 0; j < max_number_obsticals; j++) {
				if (a>largest_area[j] && Change == 0) {
					largest_area[j] = (int)a;
					largest_contour_index[j] = i;
					Change = 1;
				}
			}
		}
		}


		for (int i = 0; i < max_number_obsticals; i++) {
			if (largest_area[i] == 0){

				// Save in output variable
				new_obstacle[i].pos_x = 0;
				new_obstacle[i].pos_y = 0;
				new_obstacle[i].width = 0;
				new_obstacle[i].height = 0;
				new_obstacle[i].area = 0;
			}
			else {
				// Find the bounding rectangle for biggest contour
				rect = boundingRect(contours[largest_contour_index[i]]);

				// Save in output variable
				new_obstacle[i].pos_x = (rect.x+rect.width/2)*downsize_factor;
				new_obstacle[i].pos_y = (rect.y+rect.height/2)*downsize_factor;
				new_obstacle[i].width = rect.width*downsize_factor;
				new_obstacle[i].height =rect.height*downsize_factor;
				new_obstacle[i].area = largest_area[i]*downsize_factor*downsize_factor;
				if (Y_UP_filter == true && new_obstacle[i].pos_x < y_up_del*downsize_factor){
					new_obstacle[i].pos_x = 0;
					new_obstacle[i].pos_y = 0;
					new_obstacle[i].width = 0;
					new_obstacle[i].height = 0;
					new_obstacle[i].area = 0;
				}
				if (Y_DOWN_filter == true && new_obstacle[i].pos_x > y_down_del*downsize_factor){
					new_obstacle[i].pos_x = 0;
					new_obstacle[i].pos_y = 0;
					new_obstacle[i].width = 0;
					new_obstacle[i].height = 0;
					new_obstacle[i].area = 0;
				}

			}
		}

		t = clock() - t;
		for (int i = 0; i < max_number_obsticals; i++) {
			if (largest_area[i] =! 0){
				printf("VISION OBSTACLE x pos %d y pos %d width %d height %d area %d Time elapsed (s) %f\n",new_obstacle[i].pos_x,new_obstacle[i].pos_y,new_obstacle[i].width,new_obstacle[i].height,new_obstacle[i].area,((float)t/CLOCKS_PER_SEC));
			}
		}

		if (draw){
			for (int i = 0; i < max_number_obsticals; i++) {
				if (largest_area[i] != 0){
					// Draw center of obstacle (crosshair)
					struct point_t center;
					center.x = new_obstacle[i].pos_x;
					center.y = new_obstacle[i].pos_y;
					uint8_t color[4] = {255, 255, 255, 255};

					//Crosshair
					image_draw_crosshair(img, &center, color, 10);
					// Obstacle rectangle
					image_draw_rectangle(img,new_obstacle[i].pos_x,new_obstacle[i].pos_x+new_obstacle[i].width,new_obstacle[i].pos_y,new_obstacle[i].pos_y+new_obstacle[i].height,color);
				}
			}
		}
	} else{
		t = clock() - t;

		printf("VISION NO OBSTACLE Time elapsed (s) %f\n",((float)t/CLOCKS_PER_SEC));

		new_obstacle[0].pos_x = 0;
		new_obstacle[0].pos_y = 0;
		new_obstacle[0].width = 0;
		new_obstacle[0].height = 0;
		new_obstacle[0].area = 0;

	};

}
