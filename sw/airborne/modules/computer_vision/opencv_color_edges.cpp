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
#include "opencv_color_edges.h"
#include <stdio.h>
#include <time.h>

extern "C" {
#include "modules/computer_vision/lib/vision/image.h"
}


using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

RNG rng(12345);

struct obstacle opencv_color_edges(struct image_t *img,
		bool GRAY_SCALE,
		bool BLUR_IMAGE,int BLUR_SIZE_IMAGE,
		bool BLUR_EDGES,int BLUR_SIZE_EDGES,
		bool BORDERS,int BORDER_MARGIN,
		bool Y_UP_filter,int y_up_del,
		bool Y_DOWN_filter,int y_down_del,
		bool draw,
		int downsize_factor,
		int APS,
		double min_obs_size,double max_obs_size)
{
	struct obstacle new_obstacle;

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
	Mat image, contour_image, thresh_image,blur_image,edge_image,store_image;

	// convert UYVY in paparazzi to YUV in opencv
	cvtColor(M, store_image, CV_YUV2RGB_Y422);

	if (GRAY_SCALE == true) {
		cvtColor(store_image, store_image, CV_RGB2GRAY);
	}
	else {
		cvtColor(store_image, store_image, CV_RGB2YUV);
	}


	// Threshold all values within the indicted YUV values. (Change to CANNY edge detection)
	inRange(M, Scalar(lum_min,cb_min,cr_min), Scalar(lum_max,cb_max,cr_max), thresh_image);

	// Blur image
	blur(thresh_image, blur_image, Size(2, 2));
	// ----------------------------------------------

	// Find contours
	contour_image = blur_image;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(contour_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	//drawContours(M, contours, -1, Scalar(0, 255, 0), 2);


	if (contours.size()>0)
	{

		// Find Largest Contour
		int largest_contour_index = 0;
		int largest_area = 0;
		Rect rect;

		// iterate through each contour.
		for (unsigned int i = 0; i < contours.size(); i++) {
		//  Find the area of contour
		double a = contourArea(contours[i], false);
		if (a > min_obs_size*contour_image.cols*contour_image.rows){
			if (a > largest_area) {
			  largest_area = a;
			  // Store the index of largest contour
			  largest_contour_index = i;
			}
		}
		}

		/*
		// Get the moments and mass centers
		vector<Moments> mu(contours.size());
		vector<Point2f> mc(contours.size());
		for (unsigned int i = 0; i < contours.size(); i++) {
		mu[i] = moments(contours[i],true);
		mc[i] = Point2f(mu[i].m10 / mu[i].m00 , mu[i].m01 / mu[i].m00);
		}
		//new_obstacle.pos_x = mc[largest_contour_index].x*downsize_factor;
		//new_obstacle.pos_y = mc[largest_contour_index].y*downsize_factor;
		*/

		//printf("Moments %f %f %f\n",mu[largest_contour_index].m10,mu[largest_contour_index].m01,mu[largest_contour_index].m00);

		// Find the bounding rectangle for biggest contour
		rect = boundingRect(contours[largest_contour_index]);

		// Save in output variable
		new_obstacle.pos_x = (rect.x+rect.width/2)*downsize_factor;
		new_obstacle.pos_y = (rect.y+rect.height/2)*downsize_factor;
		new_obstacle.width = rect.width*downsize_factor;
		new_obstacle.height =rect.height*downsize_factor;
		new_obstacle.area = largest_area*downsize_factor*downsize_factor;

		//printf("Rect x %d y %d w %d h %d\n",rect.x+rect.width/2,rect.y+rect.height/2,rect.width,rect.height);
		//printf("Total x %d Total y %d\n",img->w,img->h);
		t = clock() - t;
		printf("VISION OBSTACLE x pos %d y pos %d width %d height %d area %d Time elapsed (s) %f\n",new_obstacle.pos_x,new_obstacle.pos_y,new_obstacle.width,new_obstacle.height,new_obstacle.area,((float)t/CLOCKS_PER_SEC));


		if (draw){

			// Draw center of obstacle (crosshair)
			struct point_t center;
			center.x = new_obstacle.pos_x;
			center.y = new_obstacle.pos_y;
			uint8_t color[4] = {255, 255, 255, 255};

			//Crosshair
			image_draw_crosshair(img, &center, color, 10);
			// Obstacle rectangle
			image_draw_rectangle(img,new_obstacle.pos_x,new_obstacle.pos_x+new_obstacle.width,new_obstacle.pos_y,new_obstacle.pos_y+new_obstacle.height,color);

		}

	} else{
		t = clock() - t;

		printf("VISION NO OBSTACLE Time elapsed (s) %f\n",((float)t/CLOCKS_PER_SEC));

		new_obstacle.pos_x = 0;
		new_obstacle.pos_y = 0;
		new_obstacle.width = 0;
		new_obstacle.height = 0;
		new_obstacle.area = 0;

	};


	return new_obstacle;

}
