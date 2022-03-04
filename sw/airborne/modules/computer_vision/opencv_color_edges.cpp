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


#include "opencv_color_edges.h"
#include <stdio.h>
//#include <stdbool.h>

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

RNG rng(12345);

struct obstacle opencv_color_edges(char *img, int width, int height,int lum_min, int lum_max,
        int cb_min, int cb_max,
        int cr_min, int cr_max,bool draw)
{
	struct obstacle new_obstacle;

	// Create a new image, using the original bebop image.
	Mat M(width, height, CV_8UC2, img); // original
	Mat image, contour_image, thresh_image,edge_image;

	// convert UYVY in paparazzi to YUV in opencv
	cvtColor(M, M, CV_YUV2RGB_Y422);
	cvtColor(M, M, CV_RGB2YUV);

	// Threshold all values within the indicted YUV values.
	inRange(M, Scalar(lum_min,cb_min,cr_min), Scalar(lum_max,cb_max,cr_max), thresh_image);

	blur(thresh_image, thresh_image, Size(2, 2));

	edge_image = thresh_image;
	//int edgeThresh = 35;
	//Canny(edge_image, edge_image, edgeThresh, edgeThresh * 3);
	// Find contours
	contour_image = edge_image;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(contour_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	if (contours.size()>0)
	{

	//printf("Found contours %d\n",int(contours.size()));

	// Find Largest Contour
	int largest_contour_index = 0;
	int largest_area = 0;
	Rect bounding_rect;

	// iterate through each contour.
	for (unsigned int i = 0; i < contours.size(); i++) {
	//  Find the area of contour
	double a = contourArea(contours[i], false);
	if (a > largest_area) {
	  largest_area = a;
	  // Store the index of largest contour
	  largest_contour_index = i;
	  // Find the bounding rectangle for biggest contour
	  bounding_rect = boundingRect(contours[i]);
	}
	}
	Scalar color(255, 255, 255);

	if (draw){
	// Draw the contour and rectangle
	rectangle(M, bounding_rect,  Scalar(0, 255, 0), 2, 8, 0);
	}

	// Get the moments
	vector<Moments> mu(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++) {
	mu[i] = moments(contours[i], false);
	}

	//  Get the mass centers:
	vector<Point2f> mc(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++) {
	mc[i] = Point2f(mu[i].m10 / mu[i].m00 , mu[i].m01 / mu[i].m00);
	}


	printf("Largest contour %d\n",largest_area);
	//printf("Bounding_rect %d %d %d %d\n",bounding_rect.x,bounding_rect.y,bounding_rect.height,bounding_rect.width);
	//printf("index %d  \n",largest_contour_index);
	//printf("Center %f %f\n",mc[largest_contour_index].x,mc[largest_contour_index].y);

	if (draw){
	coloryuv_opencv_to_yuv422(M, img, width, height);
	//grayscale_opencv_to_yuv422(thresh_image, img, width, height);
	//colorbgr_opencv_to_yuv422(drawing, img, width, height);
	}

	new_obstacle.pos_x = mc[largest_contour_index].x;
	new_obstacle.pos_y = mc[largest_contour_index].y;
	new_obstacle.width = bounding_rect.width;
	new_obstacle.height = bounding_rect.height;
	new_obstacle.area = largest_area;

	} else{
		printf("No contour\n");

		new_obstacle.pos_x = 0;
		new_obstacle.pos_y = 0;
		new_obstacle.width = 0;
		new_obstacle.height = 0;
		new_obstacle.area = 0;

	};

	return new_obstacle;

}
