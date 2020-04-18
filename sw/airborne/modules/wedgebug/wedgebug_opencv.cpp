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


#include "wedgebug_opencv.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
using namespace cv;





int save_image_gray(void *img, int width, int height, char *myString)
{
	// Create a new image, using the original bebop image.
	Mat M(height, width, CV_8UC1, img);
	// Savin image to my documents
	imwrite(myString, M);

	// Code below is for testing
	/*
	std::cout << "Rows/height (240): " << M.size[0] << std::endl;
	std::cout << "Columns/width (480): " << M.size[1] << "\n" << std::endl;
	std::cout << M.at(5,5)[0] << std::endl;

	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc(M ,&minVal, &maxVal, &minLoc, &maxLoc);
	std::cout << "Merged: Min=" << minVal << "; Max=" << maxVal << std::endl;
	 */
  return 0;
}



int save_image_color(void *img, int width, int height,char *myString)
{
	// Create a new image, using the original bebop image.
	Mat M(height, width, CV_8UC2, img); // CV_8UC2 is the openCV format for UYVY images
	// Definition of Mat: Mat::Mat(int _rows, int _cols, int _type, void* _data, size_t _step)
	// Remember that void takes any data type including char
	Mat image;
	cvtColor(M, image, CV_YUV2GRAY_Y422);
	imwrite(myString, image);

	// Code below is for testing


	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc(image ,&minVal, &maxVal, &minLoc, &maxLoc);
	std::cout << "Left1: Min=" << minVal << "; Max=" << maxVal << std::endl;

  return 0;
}

/*

int BM(void *img_left,void *img_right, void *img_output, int width, int height)
{
	return0


}
*/

int SBM(struct image_t *left, image_t *right, image_t *matched, int ndisparities, int SADWindowSize)
{

	Mat img_left(left->h, left->w, CV_8UC1, left->buf);
	Mat img_right(right->h, right->w, CV_8UC1, right->buf);
	Mat img_depth;
	Mat img_depth_norm;
	double minVal;
	double maxVal;



	Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);
	//sbm->setMinDisparity(0);
	sbm->compute(img_left, img_right, img_depth); //type of img_depth is CV_16U
	img_depth.convertTo(img_depth_norm, CV_8UC1);
	minMaxLoc(img_depth_norm ,&minVal, &maxVal);

	int min = 255;
	//int i = 0;

	// Demonstrating depth calculation
	// Get minimum value (which is not 0)
	for (int i =0; i < img_depth_norm.rows * img_depth_norm.cols; i++)
	{
		if (img_depth_norm.data[i] < min && img_depth_norm.data[i] != 0)
		{
			min = img_depth_norm.data[i];

		}
	}
	int depth = ((WEDGEBUG_CAMERA_BASELINE * WEDGEBUG_CAMERA_FOCAL_LENGTH) / min) / 10;
	std::cout << "max depth in cm: " << depth << std::endl;


	//img_depth.convertTo(img_depth_norm, CV_8UC1, 255/(maxVal - minVal));

	//std::cout << minVal << ":" << maxVal << std::endl;
	//std::cout << "Height : Width  - matched input: " << matched->h << " : " << matched->w << std::endl;
	//std::cout << "Height Width  - opencCV CV_8UC1: " << img_depth_norm.rows << " : " << img_depth_norm.cols << std::endl;

	for (int i = 0; i < (matched->w * matched->h); i++)
	{
		((uint8_t*)matched->buf)[i]  = img_depth_norm.data[i];
	}




	return 0;
}




