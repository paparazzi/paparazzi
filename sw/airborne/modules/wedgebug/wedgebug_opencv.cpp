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

	/*
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc(image ,&minVal, &maxVal, &minLoc, &maxLoc);
	std::cout << "Left1: Min=" << minVal << "; Max=" << maxVal << std::endl;
	*/
  return 0;
}

