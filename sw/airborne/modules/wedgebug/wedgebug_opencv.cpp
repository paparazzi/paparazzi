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
#include <iostream>
#include <string>


using namespace cv;


string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}



int save_image_gray(void *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  //Mat M(height, width, CV_8UC2, img); // CV_8UC2 is the openCV format for UYVY images
  // Definition of Mat: Mat::Mat(int _rows, int _cols, int _type, void* _data, size_t _step)
	//Remembner that void takes any data type includin char
	//Mat image;
  //cvtColor(M, image, CV_YUV2GRAY_Y422);
  //imwrite( "Gray_Image.jpg", image);

	uint8_t* img2 = static_cast<uint8_t*>(img);
	Mat M(height, width, CV_8UC1, img2);


	imwrite( "merged_stereo_image.jpg", M);

	//std::cout << "Rows/height (240): " << M.size[0] << std::endl;
	//std::cout << "Columns/width (480): " << M.size[1] << "\n" << std::endl;
	//std::cout << M.at(5,5)[0] << std::endl;

	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc(M ,&minVal, &maxVal, &minLoc, &maxLoc);
	std::cout << "Merged: Min=" << minVal << "; Max=" << maxVal << std::endl;


	uint8_t min;
	uint8_t max;
	for(int i = 0; i < ((width * height)-1); i++){
		//std::cout << i << std::endl;
		if(i == 1){min = img2[i]; max = img2[i];}
		else{
			if(img2[i] < min){min = img2[i];}
			if(img2[i] > max){max = img2[i];}
		}
	}
	std::cout << "Merged - from buffer: Min=" << +min << "; Max=" << +max << std::endl;
/*
	unsigned char min2;
	unsigned char max2;
	for(int i = 0; i < ((M.cols * M.rows)-1); i++){
		if(i == 1){min2 = M.data[i]; max2 = M.data[i]; }
		else{
			if(M.data[i] < min){min2 = M.data[i];}
			if(M.data[i] > max){max2 = M.data[i];}
		}
		//if (i < 10){std::cout << +M.data[i] << std::endl;}
	}
	//std::cout << "Merged - from oCV buffer: Min=" << +min2 << "; Max=" << +max2 << std::endl;
	//std::cout << type2str(M.type()) << std::endl;

	/*
	for (int i = 0; i < M.rows - 1; i++){
		for (int j = 0; j < M.cols -1; j++){
			if (i < 10 && j < 10){std::cout << +M.at<unsigned char>(i, j) << std::endl;}
		}
	}
	*/

	uint8_t min3;
	uint8_t max3;

	for(int i = 0; i < ((M.cols * M.rows)-1); i++){
		M.data[i] = img2[i];

		if(i == 0){min3 = M.data[i]; max3 = M.data[i]; }
		else{
			if(M.data[i] < min3){min3 = M.data[i];}
			if(M.data[i] > max3){max3 = M.data[i];}
		}
		if(i <100){std::cout << +M.data[i] << ":" << +img2[i] << std::endl;}
		}

	std::cout << +img2[0] << std::endl;


	std::cout << "Merged - from oCV buffer: Min=" << +min3 << "; Max=" << +max3 << std::endl;

  return 0;
}


































int save_image_color(void *img, int width, int height)
{
	// Create a new image, using the original bebop image.
	Mat M(height, width, CV_8UC2, img); // CV_8UC2 is the openCV format for UYVY images
	// Definition of Mat: Mat::Mat(int _rows, int _cols, int _type, void* _data, size_t _step)
	// Remember that void takes any data type including char
	Mat image;
	cvtColor(M, image, CV_YUV2GRAY_Y422);
	imwrite( "Color_Image.jpg", image);

	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc(image ,&minVal, &maxVal, &minLoc, &maxLoc);
	std::cout << "Left1: Min=" << minVal << "; Max=" << maxVal << std::endl;


  return 0;
}

