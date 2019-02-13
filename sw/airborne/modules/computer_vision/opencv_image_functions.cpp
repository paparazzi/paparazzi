/*
 * Copyright (C) 2016 Roland Meertens
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/opencv_image_functions.cpp
 *
 * A small library with functions to convert between the Paparazzi used YUV422 arrays
 * and the opencv image functions.
 */

#include "opencv_image_functions.h"


using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

void coloryuv_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 3);

  int nRows = image.rows;
  int nCols = image.cols;

  int byte_index = 0;
  for(int r = 0; r < nRows; ++r) {
    for(int c = 0; c < nCols; ++c) {
      Vec3b yvu = image.at<Vec3b>(r, c);
      if((byte_index % 4) == 0) {
        img[byte_index++] = yvu.val[1]; // U
      } else {
        img[byte_index++] = yvu.val[2]; // V
      }
      img[byte_index++] = yvu.val[0]; // Y
    }
  }
}

void colorbgr_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  // Convert to YUV color space
  cvtColor(image, image, COLOR_BGR2YUV);
  // then call the to color function
  coloryuv_opencv_to_yuv422(image, img, width, height);
}


void grayscale_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 1);

  int n_rows = image.rows;
  int n_cols = image.cols;

  // If the image is one block in memory we can iterate over it all at once!
  if (image.isContinuous()) {
    n_cols *= n_rows;
    n_rows = 1;
  }

  // Iterate over the image, setting only the Y value
  // and setting U and V to 127
  int i, j;
  uchar *p;
  int index_img = 0;
  for (i = 0; i < n_rows; ++i) {
    p = image.ptr<uchar>(i);
    for (j = 0; j < n_cols; j++) {
      img[index_img++] = 127;
      img[index_img++] = p[j];


    }
  }
}
