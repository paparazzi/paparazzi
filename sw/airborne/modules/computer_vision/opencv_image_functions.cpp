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

void color_opencv_to_yuv422(Mat image, char *img, int width, int height)
{

//Turn the opencv RGB colored image back in a YUV colored image for the drone
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      cv::Vec3b pixelHere = image.at<cv::Vec3b>(row, col);
      img[(row * width + col) * 2 + 1] = 0.299 * pixelHere[0] + 0.587 * pixelHere[1] + 0.114 * pixelHere[2];
      if (col % 2 == 0) { // U
        img[(row * width + col) * 2] = 0.492 * (pixelHere[2] - img[(row * width + col) * 2 + 1] + 127);
      } else { // V
        img[(row * width + col) * 2] = 0.877 * (pixelHere[0] - img[(row * width + col) * 2 + 1] + 127);
      }
    }
  }
}


void grayscale_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {

      img[(row * width + col) * 2 + 1] =   image.at<uint8_t>(row, col);
      img[(row * width + col) * 2 ] = 127;
    }
  }
}
