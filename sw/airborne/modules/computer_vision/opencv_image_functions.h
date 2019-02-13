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
 * @file modules/computer_vision/opencv_image_functions.h
 *
 * A small library with functions to convert between the Paparazzi used YUV422 arrays
 * and the opencv image functions.
 */

#ifndef OPENCV_IMAGE_FUNCTIONS_H
#define OPENCV_IMAGE_FUNCTIONS_H
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Converts cv::Mat with three channels to a YUV422 image.
 * Note that the rgb function first converts to YUV, and then to YUV422 making
 * this function slower than coloryuv_opencv_to_yuv422.
 */
void colorbgr_opencv_to_yuv422(cv::Mat image, char *img, int width, int height);

/**
 * Converts cv::Mat with three channels YUV to a YUV422 image.
 */
void coloryuv_opencv_to_yuv422(cv::Mat image, char *img, int width, int height);

/**
 * Converts cv::Mat with one to a YUV422 image.
 * The U and V channels are set to 127.
 */
void grayscale_opencv_to_yuv422(cv::Mat image, char *img, int width, int height);
#endif
