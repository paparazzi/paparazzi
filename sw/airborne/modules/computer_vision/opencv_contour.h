/*
 * Copyright (C) 2016 Peng Lu
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
 * @file modules/computer_vision/opencv_contour.h
 *
 * A small library with functions to convert between the Paparazzi used YUV422 arrays
 * and the opencv image functions.
 */

#ifndef OPENCV_IMAGE_FUNCTIONS_H
#define OPENCV_IMAGE_FUNCTIONS_H
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void find_contour(char *img, int width, int height);
  
#endif
