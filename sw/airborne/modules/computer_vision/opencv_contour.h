/*
 * Copyright (C) 2016 Roland Meertens and Peng Lu
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
 * Detects contours of an obstacle used in the autonomous drone racing.
 */

#ifndef OPENCV_CONTOUR_H
#define OPENCV_CONTOUR_H

#ifdef __cplusplus
extern "C" {
#endif

struct contour_estimation {
  float contour_d_x;
  float contour_d_y;
  float contour_d_z;
};

struct contour_threshold {
  int lower_y, upper_y, lower_u, upper_u, lower_v, upper_v;
};

extern struct contour_estimation cont_est;
extern struct contour_threshold cont_thres;

void find_contour(char *img, int width, int height);

#ifdef __cplusplus
}
#endif

#endif
