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
 * @file "modules/computer_vision/cv_opencvdemo.h"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include <stdbool.h>

#ifndef OPENCV_COLOR_EDGES_H
#define OPENCV_COLOR_EDGES_H

#ifdef __cplusplus
extern "C" {
#endif

struct obstacle{
	int pos_x;
	int pos_y;
	int width;
	int height;
	int area;
};

struct obstacle opencv_color_edges(char *img, int width, int height,int lum_min, int lum_max,
        int cb_min, int cb_max,
        int cr_min, int cr_max,bool draw);

#ifdef __cplusplus
}
#endif

#endif

