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
	bool updated;
};

void opencv_color_edges(struct obstacle *new_obstacle, struct image_t *img,
		bool GRAY_SCALE,
		bool BLUR_IMAGE,int BLUR_SIZE_IMAGE,
		bool BLUR_EDGES,int BLUR_SIZE_EDGES,
		bool BORDERS,int BORDER_MARGIN,
		bool Y_UP_filter,int y_up_del,
		bool Y_DOWN_filter,int y_down_del,
		int thresholdmin,int thresholdmax,
		int kernal_size,
		int max_number_obsticals,
		bool draw,
		int downsize_factor,
		double min_obs_size,double max_obs_size);

#ifdef __cplusplus
}
#endif

#endif

