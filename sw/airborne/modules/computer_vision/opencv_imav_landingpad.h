/*
 * Copyright (C) IMAV 2016
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
 * @file "modules/computer_vision/opencv_helipad.h"
 * @author IMAV 2016
 */

#ifndef OPENCV_IMAV_LANDINGPAD_H
#define OPENCV_IMAV_LANDINGPAD_H

#ifdef __cplusplus
extern "C" {
#endif

struct results {
    int maxx;
    int maxy;
    int marker;
};

struct results opencv_imav_landing(char *img, int width, int height, int v_squares, int binary_threshold, int mod);

#ifdef __cplusplus
}
#endif

#endif
