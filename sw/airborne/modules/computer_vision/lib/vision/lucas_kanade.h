/*
 * Copyright (C) 2014
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/cv/opticflow/lucas_kanade.h
 * @brief efficient fixed-point optical-flow
 *
 */

#ifndef OPTIC_FLOW_INT_H
#define OPTIC_FLOW_INT_H

#include "std.h"

void multiplyImages(int *ImA, int *ImB, int *ImC, int width, int height);
void getImageDifference(int *ImA, int *ImB, int *ImC, int width, int height);
void getSubPixel_gray(int *Patch, unsigned char *frame_buf, int center_x, int center_y, int half_window_size,
                      int subpixel_factor);
void getGradientPatch(int *Patch, int *DX, int *DY, int half_window_size);
int getSumPatch(int *Patch, int size);
int calculateG(int *G, int *DX, int *DY, int half_window_size);
int calculateError(int *ImC, int width, int height);
int opticFlowLK(unsigned char *new_image_buf, unsigned char *old_image_buf, uint16_t *p_x, uint16_t *p_y, uint16_t n_found_points,
                uint16_t imW, uint16_t imH, uint16_t *new_x, uint16_t *new_y, bool_t *status, uint16_t half_window_size, uint8_t max_iterations);
void OFfilter(float *OFx, float *OFy, float dx, float dy, int count, int OF_FilterType);

#endif /* OPTIC_FLOW_INT_H */
