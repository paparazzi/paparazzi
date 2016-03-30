/*
 * Copyright (C) 2014 G. de Croon
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/lib/vision/lucas_kanade.h
 * @brief efficient fixed-point optical-flow calculation
 *
 * - Initial fixed-point C implementation by G. de Croon
 * - Algorithm: Lucas-Kanade by Yves Bouguet
 * - Publication: http://robots.stanford.edu/cs223b04/algo_tracking.pdf
 */

#ifndef OPTIC_FLOW_INT_H
#define OPTIC_FLOW_INT_H

#include "std.h"
#include "image.h"

struct flow_t *opticFlowLK(struct image_t *new_img, struct image_t *old_img, struct point_t *points,
                           uint16_t *points_cnt, uint16_t half_window_size,
                           uint16_t subpixel_factor, uint8_t max_iterations, uint8_t step_threshold, uint8_t max_points, uint8_t pyramid_level);

#endif /* OPTIC_FLOW_INT_H */
