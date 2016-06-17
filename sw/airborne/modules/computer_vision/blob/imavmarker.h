/*
 * Copyright (C) 2015
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
 * @file "modules/computer_vision/blob/imavmarker.h"
 * Find a IMAV pattern
 */



#include "modules/computer_vision/lib/vision/image.h"


struct marker_deviation_t {
  int32_t x;
  int32_t y;
  uint16_t inlier;
};


struct marker_deviation_t marker(struct image_t *input, uint8_t M);
