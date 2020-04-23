/*
 * Copyright (C) Ralph Rudi schmidt <ralph.r.schmidt@outlook.com>

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
/** @file "modules/wedgebug/wedgebug.h"
 * @author Ralph Rudi schmidt <ralph.r.schmidt@outlook.com>
 * An integration of the WegdeBug algorithm (Laubach 1999) for path finding, for drones with stereo vision.
 */
#ifndef WEDGEBUG_H
#define WEDGEBUG_H



// Including library for types
#include <stdint.h>

// Periodic-type functions
extern void wedgebug_init(void);
extern void wedgebug_periodic(void);

// Global variables - Defines as settings
extern int N_disparities;
extern int block_size_disparities;
extern int min_disparity;
extern int max_disparity;

// Global functions
void post_disparity_crop_rect(uint16_t* height_start, uint16_t* height_offset, uint16_t* width_start, uint16_t* width_offset, const uint16_t height_old,const uint16_t width_old, const int disp_n, const int block_size);





#endif  // WEDGEBUG_H
