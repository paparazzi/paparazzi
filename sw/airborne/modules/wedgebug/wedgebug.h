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
#include "modules/computer_vision/lib/vision/image.h"// For image-related structures

// Periodic-type functions
extern void wedgebug_init(void);
extern void wedgebug_periodic(void);


// Structures
/* Kernel - processes single channel images */
struct kernel_C1 {
	  uint16_t w;           ///< Kernel width
	  uint16_t h;           ///< Kernel height
	  uint32_t buf_size;    ///< Size of values of weight buffer and values buffer
	  void *buf_weights;    ///< Kernel weight buffer
	  void *buf_values;     ///< Kernel value buffer. These are the values underneath the kernel
};


// Global variables - Defines as settings
extern int N_disparities;
extern int block_size_disparities;
extern int min_disparity;
extern int max_disparity;

// Global functions
extern void post_disparity_crop_rect(struct crop_t *img_cropped_info,struct img_size_t *original_img_dims,const int disp_n,const int block_size);
extern void set_state(uint8_t state, uint8_t change_allowed);
extern void kernel_create(struct kernel_C1 *kernel, uint16_t width, uint16_t height);
extern void kernel_free(struct kernel_C1 *kernel);
extern uint8_t getMedian(uint8_t *a, uint32_t n);








#endif  // WEDGEBUG_H
