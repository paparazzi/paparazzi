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
  enum image_type type;   // Type of image on which kernel is laid onto
  uint16_t w;           //s/< Kernel width
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


extern uint16_t K_median_h; // Height of kernel for the median kernel
extern uint16_t K_median_w; // Width of kernel for the median kernel
extern int SE_opening_OCV;  // SE size for the opening operation
extern int SE_closing_OCV;  // SE size for the closing operation
extern int
SE_dilation_OCV_1;// SE size for the first dilation operation (Decides where edges are detected, increase to increase drone safety zone NOTE. This functionality should be replaced with c space expansion)
extern int SE_dilation_OCV_2; // SE size for the second dilation operation (see state 6 "WEDGEBUG_START" )
extern int
SE_erosion_OCV;  // SE size for the erosion operation (see state 3 "WEDGEBUG_START" and state 6 "POSITION_EDGE", its needed to "drag" the depth of the foreground objects over the edges detected)


// Setting thresholds

extern uint16_t
threshold_median_depth;   //! Below this median depth (cm), an obstacle is considered to block the way (i.e. the blocking obstacle needs to be close)
extern uint16_t threshold_depth_of_edges; //! Below this depth (cm) edges are eligible for the WedgeBug algorith

extern int
threshold_edge_magnitude;        // Edges with a magnitude above this value are detected. Above this value, edges are given the value 127, otherwise they are given the value zero.
extern float threshold_distance_to_goal;    // Above this threshold, the goal is considered reached
extern float threshold_distance_to_angle;   // Above this threshold, the angle/heading is considered reached
extern int16_t max_obstacle_confidence;     // This is the max confidence that an obstacle was spotted
extern int16_t max_free_path_confidence;    // This is the max confidence that an obstacle was not spotted
extern int16_t max_position_confidence;     // This is the max confidence that a specific position was reached
extern int16_t max_heading_confidence;      // This is the max confidence that a specific heading was reached
extern int16_t max_edge_found_micro_confidence; // This is the max confidence that edges (micro-see above) were found
extern int16_t max_edge_found_macro_confidence; // This is the max confidence that edges (macro-see above were found
extern int16_t max_no_edge_found_confidence;  // This is the max confidence that no edges were found



extern int heat_map_type; // Heat map used when saving image
extern uint8_t save_images_flag;



// Global functions
extern void post_disparity_crop_rect(struct crop_t *_img_cropped_info, struct img_size_t *_original_img_dims,
                                     const int disp_n, const int block_size);
extern void set_state(uint8_t _state, uint8_t change_allowed);
void kernel_create(struct kernel_C1 *kernel, uint16_t width, uint16_t height, enum image_type type);
extern void kernel_free(struct kernel_C1 *kernel);
extern uint8_t getMedian(uint8_t *a, uint32_t n);








#endif  // WEDGEBUG_H
