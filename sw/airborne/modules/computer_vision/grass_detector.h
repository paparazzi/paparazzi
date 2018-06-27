/*
 * Copyright (C) 2015
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/grass_detector.h
 */

#ifndef GRASS_DETECTOR_CV_H
#define GRASS_DETECTOR_CV_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
extern void grass_detector_init(void);

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern double settings_count_threshold;

extern int32_t grass_count;

enum grass_status {
    GRASS_UNSURE,               ///< There aren't enough green pixels to be sure
    GRASS_OUTSIDE,              ///< We are outside the grass
    GRASS_INSIDE                ///< We are inside the grass
};

typedef struct grass_detector_t {
    enum grass_status inside;   ///< Wether correction is neccesary
    double angle;               ///< XY body angle towards the grass
    double range;               ///< Normalized range to grass [0-1]
} grass_detector;

extern grass_detector cv_grass_detector;

extern struct video_listener *listener;

#endif /* GRASS_DETECTOR_CV_H */
