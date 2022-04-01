/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
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
 * @file modules/computer_vision/cv_detect_color_object.h
 * Assumes the color_object consists of a continuous color and checks
 * if you are over the defined color_object or not
 */

#ifndef COLOR_OBJECT_DETECTOR_CV_H
#define COLOR_OBJECT_DETECTOR_CV_H

#include <stdint.h>
#include <stdbool.h>

// Module settings
extern bool GRAY_SCALE1;
extern bool BLUR_IMAGE1;
extern uint8_t  BLUR_SIZE_IMAGE1;
extern bool BLUR_EDGES1;
extern uint8_t  BLUR_SIZE_EDGES1;
extern bool BORDERS1;
extern uint8_t  BORDER_MARGIN1;
extern bool Y_UP_filter1;
extern int  y_up_del1;
extern bool Y_DOWN_filter1;
extern int  y_down_del1;
extern uint8_t  thresholdmin1;
extern uint8_t  thresholdmax1;
extern uint8_t  kernal_size1;
extern double min_obs_size1;
extern double max_obs_size1;

extern bool GRAY_SCALE2;
extern bool BLUR_IMAGE2;
extern uint8_t  BLUR_SIZE_IMAGE2;
extern bool BLUR_EDGES2;
extern uint8_t  BLUR_SIZE_EDGES2;
extern bool BORDERS2;
extern uint8_t  BORDER_MARGIN2;
extern bool Y_UP_filter2;
extern int  y_up_del2;
extern bool Y_DOWN_filter2;
extern int  y_down_del2;
extern uint8_t  thresholdmin2;
extern uint8_t  thresholdmax2;
extern uint8_t  kernal_size2;
extern double min_obs_size2;
extern double max_obs_size2;

extern bool cod_draw1;
extern bool cod_draw2;

extern const uint8_t max_number_obsticals;

// Module functions
extern void color_object_detector_init(void);
extern void color_object_detector_periodic(void);

#endif /* COLOR_OBJECT_DETECTOR_CV_H */
