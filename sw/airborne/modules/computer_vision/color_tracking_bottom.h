/*
 * Copyright (C) 2016 - IMAV 2016
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
 * @file modules/computer_vision/marker_tracking.h
 * @author IMAV 2016
 */

#ifndef COLOR_BOTTOM_CV_PLUGIN_H
#define COLOR_BOTTOM_CV_PLUGIN_H

#include <stdint.h>

// Module functions
void color_tracking_bottom_init(void);
uint8_t init_variables(void);

// Color
extern int color;

// Colorfilter
extern uint8_t color_lum_min_red;
extern uint8_t color_lum_max_red;
extern uint8_t color_cb_min_red;
extern uint8_t color_cb_max_red;
extern uint8_t color_cr_min_red;
extern uint8_t color_cr_max_red;

extern uint8_t color_lum_min_blue;
extern uint8_t color_lum_max_blue;
extern uint8_t color_cb_min_blue;
extern uint8_t color_cb_max_blue;
extern uint8_t color_cr_min_blue;
extern uint8_t color_cr_max_blue;

// Reliable color detection
extern int blob_threshold_bottom;

// Image-modification triggers
extern uint8_t modify_image_bottom;

// Marker detected in the bottom camera
extern uint8_t BOTTOM_MARKER;

// Navigation
extern float vel_gain_color;
extern float vz_desired;
extern float height_above_target;
extern float target_reached;

// Additional functions
extern uint8_t color_tracking_bottom_periodic(void); ///< A dummy for now
extern uint8_t start_color_tracking_bottom(void);
extern uint8_t stop_color_tracking_bottom(void);

#endif