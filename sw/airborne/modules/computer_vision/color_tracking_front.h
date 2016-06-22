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

#ifndef COLOR_FRONT_CV_PLUGIN_H
#define COLOR_FRONT_CV_PLUGIN_H

#include <stdint.h>

// Module functions
void color_tracking_front_init(void);

// Colorfilter
extern uint8_t color_lum_min_front;
extern uint8_t color_lum_max_front;
extern uint8_t color_cb_min_front;
extern uint8_t color_cb_max_front;
extern uint8_t color_cr_min_front;
extern uint8_t color_cr_max_front;
extern int blob_threshold_front;

// Image-modification triggers
extern uint8_t modify_image_front;

#endif