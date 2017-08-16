/*
 * Copyright (C) K. N. McGuire
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
 * @file "modules/range_module/range_module.h"
 * @author K. N. McGuire
 * This module contains functions to accomendate the use of single point range sensors. 
 */

#ifndef RANGE_MODULE_H
#define RANGE_MODULE_H
#include <std.h>

struct range_finders_ {
  int16_t front;  // mm
  int16_t right;  // mm
  int16_t left;   // mm
  int16_t back;   // mm
  int16_t bottom;   // mm
  int16_t top;   // mm

};


extern float inner_border_FF;
extern float outer_border_FF;
extern float min_vel_command;
extern float max_vel_command;


extern void range_init(void);
extern void range_run(void);


void range_sensor_force_field(float *vel_body_x, float *vel_body_y, float *vel_body_z, int16_t avoid_inner_border, int16_t avoid_outer_border,
    int16_t tinder_range, float min_vel_command_lc, float max_vel_command_lc);
void stereo_force_field(float *vel_body_x, float distance_stereo, float avoid_inner_border, float avoid_outer_border,
                        float tinder_range, float min_vel_command_lc, float max_vel_command_lc);


#endif

