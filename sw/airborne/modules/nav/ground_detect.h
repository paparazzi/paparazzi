/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/nav/ground_detect.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Ground detection module
 */

#ifndef GROUND_DETECT_H
#define GROUND_DETECT_H

#include "std.h"

union ground_detect_bitmask_t {
  uint16_t value;
  struct {
    bool vspeed_trigger : 1;      
    bool spec_thrust_trigger : 1;  
    bool accel_filt_trigger: 1;     
    bool agl_trigger : 1;  
    bool hx711_trigger : 1;          
  };
};

struct ground_detect_values_t {
  float vspeed_ned;
  float spec_thrust_down;
  float accel_filter;
  float agl_dist_value_filtered;
};

extern void ground_detect_init(void);
extern void ground_detect_periodic(void);

extern bool ground_detect(void);

extern void ground_detect_filter_accel(void);

extern bool disarm_on_not_in_flight;
extern bool ground_detect_reverse_thrust(void);
extern void ground_detect_stop_reverse_thrust(void);
extern void ground_detect_start_reverse_thrust(void);

extern uint16_t reverse_th_level; // Reverse thrust level in pprz units

#endif  // GROUND_DETECT_H
