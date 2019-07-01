/*
 * Copyright (C) Freek van Tienen
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
 * @file "modules/computer_vision/bebop_ae_awb.h"
 * @author Freek van Tienen, Kirk Scheper
 * Auto exposure and Auto white balancing for the front camera on the Parrot Bebop 1 and 2 and the Disco
 */

#ifndef BEBOP_AE_AWB_H
#define BEBOP_AE_AWB_H

#include "std.h"

struct ae_setting_t {
  bool    active;
  bool    prev_active;
  float   exposure_gain;
  float   bright_ignore;
  float   dark_ignore;
  uint8_t middle_index;
  uint8_t dark_bins;
  uint8_t bright_bins;
};
extern struct ae_setting_t ae_set;

struct awb_setting_t {
  bool  active;
  bool  prev_active;
  float gain;
  bool  gain_scheduling;
  float gain_scheduling_target;
  float gain_scheduling_tolerance;
  float gain_scheduling_step;
};
extern struct awb_setting_t awb_set;

extern void bebop_ae_awb_init(void);

#endif

