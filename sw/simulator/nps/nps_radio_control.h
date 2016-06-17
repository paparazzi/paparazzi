/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef NPS_RADIO_CONTROL_H
#define NPS_RADIO_CONTROL_H

#include "std.h"

#define MODE_SWITCH_MANUAL -1.0
#define MODE_SWITCH_AUTO1   0.0
#define MODE_SWITCH_AUTO2   1.0

enum NpsRadioControlType {
  SCRIPT,
  JOYSTICK,
  SPEKTRUM
};

extern void nps_radio_control_init(enum NpsRadioControlType type, int num_script, char *js_dev);

extern bool nps_radio_control_available(double time);

struct NpsRadioControl {
  double next_update;
  bool valid;
  double throttle;
  double roll;
  double pitch;
  double yaw;
  double mode;
  enum NpsRadioControlType type;
  int num_script;
  char *js_dev;
};

extern struct NpsRadioControl nps_radio_control;


#endif /* NPS_RADIO_CONTROL_H */
