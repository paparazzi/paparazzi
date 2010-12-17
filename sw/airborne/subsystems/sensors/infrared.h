/*
 * $Id$
 *
 * Copyright (C) 2003-2010 The Paparazzi Team
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
 *
 */

#ifndef SUBSYSTEMS_SENSORS_INFRARED_H
#define SUBSYSTEMS_SENSORS_INFRARED_H

#include "std.h"
#include "generated/airframe.h"

struct Infrared {
  /* the 3 channels of the sensor
   */
  int16_t ir1;
  int16_t ir2;
  int16_t ir3;
  /* neutrals in radians
   */
  float roll_neutral;
  float pitch_neutral;
  float pitch_vneutral;
  /* roll, pitch, yaw unscaled reading
   */
  int16_t roll;
  int16_t pitch;
  int16_t top;
  /* coefficients used to compensate
     for sensors sensitivity
  */
  float lateral_correction;
  float longitudinal_correction;
  float vertical_correction;
  /* coefficients used to compensate
     for masking
  */
  float correction_left;
  float correction_right;
  float correction_up;
  float correction_down;
};

extern struct Infrared infrared;

void infrared_init(void);
void infrared_update(void);

#endif /* SUBSYSTEMS_SENSORS_INFRARED_H */
