/*
 * Copyright (C) 2025 Alejandro Rochas <alrochas@ucm.es>
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

/** @file modules/lidar/slam/servo_lidar.h
 *  @brief driver for the servo to move the lidar
 *
 */


#ifndef SERVO_LIDAR_H
#define SERVO_LIDAR_H

#include "std.h"

#define PWM2ANGLE(pwm) (((pwm) + MAX_PPRZ) * 90 / MAX_PPRZ) - 90 

struct ServoLidar {
  bool enabled;
  uint8_t speed;
  int32_t position;
  float angle;
  uint8_t direction;
  uint32_t last_update;
};

extern struct ServoLidar servoLidar;

extern void servoLidar_init(void);
extern void servoLidar_periodic(void);

#endif