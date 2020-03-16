/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/joystick/joystick.h"
 * @author Gautier Hattenberger
 * Handle JOYSTICK_RAW messages
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "std.h"

/**
 * Joystick structure
 */
struct Joystick {
  int32_t roll;     ///< roll command
  int32_t pitch;    ///< pitch command
  int32_t yaw;      ///< yaw command
  int32_t throttle; ///< throttle command
};

extern struct Joystick joystick;

/**
 * Init function
 */
extern void joystick_init(void);

/**
 * JOYSTICK_RAW message parser
 * if valid, send a JOYSTICK ABI message
 */
extern void joystick_parse(uint8_t *buf);

#endif

