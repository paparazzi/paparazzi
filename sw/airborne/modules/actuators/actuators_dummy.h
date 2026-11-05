/*
 * Copyright (C) 2012 Gautier Hattenberger
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

#ifndef ACTUATORS_DUMMY_H
#define ACTUATORS_DUMMY_H

#include "std.h"

/* No-op setter for the generated actuator table. */
static inline void actuators_dummy_set(uint8_t idx, int16_t value)
{
  (void)idx;
  (void)value;
}

#define ActuatorDummySet actuators_dummy_set
#define ActuatorsDummyInit() {}
#define ActuatorsDummyCommit() {}

#endif
