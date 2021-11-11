/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file actuators_spektrum.h
 *  Spektrum actuator driver, which can output as 7 spektrum channels at ~11ms.
 *  Channels min, averga and maximum should be: 340, 1024, 1708
 */

#ifndef ACTUATORS_SPEKTRUM_H
#define ACTUATORS_SPEKTRUM_H

#include "std.h"

/* Maximum amount of spektrum actuator channels */
#define ACTUATORS_SPEKTRUM_MAX_NB   7

/* Main actuator structure */
struct ActuatorsSpektrum {
  int32_t cmds[ACTUATORS_SPEKTRUM_MAX_NB];
  struct link_device *device;
  struct link_device *device2;
};

/* Functions used in actuator macros */
extern struct ActuatorsSpektrum actuators_spektrum;
extern void actuators_spektrum_init(void);
extern void actuators_spektrum_set(void);

/* Actuator macros */
#define ActuatorSpektrumSet(_i, _v) { actuators_spektrum.cmds[_i] = _v; }
#define ActuatorsSpektrumInit() actuators_spektrum_init()
#define ActuatorsSpektrumCommit() actuators_spektrum_set()


#endif /* ACTUATORS_SPEKTRUM_H */
