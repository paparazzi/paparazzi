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

/** @file actuators_sbus.h
 *  Sbus actuator driver, which can output as 7 sbus channels at ~11ms.
 *  Channels min, average and maximum should be: min 0 neutral 1023  max 2047
 */

#ifndef ACTUATORS_SBUS_H
#define ACTUATORS_SBUS_H

#include "std.h"

/* Maximum amount of sbus actuator channels */
#define ACTUATORS_SBUS_MAX_NB   16

/* Main actuator structure */
struct ActuatorsSbus {
  int32_t cmds[ACTUATORS_SBUS_MAX_NB];
  struct link_device *device;
};

/* Functions used in actuator macros */
extern struct ActuatorsSbus actuators_sbus;
extern void actuators_sbus_init(void);
extern void actuators_sbus_set(void);

/* Actuator macros */
#define ActuatorSbusSet(_i, _v) { actuators_sbus.cmds[_i] = _v; }
#define ActuatorsSbusInit() actuators_sbus_init()
#define ActuatorsSbusCommit() actuators_sbus_set()


#endif /* ACTUATORS_SBUS_H */
