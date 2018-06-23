/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/actuators/actuators_md25.h"
 * @author Gautier Hattenberger
 * Driver for the MD25 rover controller board
 */

#ifndef ACTUATORS_MD25_H
#define ACTUATORS_MD25_H

#include "std.h"
#include "mcu_periph/i2c.h"

/* Main actuator structure */
struct ActuatorsMD25 {
  uint8_t cmds[2];      ///< commands
  int32_t encoders[2];  ///< encoder values
  uint8_t bat;          ///< batterie voltage (in decivolt)
  uint8_t current[2];   ///< current in motors (in deciamp)
  uint8_t mode;         ///< control mode
  uint8_t accel_rate;   ///< accel rate (from 1 to 10)
  bool initialized;     ///< init flag
  struct i2c_transaction trans_cmd;     ///< i2c struct for command
  struct i2c_transaction trans_sensors; ///< i2c struct for sensors
};

extern struct ActuatorsMD25 actuators_md25;

extern void actuators_md25_init(void);
extern void actuators_md25_periodic(void);
extern void actuators_md25_event(void);
extern void actuators_md25_set(void);

/* Actuator macros */
#define ActuatorMD25Set(_i, _v) { actuators_md25.cmds[_i] = _v; }
#define ActuatorsMD25Init() actuators_md25_init()
#define ActuatorsMD25Commit() actuators_md25_set()

#endif

