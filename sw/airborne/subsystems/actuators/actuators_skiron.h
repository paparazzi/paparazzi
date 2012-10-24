/*
 * Copyright (C) 2011 Gautier Hattenberger
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

/** @file actuators_skiron.h
 *  Skiron motor speed controller by Michel.
 */

#ifndef ACTUATORS_SKIRON_H
#define ACTUATORS_SKIRON_H

#include "std.h"
#include "mcu_periph/i2c.h"

#include "generated/airframe.h"

// Use I2C broadcast adderss
#define ACTUATORS_SKIRON_I2C_ADDR 0x00

struct ActuatorsSkiron {
  struct i2c_transaction trans;
};

extern struct ActuatorsSkiron actuators_skiron;

extern void actuators_skiron_init(void);
extern void actuators_skiron_set(void);

#define ActuatorSkironSet(_i, _v) { actuators_skiron.trans.buf[_i] = _v; }
#define ActuatorsSkironInit() actuators_skiron_init()
#define ActuatorsSkironCommit() actuators_skiron_set()

#endif /* ACTUATORS_SKIRON_H */
