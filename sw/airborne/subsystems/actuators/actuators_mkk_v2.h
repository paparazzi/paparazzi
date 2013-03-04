/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file actuators_mkk_v2.h
 *  Actuators driver for Mikrokopter motor controllers.
 */

#ifndef ACTUATORS_MKK_V2_H
#define ACTUATORS_MKK_V2_H

#include "std.h"
#include "mcu_periph/i2c.h"

#include "generated/airframe.h"

struct actuators_mkk_v2_telemetry_struct {
  uint8_t Version;        // Motor controller version
  uint8_t Current;        // In 0.1 A steps, read back from BL
  uint8_t MaxPWM;         // Read back from BL -> is less than 255 if BL is in current limit, not running (250) or starting (40)
  int8_t  Temperature;    // Old BL-Ctrl will return a 255 here, the new version the temp. in °C
};

struct actuators_mkk_v2_struct {
  uint8_t read_number;
  uint16_t setpoint[ACTUATORS_MKK_V2_NB];
  struct i2c_transaction trans[ACTUATORS_MKK_V2_NB];
  struct actuators_mkk_v2_telemetry_struct data[ACTUATORS_MKK_V2_NB];
};

extern struct actuators_mkk_v2_struct actuators_mkk_v2;

extern void actuators_mkk_v2_init(void);
extern void actuators_mkk_v2_set(void);

#define ActuatorMkk_v2Set(_i, _v) { actuators_mkk_v2.setpoint[_i] = _v; }
#define ActuatorsMkk_v2Init() actuators_mkk_v2_init()
#define ActuatorsMkk_v2Commit() actuators_mkk_v2_set()

#endif /* ACTUATORS_MKK_V2_H */
