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

/** @file actuators_mkk.h
 *  Actuators driver for Mikrokopter motor controllers.
 */

#ifndef ACTUATORS_MKK_H
#define ACTUATORS_MKK_H

#include "std.h"
#include "mcu_periph/i2c.h"

#include "generated/airframe.h"


struct ActuatorsMkk {
  struct i2c_transaction trans[ACTUATORS_MKK_NB];
  uint16_t submit_err_cnt;
};

extern struct ActuatorsMkk actuators_mkk;

extern void actuators_mkk_init(void);
extern void actuators_mkk_set(void);

#define ActuatorMkkSet(_i, _v) { actuators_mkk.trans[_i].buf[0] = _v; }
#define ActuatorsMkkInit() actuators_mkk_init()
#define ActuatorsMkkCommit() actuators_mkk_set()

#endif /* ACTUATORS_MKK_H */
