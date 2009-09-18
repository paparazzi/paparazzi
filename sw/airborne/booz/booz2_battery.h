/*
 * $Id$
 *
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

#ifndef BOOZ2_BATTERY_H
#define BOOZ2_BATTERY_H

#include "std.h"

#include "airframe.h"

/* decivolts */
extern uint8_t booz2_battery_voltage;

static inline void Booz2BatteryISRHandler(uint16_t _val) {
  uint32_t cal_v = (uint32_t)(_val) * BATTERY_SENS_NUM / BATTERY_SENS_DEN;
  uint32_t sum = (uint32_t)booz2_battery_voltage + cal_v;
  booz2_battery_voltage = (uint8_t)(sum/2);
}


extern void booz2_battery_init(void);

#endif /* BOOZ2_BATTERY_H */
