/*
 * Copyright (C) 2010 ENAC
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

/** @file modules/sensors/pressure_board_navarro.h
 *
 * Pressure Board Navarro (2010)
 *
 * by Matthieu Navarro
 *
 * combine differential and absolute pressure sensor (ETS raw sensors)
 * controlled by a dspic
 * return scaled values over I2C
 *
 */

#ifndef PRESSURE_BOARD_NAVARRO_H
#define PRESSURE_BOARD_NAVARRO_H

#include "std.h"
#include "mcu_periph/i2c.h"

struct PBNState {
  uint16_t altitude_adc;
  uint16_t airspeed_adc;
  uint16_t altitude_offset;
  uint16_t airspeed_offset;
  float altitude;
  float airspeed;
  float airspeed_filter;
  bool_t data_valid;
};

extern struct PBNState pbn;

extern struct i2c_transaction pbn_trans;

extern void pbn_init(void);
extern void pbn_periodic(void);
extern void pbn_read_event(void);

#define PbnEvent() { if (pbn_trans.status == I2CTransSuccess) pbn_read_event(); }

#endif // PRESSURE_BOARD_NAVARRO_H
