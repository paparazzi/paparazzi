/*
 * Copyright (C) 2010 Martin Mueller
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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

/** @file peripherals/bmp085.h
 *  Bosch BMP085 driver interface.
 */

#ifndef BMP085_H
#define BMP085_H

#include "mcu_periph/i2c.h"
#include "std.h"

enum Bmp085Status {
  BMP085_STATUS_UNINIT,
  BMP085_STATUS_IDLE,
  BMP085_STATUS_START_TEMP,
  BMP085_STATUS_READ_TEMP,
  BMP085_STATUS_START_PRESS,
  BMP085_STATUS_READ_PRESS
};

struct Bmp085Calib {
  // These values come from EEPROM on sensor
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t b1;
  int16_t b2;
  int16_t mb;
  int16_t mc;
  int16_t md;

  // These values are calculated
  int32_t b5;
};

typedef bool (*Bmp085EOC)(void);

struct Bmp085 {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  Bmp085EOC eoc;                      ///< function to check End Of Conversion
  enum Bmp085Status status;           ///< state machine status
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  struct Bmp085Calib calib;
  int32_t ut;                         ///< uncompensated temperature
  int32_t up;                         ///< uncompensated pressure
  int32_t temperature;                ///< temperature in 0.1 deg Celcius
  int32_t pressure;                   ///< pressure in Pascal
};

extern void bmp085_read_eeprom_calib(struct Bmp085 *bmp);
extern void bmp085_init(struct Bmp085 *bmp, struct i2c_periph *i2c_p, uint8_t addr);
extern void bmp085_periodic(struct Bmp085 *bmp);
extern void bmp085_event(struct Bmp085 *bmp);

#endif
