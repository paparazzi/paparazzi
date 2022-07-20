/*
 * Copyright (C) 2021 Paparazzi Team
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

/**
 * @file modules/sensors/mag_qmc5883.h
 *
 * Module for QMC5883 magnetometer.
 */

#ifndef QMC5883_H
#define QMC5883_H

#include "std.h"
#include "mcu_periph/i2c.h"

enum Qmc5883_Status {
  QMC5883_INIT,
  QMC5883_SET_MODE,
  QMC5883_REQUEST,
  QMC5883_ACQUIRE,
  QMC5883_PARSE
};

struct Qmc5883_Status_Register {
  uint8_t DRDY;  // data ready
  uint8_t OVL;   // overflow
  uint8_t DOR;   // data skip
};

struct Qmc5883 {
  struct i2c_transaction trans;
  uint8_t waddr;
  uint8_t raddr;
  enum Qmc5883_Status status;
  uint8_t raw_mag_data[6];
  struct Qmc5883_Status_Register status_register;
  int16_t mag_data[3];
  uint8_t mode; // mode setting bit for the chip
  bool set_mode;
  bool initialized;
  uint8_t status_debug;
  uint8_t trans_status;
};

extern struct Qmc5883 qmc;

extern void mag_qmc5883_init(void);
extern void mag_qmc5883_event(void);
extern void mag_qmc5883_periodic(void);

#endif /* QMC5883_H */
