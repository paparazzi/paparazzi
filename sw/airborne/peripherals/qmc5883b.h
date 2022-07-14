/*
 * Copyright (C) 2020 Paparazzi Team
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
 * @file peripherals/qmc5883b.c
 *
 * Driver for QMC5883 magnetometer. 
 * This chip is often used as a replacement of the HMC5883L and not communicated in the device documentation.
 * 
 * If your "HMC5883L" magnetometer does somehow not work, you likely have a QMC5883, try this driver instead.
 * 
 * Only i2c support
 */

#ifndef QMC5883B_H
#define QMC5883B_H

#include "std.h"
#include "mcu_periph/i2c.h"

enum Qmc5883B_Status {
  QMC5883B_AQUIRE,
  QMC5883B_PARSE
};

struct Qmc5883B {
  struct i2c_transaction trans;
  uint8_t addr;
  enum Qmc5883B_Status status;
  struct Int16Vect3 mag_data_vect;
  int16_t mag_data[3];
  uint8_t mode; // mode setting bit for the chip
}

extern struct Qmc5883B qmc;

extern void qmc5883b_init(void);
extern void qmc5883b_event(void);
extern void qmc5883b_periodic(void);

#endif /* QMC5883B_H */
