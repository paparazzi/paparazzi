/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/lis3mdl.h
 *
 * ST LIS3MDL 3-axis magnetometer driver interface (I2C).
 */

#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

#define LIS3MDL_ADDR1  (0b0011110 << 1)
#define LIS3MDL_ADDR2  (0b0011100 << 1)

#define LIS3MDL_PERFORMANCE_LOW_POWER  0b00
#define LIS3MDL_PERFORMANCE_MEDIUM     0b01
#define LIS3MDL_PERFORMANCE_HIGH       0b10
#define LIS3MDL_PERFORMANCE_ULTRA_HIGH 0b11

#define LIS3MDL_DATA_RATE_0_625_HZ     0b000
#define LIS3MDL_DATA_RATE_1_25_HZ      0b001
#define LIS3MDL_DATA_RATE_2_5_HZ       0b010
#define LIS3MDL_DATA_RATE_5_HZ         0b011
#define LIS3MDL_DATA_RATE_10_HZ        0b100
#define LIS3MDL_DATA_RATE_20_HZ        0b101
#define LIS3MDL_DATA_RATE_40_HZ        0b110
#define LIS3MDL_DATA_RATE_80_HZ        0b111

#define LIS3MDL_MODE_CONTINUOUS        0b00
#define LIS3MDL_MODE_SINGLE            0b01
#define LIS3MDL_MODE_POWER_DOWN        0b11

#define LIS3MDL_SCALE_4_GAUSS          0b00
#define LIS3MDL_SCALE_8_GAUSS          0b01
#define LIS3MDL_SCALE_12_GAUSS         0b10
#define LIS3MDL_SCALE_16_GAUSS         0b11

/** config status states */
enum Lis3mdlStatus {
  LIS3MDL_CONF_UNINIT,
  LIS3MDL_CONF_REG,
  LIS3MDL_STATUS_IDLE,
  LIS3MDL_STATUS_MEAS
};

// main structure
struct Lis3mdl {
  struct i2c_periph *i2c_p;           ///< peripheral used for communcation
  struct i2c_transaction i2c_trans;   ///< i2c transaction
  bool initialized;                   ///< config done flag
  enum Lis3mdlStatus status;          ///< init status
  volatile bool data_available;       ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in mag coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
};

// Functions
extern void lis3mdl_init(struct Lis3mdl *mag, struct i2c_periph *i2c_p, uint8_t addr, uint8_t data_rate, uint8_t scale, uint8_t mode, uint8_t perf);
extern void lis3mdl_configure(struct Lis3mdl *mag);
extern void lis3mdl_event(struct Lis3mdl *mag);
extern void lis3mdl_read(struct Lis3mdl *mag);

/// convenience function: read or start configuration if not already initialized
static inline void lis3mdl_periodic(struct Lis3mdl *mag)
{
  if (mag->initialized) {
    lis3mdl_read(mag);
  } else {
    lis3mdl_configure(mag);
  }
}

#endif /* LIS3MDL_H */
