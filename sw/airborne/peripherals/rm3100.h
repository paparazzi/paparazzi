/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file peripherals/rm3100.h
 *
 * PNI RM3100 3-axis magnetometer driver interface (I2C).
 */

#ifndef RM3100_H
#define RM3100_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/**
 *  The lower 2 bits of the slave address are user-configurable, using pins 3 and 28
 */
#define RM3100_ADDR0  (0b0100000 << 1)
#define RM3100_ADDR1  (0b0100001 << 1)
#define RM3100_ADDR2  (0b0100010 << 1)
#define RM3100_ADDR3  (0b0100011 << 1)

// Continuous mode data rate in Hz
#define RM3100_RATE_600   0x92
#define RM3100_RATE_300   0x93
#define RM3100_RATE_150   0x94
#define RM3100_RATE_75    0x95
#define RM3100_RATE_37    0x96
#define RM3100_RATE_18    0x97
#define RM3100_RATE_9     0x98
#define RM3100_RATE_4_5   0x99
#define RM3100_RATE_2_3   0x9A
#define RM3100_RATE_1_2   0x9B
#define RM3100_RATE_0_6   0x9C
#define RM3100_RATE_0_3   0x9D
#define RM3100_RATE_0_15  0x9E
#define RM3100_RATE_0_075 0x9F

// Default data rate
#define RM3100_TMRC_DEFAULT RM3100_RATE_150

/** config status states */
enum Rm3100Status {
  RM3100_CONF_UNINIT,
  RM3100_CONF_CCR_DONE,
  RM3100_CONF_TMRC_DONE,
  RM3100_CONF_CCM_DONE,
  RM3100_STATUS_IDLE,
  RM3100_STATUS_MEAS
};

// main structure
struct Rm3100 {
  struct i2c_periph *i2c_p;           ///< peripheral used for communcation
  struct i2c_transaction i2c_trans;   ///< i2c transaction
  uint8_t data_rate;                  ///< sensor data rate
  bool initialized;                   ///< config done flag
  enum Rm3100Status status;           ///< init status
  volatile bool data_available;       ///< data ready flag
  union {
    struct Int32Vect3 vect;           ///< data vector in mag coordinate system
    int32_t value[3];                 ///< data values accessible by channel index
  } data;
};

// Functions
extern void rm3100_init(struct Rm3100 *mag, struct i2c_periph *i2c_p, uint8_t addr, uint8_t data_rate);
extern void rm3100_configure(struct Rm3100 *mag);
extern void rm3100_event(struct Rm3100 *mag);
extern void rm3100_read(struct Rm3100 *mag);

/// convenience function: read or start configuration if not already initialized
static inline void rm3100_periodic(struct Rm3100 *mag)
{
  if (mag->initialized) {
    rm3100_read(mag);
  } else {
    rm3100_configure(mag);
  }
}

#endif /* RM31000_H */
