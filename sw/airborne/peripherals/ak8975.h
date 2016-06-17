/*
 * Copyright (C) 2015 Xavier Paris, Gautier Hattenberger
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
 * @file peripherals/ak8975.h
 *
 * Driver for the AKM AK8975 magnetometer.
 */

#ifndef AK8975_H
#define AK8975_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/* Address and register definitions */
#define AK8975_I2C_SLV_ADDR       (0x0C<<1)
#define AK8975_REG_ST1_ADDR       0x02
#define AK8975_REG_CNTL_ADDR      0x0A
#define AK8975_REG_ASASX          0x10
#define AK8975_MODE_FUSE_ACCESS   0b00001111
#define AK8975_MODE_POWER_DOWN    0b00000000
#define AK8975_MODE_SINGLE_MEAS   0b00000001

/** config status states */
enum Ak8975ConfStatus {
  AK_CONF_UNINIT,
  AK_REQ_CALIBRATION,
  AK_DISABLE_ACCESS_CALIBRATION,
  AK_CONF_REQUESTED
};

/** Normal status states */
enum Ak8975Status {
  AK_STATUS_IDLE,
  AK_STATUS_MEAS,
  AK_STATUS_READ,
  AK_STATUS_DONE
};

struct Ak8975 {
  struct i2c_periph *i2c_p;           ///< peripheral used for communcation
  struct i2c_transaction i2c_trans;   ///< i2c transaction used for communication with the ak8936
  bool initialized;                 ///< config done flag

  enum Ak8975Status status;           ///< main status
  enum Ak8975ConfStatus init_status;  ///< init status
  uint32_t last_meas_time;            ///< last measurement time in ms

  volatile bool data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in mag coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
};


// Functions
extern void ak8975_init(struct Ak8975 *ak, struct i2c_periph *i2c_p, uint8_t addr);
extern void ak8975_configure(struct Ak8975 *ak);
extern void ak8975_event(struct Ak8975 *ak);
extern void ak8975_read(struct Ak8975 *ak);

/// convenience function: read or start configuration if not already initialized
static inline void ak8975_periodic(struct Ak8975 *ak)
{
  if (ak->initialized) {
    ak8975_read(ak);
  } else {
    ak8975_configure(ak);
  }
}

#endif /* AK8975_H */
