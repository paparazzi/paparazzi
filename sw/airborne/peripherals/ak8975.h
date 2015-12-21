/*
 * Copyright (C) 201 Xavier Paris
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
 * @file peripherals/ak8975.c
 *
 */

#ifndef AK8975_H
#define AK8975_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

#define AK8975_I2C_SLV_ADDR       0x0C
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

struct Ak8975 {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Ak8975ConfStatus init_status; 
  bool_t initialized;
};


// Functions
extern void ak8975_init(struct Ak8975 *ak, struct i2c_periph *i2c_p, uint8_t addr);
extern bool_t ak8975_mpu_configure(struct Ak8975 *ak);
extern float  i2cMasterGetAK8975_ajustedValue(const int16_t rawValue, const uint8_t axis);

#endif /* AK8975_H */
