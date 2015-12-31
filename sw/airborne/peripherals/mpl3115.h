/*
 * Copyright (C) 2011 Gautier Hattenberger
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

/** @file peripherals/mpl3115.h
 * Driver for the baro MPL3115A2 from Freescale (i2c)
 */

#ifndef MPL3115_H
#define MPL3115_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Device address (8 bits) */
#define MPL3115_I2C_ADDR 0xC0

/* Registers */
#define MPL3115_REG_STATUS 0x00
#define MPL3115_REG_OUT_P_MSB 0x01
#define MPL3115_REG_OUT_P_CSB 0x02
#define MPL3115_REG_OUT_P_LSB 0x03
#define MPL3115_REG_OUT_T_MSB 0x04
#define MPL3115_REG_OUT_T_LSB 0x05
#define MPL3115_REG_WHO_AM_I 0x0C
#define MPL3115_REG_PT_DATA_CFG 0x13
#define MPL3115_REG_CTRL_REG1 0x26
#define MPL3115_REG_CTRL_REG2 0x27
#define MPL3115_REG_CTRL_REG3 0x28
#define MPL3115_REG_CTRL_REG4 0x29
#define MPL3115_REG_CTRL_REG5 0x2A

#define MPL3115_OST_BIT (1<<1) // One Shot control bit

/* Default conf */
#ifndef MPL3115_PT_DATA_CFG
#define MPL3115_PT_DATA_CFG 0x2 // Enable data event flag for pressure
#endif
#ifndef MPL3115_OVERSAMPLING
#define MPL3115_OVERSAMPLING 0x5 // Oversample ratio (0x5: 130ms between data sample)
#endif


enum Mpl3115Status {
  MPL_CONF_UNINIT,
  MPL_CONF_PT_DATA,
  MPL_CONF_CTRL1,
  MPL_CONF_DONE
};

struct Mpl3115 {
  struct i2c_periph *i2c_p;
  struct i2c_transaction trans;       ///< I2C transaction for reading and configuring
  struct i2c_transaction req_trans;   ///< I2C transaction for conversion request
  enum Mpl3115Status init_status;
  bool_t initialized;                 ///< config done flag
  volatile bool_t data_available;     ///< data ready flag
  bool_t raw_mode;                    ///< set to TRUE to enable raw output
  bool_t alt_mode;                    ///< set to TRUE to enable altitude output (otherwise pressure)
  int16_t temperature;                ///< temperature in 1/16 degrees Celcius
  uint32_t pressure;                  ///< pressure in 1/4 Pascal
  float altitude;                     ///< altitude in meters
};

// Functions
extern void mpl3115_init(struct Mpl3115 *mpl, struct i2c_periph *i2c_p, uint8_t addr);
extern void mpl3115_configure(struct Mpl3115 *mpl);
extern void mpl3115_read(struct Mpl3115 *mpl);
extern void mpl3115_event(struct Mpl3115 *mpl);
extern void mpl3115_periodic(struct Mpl3115 *mpl);

#endif // MPL3115_H
