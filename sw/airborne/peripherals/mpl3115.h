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

/* Default I2C device */
#ifndef MPL3115_I2C_DEV
#define MPL3115_I2C_DEV i2c0
#endif

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
#ifndef MPL3115_RAW_OUTPUT
#define MPL3115_RAW_OUTPUT 0x0 // Raw output (disable alt mode if true)
#endif
#ifndef MPL3115_ALT_MODE
#define MPL3115_ALT_MODE 0x1 // 0: baro, 1: alti (disable by raw mode)
#endif

#define MPL3115_CTRL_REG1 ((MPL3115_OVERSAMPLING<<3)|(MPL3115_RAW_OUTPUT<<6)|(MPL3115_ALT_MODE<<7))

// Config done flag
extern bool_t mpl3115_initialized;
// Data ready flag
extern volatile bool_t mpl3115_data_available;
// Data
extern uint32_t mpl3115_pressure;
extern int16_t mpl3115_temperature;
extern float mpl3115_alt;
// I2C transaction structure
//extern struct i2c_transaction mpl3115_trans;

// Functions
extern void mpl3115_init(void);
extern void mpl3115_configure(void);
extern void mpl3115_read(void);
extern void mpl3115_event(void);

// Macro for using MPL3115 in periodic function
#define Mpl3115Periodic() {                 \
  if (mpl3115_initialized) mpl3115_read();  \
  else mpl3115_configure();                 \
}

#endif // MPL3115_H
