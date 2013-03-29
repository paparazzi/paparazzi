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

/* driver for the accelrometer LSM303DLx from Analog Devices
 * this extra header allows standalone operation of the LSM303DLx using I2C protocol
 */

#ifndef LSM303DL_EXTRA_I2C_H
#define LSM303DL_EXTRA_I2C_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include address and register definition */
#include "peripherals/lsm303dl.h"

/* Default conf */
#ifdef LSM303_ACC_RATE 
#define LSM303DL_CTRL_REG1_A LSM303_ACC_RATE
#endif

#ifndef LSM303DL_CTRL_REG1_A
#define LSM303DL_CTRL_REG1_A LSM303_RATE_100 // 100hz normal
#endif
#ifndef LSM303DL_CTRL_REG4_A
#define LSM303DL_CTRL_REG4_A 0x30 // ???
#endif
#ifndef LSM303DL_CTRL_REG2_A
#define LSM303DL_CTRL_REG2_A 0x00 // ???
#endif

/* Default I2C address */
#ifndef LSM303DL_I2C_ADDR
#define LSM303DL_I2C_ADDR LSM303DL_ADDR
#endif

/* Default I2C device */
#ifndef LSM303DL_I2C_DEVICE
#define LSM303DL_I2C_DEVICE i2c1
#endif

// Config done flag
extern bool_t lsm303dl_initialized;
// Data ready flag
extern volatile bool_t lsm303dl_data_available;
// Data vector
extern struct Int16Vect3 lsm303dl_data;
// I2C transaction structure
extern struct i2c_transaction lsm303dl_trans;

// TODO IRQ handling

// Functions
extern void lsm303dl_init(void);
extern void lsm303dl_configure(void);
extern void lsm303dl_read(void);
extern void lsm303dl_event(void);

// Macro for using LSM303 in periodic function
#define Lsm303dlPeriodic() {                 \
  if (lsm303dl_initialized) lsm303dl_read();  \
  else lsm303dl_configure();                 \
}

#define AccelEvent(_handler) {     \
    lsm303dl_event();              \
    if (lsm303dl_data_available) { \
      _handler();                 \
    }                             \
  }

#endif // LSM303DL_EXTRA_I2C_H

