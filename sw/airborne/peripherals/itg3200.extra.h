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

/* driver for the gyro ITG3200 from InvenSense
 * this extra header allows standalone operation of the ITG3200
 */

#ifndef ITG3200_EXTRA_H
#define ITG3200_EXTRA_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include address and register definition */
#include "peripherals/itg3200.h"

/* Default conf */
#ifndef ITG3200_SMPLRT_DIV
#define ITG3200_SMPLRT_DIV 0x00 // Sample rate divider
#endif
#ifndef ITG3200_FS_SEL
#define ITG3200_FS_SEL 0x3 // Full scale range +- 2000°/s
#endif
#ifndef ITG3200_DLPF_CFG
#define ITG3200_DLPF_CFG 0x3 // Internal sampling (1kHz, 42Hz LP Bandwidth)
#endif
#ifndef ITG3200_INT_CFG
#define ITG3200_INT_CFG 0x00 // No interrupt
#endif
#ifndef ITG3200_CLK_SEL
#define ITG3200_CLK_SEL 0x1 // PLL with X gyro reference
#endif

#define ITG3200_DLPF_FS ((ITG3200_FS_SEL<<3)|(ITG3200_DLPF_CFG))
#define ITG3200_PWR_MGM (ITG3200_CLK_SEL)

/* Default I2C address */
#ifndef ITG3200_I2C_ADDR
#define ITG3200_I2C_ADDR ITG3200_ADDR
#endif

/* Default I2C device */
#ifndef ITG3200_I2C_DEVICE
#define ITG3200_I2C_DEVICE i2c1
#endif

// Data ready flag
extern volatile bool_t itg3200_data_available;
// Data vector
extern struct Int32Rates itg3200_data;
// I2C transaction structure
extern struct i2c_transaction itg3200_trans;

// TODO IRQ handling

// Functions
extern void itg3200_init(void);
extern void itg3200_periodic(void);
extern void itg3200_event(void);

#define GyroEvent(_handler) {     \
    itg3200_event();              \
    if (itg3200_data_available) { \
      _handler();                 \
    }                             \
  }

#endif // ITG3200_EXTRA_H
