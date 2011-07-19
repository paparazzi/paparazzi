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

/* driver for the accelrometer ADXL345 from Analog Devices
 * this extra header allows standalone operation of the ADXL345 using I2C protocol
 */

#ifndef ITG3200_EXTRA_I2C_H
#define ITG3200_EXTRA_I2C_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include address and register definition */
#include "peripherals/adxl345.h"

/* Default conf */
#ifndef ADXL345_BW_RATE
#define ADXL345_BW_RATE 0x0a // Output rate 100Hz, bandwidth 50Hz
#endif
#ifndef ADXL345_POWER_CTL_MEASURE
#define ADXL345_POWER_CTL_MEASURE 0x1 // Continious measurement mode
#endif
#ifndef ADXL345_INT_ENABLE
#define ADXL345_INT_ENABLE 0x0 // Disable interrups
#endif
#ifndef ADXL345_DATA_FORMAT_FULL_RES
#define ADXL345_DATA_FORMAT_FULL_RES 0x1 // Full resolution mode
#endif
#ifndef ADXL345_DATA_FORMAT_JUSTIFY
#define ADXL345_DATA_FORMAT_JUSTIFY 0x0 // Right justified with sign extension
#endif
#ifndef ADXL345_DATA_FORMAT_RANGE
#define ADXL345_DATA_FORMAT_RANGE 0x3 // g Range: 16g
#endif

#define ADXL345_POWER_CTL (ADXL345_POWER_CTL_MEASURE<<3)
#define ADXL345_DATA_FORMAT ((ADXL345_DATA_FORMAT_FULL_RES<<3)|(ADXL345_DATA_FORMAT_JUSTIFY<<2)|(ADXL345_DATA_FORMAT_RANGE))

/* Default I2C address */
#ifndef ADXL345_I2C_ADDR
#define ADXL345_I2C_ADDR ADXL345_ADDR
#endif

/* Default I2C device */
#ifndef ADXL345_I2C_DEVICE
#define ADXL345_I2C_DEVICE i2c1
#endif

// Data ready flag
extern volatile bool_t adxl345_data_available;
// Data vector
extern struct Int16Vect3 adxl345_data;
// I2C transaction structure
extern struct i2c_transaction adxl345_trans;

// TODO IRQ handling

// Functions
extern void adxl345_init(void);
extern void adxl345_periodic(void);
extern void adxl345_event(void);

#define AccelEvent(_handler) {     \
    adxl345_event();              \
    if (adxl345_data_available) { \
      _handler();                 \
    }                             \
  }

#endif // ADXL345_EXTRA_I2C_H

