/*
 *
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
 */

/* Driver for HMC5843 and HMC5883
 */

#ifndef HMC58XX_H
#define HMC58XX_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/* default I2C address */
#define HMC58XX_ADDR 0x3C

/* Registers */
#define HMC58XX_REG_CFGA   0x00
#define HMC58XX_REG_CFGB   0x01
#define HMC58XX_REG_MODE   0x02
#define HMC58XX_REG_DATXM  0x03
#define HMC58XX_REG_DATXL  0x04
#define HMC58XX_REG_DATYM  0x05
#define HMC58XX_REG_DATYL  0x06
#define HMC58XX_REG_DATZM  0x07
#define HMC58XX_REG_DATZL  0x08
#define HMC58XX_REG_STATUS 0x09
#define HMC58XX_REG_IDA    0x0A
#define HMC58XX_REG_IDB    0x0B
#define HMC58XX_REG_IDC    0x0C

/* HMC58XX default conf */
#ifndef HMC58XX_DO
#define HMC58XX_DO 0x6 // Data Output Rate (6 -> 50Hz with HMC5843, 75Hz with HMC5883)
#endif
#ifndef HMC58XX_MS
#define HMC58XX_MS 0x0 // Measurement configuration
#endif
#ifndef HMC58XX_GN
#define HMC58XX_GN 0x1 // Gain configuration (1 -> +- 1 Gauss)
#endif
#ifndef HMC58XX_MD
#define HMC58XX_MD 0x0 // Continious measurement mode
#endif

#define HMC58XX_CRA ((HMC58XX_DO<<2)|(HMC58XX_MS))
#define HMC58XX_CRB (HMC58XX_GN<<5)

/* Default I2C device is i2c2 (for lisa) */
#ifndef HMC58XX_I2C_DEVICE
#define HMC58XX_I2C_DEVICE i2c2
#endif

// Data ready flag
extern volatile bool_t hmc58xx_data_available;
// Data vector
extern struct Int16Vect3 hmc58xx_data;
// I2C transaction structure
extern struct i2c_transaction hmc58xx_i2c_trans;

// TODO IRQ handling

// Functions
extern void hmc58xx_init(void);
extern void hmc58xx_periodic(void);
extern void hmc58xx_event(void);

#define MagEvent(_m_handler) {    \
    hmc58xx_event();              \
    if (hmc58xx_data_available) { \
      _m_handler();               \
    }                             \
  }

#endif /* HMC58XX_H */
