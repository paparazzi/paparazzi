/*
 * Copyright (C) 2022 Paparazzi Team
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
 * @file peripherals/qmc5883l.h
 *
 * QST QMC5883L 3-axis magnetometer driver interface (I2C).
 */

#ifndef QMC5883L_H
#define QMC5883L_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/**
 * The default I2C 7bit address is 0D: 0001101. no option to change the address by e.g. bridging pins of the chip
 * From the official datasheet "If other I2C address options are required, one must contact the chip factory"
 * Just in cse one comes across an custom chip one can extend the ADDR option here
 */
#define QMC5883L_ADDR0  (0b00001101 << 1) //from 0x0D to 0x1A since 7bits and 1 for R/W setting

/* Continuous mode Output Data Rate (ODR) options in Hz */
#define QMC5883L_ODR_10  0x00
#define QMC5883L_ODR_50  0x04
#define QMC5883L_ODR_100 0x08
#define QMC5883L_ODR_200 0x0C  

/* Default data rate */
#define QMC5883L_ODR_DEFAULT QMC5883L_ODR_200 //We are never interested in powersavings 200 is best

/* Status states */
 enum Qmc5883lStatus {
  QMC5883L_CONF_UNINIT,    // Perform Initialization
  QMC5883L_CONF_CCR_DONE,  // Is done set CCR = Configure Continuous Rate done
  QMC5883L_CONF_TMRC_DONE, // Is done set Datarate
  QMC5883L_CONF_CCM_DONE,  // Is done set Mode continuous or single CCM = Configure Continuous Measurement
  QMC5883L_STATUS_IDLE,    // Ready for data requests
  QMC5883L_STATUS_MEAS     // Get data
};

struct Qmc5883l {
  struct i2c_periph *i2c_p;           ///< peripheral used for communcation
  struct i2c_transaction i2c_trans;   ///< i2c transaction
  uint8_t data_rate;                  ///< sensor data rate
  bool initialized;                   ///< config done flag
  enum Qmc5883lStatus status;          ///< init status
  volatile bool data_available;       ///< data ready flag
  union {
    struct Int32Vect3 vect;           ///< data vector in mag coordinate system
    int32_t value[3];                 ///< data values accessible by channel index
  } data;
};

extern void qmc5883l_init(struct Qmc5883l *mag, struct i2c_periph *i2c_p, uint8_t addr, uint8_t data_rate);
extern void qmc5883l_configure(struct Qmc5883l *mag);
extern void qmc5883l_event(struct Qmc5883l *mag);
extern void qmc5883l_read(struct Qmc5883l *mag);

/// convenience function: read or start configuration if not already initialized
static inline void qmc5883l_periodic(struct Qmc5883l *mag)
{
  if (mag->initialized) {
    qmc5883l_read(mag);
  } else {
    qmc5883l_configure(mag);
  }
}

#endif /* QMC5883L_H */




