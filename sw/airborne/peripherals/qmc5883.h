/*
 * Copyright (C) 2020 Paparazzi Team
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
 * @file peripherals/qmc5883.c
 *
 * Driver for QMC5883 magnetometer. 
 * This chip is often used as a replacement of the HMC5883L and not communicated in the device documentation.
 * 
 * If your "HMC5883L" magnetometer does somehow not work, you likely have a QMC5883, try this driver instead.
 * 
 * Only i2c support
 */

#ifndef QMC5883_H
#define QMC5883_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/* Address and register definitions */
#include "peripherals/qmc5883_regs.h"

extern struct Qmc5883Debug debug;

struct Qmc5883Debug {
  int32_t initialized;
};

struct Qmc5883Config {
  uint8_t allconfigbits;
  /*
  uint8_t osr;  ///< Over Sample Rate
  uint8_t rng;  ///< Gauss Full Scale Range
  uint8_t odr;  ///< Output data rate
  uint8_t mode; ///< Measurement mode
  */
};

/** config status states */
enum Qmc5883ConfStatus {
  QMC_CONF_UNINIT,
  QMC_CONF_1,
  QMC_CONF_DONE
};

enum Qmc5883Type {
  QMC_TYPE_DA5883,
  QMC_TYPE_DB5883
};

struct Qmc5883 {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  bool initialized;                 ///< config done flag
  enum Qmc5883ConfStatus init_status; ///< init status
  volatile bool data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in mag coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Qmc5883Config config;
  enum Qmc5883Type type;
  uint16_t adc_overflow_cnt;          ///< counts number of ADC measurement under/overflows
  int32_t dummy;
};

// Functions
extern void qmc5883_init(struct Qmc5883 *qmc, struct i2c_periph *i2c_p, uint8_t addr);
extern void qmc5883_read(struct Qmc5883 *qmc, uint8_t addr);
extern void qmc5883_periodic(struct Qmc5883 *qmc);
// extern void qmc5883_start_configure(struct Qmc5883 *qmc);

#endif /* QMC5883_H */
