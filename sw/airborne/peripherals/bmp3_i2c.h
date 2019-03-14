/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file peripherals/bmp3_i2c.h
 * @brief Sensor driver for BMP3 sensor via I2C
 *
 * Modified for Paparazzi from SDP3 driver from BoshSensortec
 * see https://github.com/BoschSensortec/BMP3-Sensor-API
 * for original code and license
 *
 */

#ifndef BMP3_I2C_H
#define BMP3_I2C_H

/* Header includes */
#include "peripherals/bmp3_regs.h"
#include "mcu_periph/i2c.h"

struct Bmp3_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Bmp3Status status;           ///< state machine status
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  struct bmp3_reg_calib_data calib; ///< calibration data
#if (BMP3_COMPENSATION == BMP3_DOUBLE_PRECISION_COMPENSATION) || ( BMP3_COMPENSATION == BMP3_SINGLE_PRECISION_COMPENSATION)
  struct bmp3_quantized_calib_data quant_calib; ///< quantized calibration data
#endif
  uint32_t raw_pressure;            ///< uncompensated pressure
  uint32_t raw_temperature;         ///< uncompensated temperature
  float pressure;                   ///< pressure in Pascal
  float temperature;                ///< temperature in deg Celcius
};

extern void bmp3_i2c_read_eeprom_calib(struct Bmp3_I2c *bmp);
extern void bmp3_i2c_init(struct Bmp3_I2c *bmp, struct i2c_periph *i2c_p, uint8_t addr);
extern void bmp3_i2c_periodic(struct Bmp3_I2c *bmp);
extern void bmp3_i2c_event(struct Bmp3_I2c *bmp);


#endif /* BMP3_I2C_H */

