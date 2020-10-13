/*
 * Chris Efstathiou hendrixgr@gmail.com
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
 * @file peripherals/bmp280_i2c.h
 * @brief Sensor driver for BMP280 sensor via I2C
 *
 *
 */

#ifndef BMP280_I2C_H
#define BMP280_I2C_H

/* Header includes */
#include "peripherals/bmp280_regs.h"
#include "mcu_periph/i2c.h"


struct Bmp280_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Bmp280Status status;           ///< state machine status
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  struct bmp280_reg_calib_data calib; ///< calibration data
  uint32_t raw_pressure;            ///< uncompensated pressure
  uint32_t raw_temperature;         ///< uncompensated temperature
  float pressure;                   ///< pressure in Pascal
  float temperature;                ///< temperature in deg Celcius
};

extern void bmp280_i2c_read_eeprom_calib(struct Bmp280_I2c *bmp);
extern void bmp280_i2c_init(struct Bmp280_I2c *bmp, struct i2c_periph *i2c_p, uint8_t addr);
extern void bmp280_i2c_periodic(struct Bmp280_I2c *bmp);
extern void bmp280_i2c_event(struct Bmp280_I2c *bmp);

#endif /* BMP280_I2C_H */
