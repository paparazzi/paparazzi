/*
 * Chris Efstathiou hendrixgr@gmail.com
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
 *
 */

/**
 * @file modules/sensors/baro_bmp280_i2c.h
 * Bosch BMP280 I2C sensor interface.
 *
 * This reads the values for pressure and temperature from the Bosch BMP280 sensor through I2C.
 */

#ifndef BARO_BMP280_I2C_H
#define BARO_BMP280_I2C_H

#include "peripherals/bmp280_i2c.h"

extern struct Bmp280_I2c baro_bmp280;

extern float baro_alt;
extern  bool baro_alt_valid;

void baro_bmp280_init(void);
void baro_bmp280_periodic(void);
void baro_bmp280_event(void);

#endif
