/*
 *
 * Copyright (C) 2012 Piotr Esden-Tempski
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

/* Register definition for MS5611
 */

#ifndef MS5611_H
#define MS5611_H

/* default i2c address
 * when CSB is set to GND addr is 0xEE
 * when CSB is set to VCC addr is 0xEC
 *
 * Note: Aspirin 2.1 has CSB bound to GND.
 */
#define MS5611_SLAVE_ADDR 0xEE

/* FIXME: For backwards compatibility with Aspirin driver (it doesnt talk to baro either) */
#define MS5611_ADDR0 0x77
#define MS5611_ADDR1 0x76

/* SPI SLAVE3 is on pin PC13 
 * Aspirin 2.2 has ms5611 on SPI bus
 */
#ifndef MS5611_SLAVE_DEV
#define MS5611_SLAVE_DEV SPI_SLAVE3
#endif

/* Number of 16bit calibration coefficients */
#define PROM_NB                 8

/* OSR definitions */
#define MS5611_OSR256        0x02
#define MS5611_OSR512        0x02
#define MS5611_OSR1024       0x04
#define MS5611_OSR2048       0x06
#define MS5611_OSR4096       0x08

/* D1 Register defines */
#define MS5611_REG_D1R       0x40 // Request D1 (pressure) conversion
#define MS5611_REG_D1(_osr)  (MS5611_REG_D1R | _osr)
#define MS5611_REG_D1OSR256  MS5611_REG_D1(MS5611_ORS256)
#define MS5611_REG_D1OSR512  MS5611_REG_D1(MS5611_OSR512)
#define MS5611_REG_D1OSR1024 MS5611_REG_D1(MS5611_OSR1024)
#define MS5611_REG_D1OSR2048 MS5611_REG_D1(MS5611_OSR2048)
#define MS5611_REG_D1OSR4096 MS5611_REG_D1(MS5611_OSR4096)

/* D2 register defines */
#define MS5611_REG_D2R       0x50 // Request D2 (temperature) conversion
#define MS5611_REG_D2(_osr)  (MS5611_REG_D2R | _osr)
#define MS5611_REG_D2OSR256  MS5611_REG_D2(MS5611_ORS256)
#define MS5611_REG_D2OSR512  MS5611_REG_D2(MS5611_OSR512)
#define MS5611_REG_D2OSR1024 MS5611_REG_D2(MS5611_OSR1024)
#define MS5611_REG_D2OSR2048 MS5611_REG_D2(MS5611_OSR2048)
#define MS5611_REG_D2OSR4096 MS5611_REG_D2(MS5611_OSR4096)

/* Commands */
#define MS5611_ADC_READ         0x00 // Read converted value
#define MS5611_SOFT_RESET       0x1E // Reset command
#define MS5611_PROM_READ        0xA0 // Start reading PROM
#define MS5611_START_CONV_D1    MS5611_REG_D1OSR4096 /* we use OSR=4096 for maximum resolution */
#define MS5611_START_CONV_D2    MS5611_REG_D2OSR4096 /* we use OSR=4096 for maximum resolution */

/* FIXME: backwards compatibility with Aspirin driver */
#define MS5611_REG_RESET MS5611_SOFT_RESET
#define MS5611_REG_ADCREAD MS5611_ADC_READ

enum ms5611_stat{
  MS5611_UNINIT,
  MS5611_RESET,
  MS5611_RESET_OK,
  MS5611_PROM,
  MS5611_IDLE,
  MS5611_CONV_D1,
  MS5611_CONV_D1_OK,
  MS5611_ADC_D1,
  MS5611_CONV_D2,
  MS5611_CONV_D2_OK,
  MS5611_ADC_D2
};

#endif /* MS5611_H */
