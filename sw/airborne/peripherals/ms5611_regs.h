/*
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

/**
 * @file peripherals/ms5611_regs.h
 * Register definitions for MS5611 barometer.
 */

#ifndef MS5611_REGS_H
#define MS5611_REGS_H

/** default i2c address
 * when CSB is set to GND addr is 0xEE
 * when CSB is set to VCC addr is 0xEC
 */
#define MS5611_I2C_SLAVE_ADDR 0xEE
#define MS5611_I2C_SLAVE_ADDR_ALT 0xEC

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

#endif /* MS5611_REGS_H */
