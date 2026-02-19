/*
 * Florian Sansou florian.sansou@enac.fr
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
 * @file peripherals/spa06_regs.h
 * @brief Sensor driver for SPA06/SPL06 sensor register definition
 *
 * 
 *
 */

#ifndef SPA06_REGS_H
#define SPA06_REGS_H

#include "std.h"

// I2C addresses (8 bits)

#define SPA06_I2C_ADDR                         0xec //0x76 -> 0xef  (if the SDO pin is pulled-down to GND)
#define SPA06_I2C_ADDR_ALT                     0xee //0x77 -> 0xee (default) 

/**\ chip identifier */
#define SPL06_CHIP_ID                          0x10
#define SPA06_CHIP_ID                          0x11

#define SPL06_READ_FLAG                        0x80

#define SPL06_REG_PRESSURE_B2                  0x00    // Pressure MSB Register
#define SPL06_REG_PRESSURE_B1                  0x01    // Pressure middle byte Register
#define SPL06_REG_PRESSURE_B0                  0x02    // Pressure LSB Register
#define SPL06_REG_PRESSURE_START               SPL06_REG_PRESSURE_B2
#define SPL06_PRESSURE_LEN                     3       // 24 bits, 3 bytes
#define SPL06_REG_TEMPERATURE_B2               0x03    // Temperature MSB Register
#define SPL06_REG_TEMPERATURE_B1               0x04    // Temperature middle byte Register
#define SPL06_REG_TEMPERATURE_B0               0x05    // Temperature LSB Register
#define SPL06_REG_TEMPERATURE_START            SPL06_REG_TEMPERATURE_B2
#define SPL06_TEMPERATURE_LEN                  3       // 24 bits, 3 bytes
#define SPL06_REG_PRESSURE_CFG                 0x06    // Pressure config
#define SPL06_REG_TEMPERATURE_CFG              0x07    // Temperature config
#define SPL06_REG_MODE_AND_STATUS              0x08    // Mode and status
#define SPL06_REG_INT_AND_FIFO_CFG             0x09    // Interrupt and FIFO config
#define SPL06_REG_INT_STATUS                   0x0A    // Interrupt and FIFO config
#define SPL06_REG_FIFO_STATUS                  0x0B    // Interrupt and FIFO config
#define SPL06_REG_RST                          0x0C    // Softreset Register
#define SPL06_RESET_BIT_SOFT_RST               0x09    // 0b1001
#define SPL06_REG_CHIP_ID                      0x0D    // Chip ID Register
#define SPL06_REG_CALIB_COEFFS_START           0x10
#define SPL06_REG_CALIB_COEFFS_END             0x21
#define SPA06_REG_CALIB_COEFFS_END             0x24

// PRESSURE_CFG_REG
#define SPL06_PRES_RATE_1HZ				             (0x00 << 4)
#define SPL06_PRES_RATE_4HZ				             (0x02 << 4)
#define SPL06_PRES_RATE_8HZ				             (0x03 << 4)
#define SPL06_PRES_RATE_32HZ				           (0x05 << 4)

// TEMPERATURE_CFG_REG
#define SPL06_TEMP_USE_EXT_SENSOR              (1<<7)
#define SPL06_TEMP_RATE_1HZ				             (0x00)
#define SPL06_TEMP_RATE_32HZ				           (0x05 << 4)

// MODE_AND_STATUS_REG
#define SPL06_MEAS_PRESSURE                    (1<<0)  // measure pressure
#define SPL06_MEAS_TEMPERATURE                 (1<<1)  // measure temperature
#define SPL06_MEAS_CON_PRE_TEM				         0x07  // Continuous pressure and temperature measurement

#define SPL06_MEAS_CFG_CONTINUOUS              (1<<2)
#define SPL06_MEAS_CFG_PRESSURE_RDY            (1<<4)
#define SPL06_MEAS_CFG_TEMPERATURE_RDY         (1<<5)
#define SPL06_MEAS_CFG_SENSOR_RDY              (1<<6)
#define SPL06_MEAS_CFG_COEFFS_RDY              (1<<7)

// INT_AND_FIFO_CFG_REG
#define SPL06_PRESSURE_RESULT_BIT_SHIFT        (1<<2)  // necessary for pressure oversampling > 8
#define SPL06_TEMPERATURE_RESULT_BIT_SHIFT     (1<<3)  // necessary for temperature oversampling > 8

#define SPL06_OVERSAMPLING_1X_T                0x00 // single. (Default) - Measurement time 3.6 ms
#define SPL06_OVERSAMPLING_2X_T                0x01 // 2 times
#define SPL06_OVERSAMPLING_4X_T                0x02 // 4 times
#define SPL06_OVERSAMPLING_8X_T                0x03 // 8 times.
#define SPL06_OVERSAMPLING_16X_T               0x04 // 16 times
#define SPL06_OVERSAMPLING_32X_T               0x05 // 32 times  
#define SPL06_OVERSAMPLING_64X_T               0x06 // 64 times 
#define SPL06_OVERSAMPLING_128X_T              0x07 // 128 times 

#define SPL06_OVERSAMPLING_1X_P                0x00 // Single. (Low Precision)
#define SPL06_OVERSAMPLING_2X_P                0x01 // 2 times (Low Power)
#define SPL06_OVERSAMPLING_4X_P                0x02 // 4 times
#define SPL06_OVERSAMPLING_8X_P                0x03 // 8 times.
#define SPL06_OVERSAMPLING_16X_P               0x04 // 16 times (Standard) Use in combination with a bit shift
#define SPL06_OVERSAMPLING_32X_P               0x05 // 32 times  Use in combination with a bit shift
#define SPL06_OVERSAMPLING_64X_P               0x06 // 64 times (High Precision) Use in combination with a bit shift
#define SPL06_OVERSAMPLING_128X_P              0x07 // 128 times Use in combination with a bit shift



#endif /* SPA06_REGS_H */
