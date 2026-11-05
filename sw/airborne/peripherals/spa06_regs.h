/*
 * Copyright (C) 2026 OpenUAS
 * Thanks to Florian Sansou florian.sansou@enac.fr for initial implementation
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
 * @file peripherals/spa06_regs.h
 * @brief Register definitions for the Goertek SPA06-003 / SPL06-001 barometer
 *
 * Registers shared by both devices use the SPL06_ prefix; SPA06_ marks
 * SPA06-only differences (chip ID, extended calibration coefficients).
 */

#ifndef SPA06_REGS_H
#define SPA06_REGS_H

#include "std.h"

// I2C addresses (8-bit, i.e. 7-bit address << 1)

#define SPA06_I2C_ADDR                         0xec   // 0x76 << 1 (SDO pin pulled down to GND)
#define SPA06_I2C_ADDR_ALT                     0xee   // 0x77 << 1 (SDO pin high, default)

/** Chip identifiers, register 0x0D: product ID [3:0], revision ID [7:4] */
#define SPL06_CHIP_ID                          0x10   // SPL06-001 (product 0, revision 1)
#define SPA06_CHIP_ID                          0x11   // SPA06-003 (product 1, revision 1)

#define SPL06_READ_FLAG                        0x80   // OR into the register address for SPI reads (bit 7 = RW = '1')

#define SPL06_REG_PRESSURE_B2                  0x00   // Pressure MSB Register
#define SPL06_REG_PRESSURE_B1                  0x01   // Pressure middle byte Register
#define SPL06_REG_PRESSURE_B0                  0x02   // Pressure LSB Register
#define SPL06_REG_PRESSURE_START               SPL06_REG_PRESSURE_B2
#define SPL06_PRESSURE_LEN                     3      // 24 bits, 3 bytes
#define SPL06_REG_TEMPERATURE_B2               0x03   // Temperature MSB Register
#define SPL06_REG_TEMPERATURE_B1               0x04   // Temperature middle byte Register
#define SPL06_REG_TEMPERATURE_B0               0x05   // Temperature LSB Register
#define SPL06_REG_TEMPERATURE_START            SPL06_REG_TEMPERATURE_B2
#define SPL06_TEMPERATURE_LEN                  3      // 24 bits, 3 bytes
#define SPL06_REG_PRESSURE_CFG                 0x06   // Pressure config
#define SPL06_REG_TEMPERATURE_CFG              0x07   // Temperature config
#define SPL06_REG_MODE_AND_STATUS              0x08   // Mode and status
#define SPL06_REG_INT_AND_FIFO_CFG             0x09   // Interrupt and FIFO config
#define SPL06_REG_INT_STATUS                   0x0A   // Interrupt status
#define SPL06_REG_FIFO_STATUS                  0x0B   // FIFO status
#define SPL06_REG_RST                          0x0C   // Softreset Register
#define SPL06_RESET_BIT_SOFT_RST               0x09   // 0b1001: write to RST to trigger a full soft reset (~40ms until ready)
#define SPL06_REG_CHIP_ID                      0x0D   // Chip ID Register
#define SPL06_REG_CALIB_COEFFS_START           0x10   // First calibration coefficient register (c0)
#define SPL06_REG_CALIB_COEFFS_END             0x21   // Last coefficient on the SPL06 (c30), 18 bytes total
#define SPA06_REG_CALIB_COEFFS_END             0x24   // Last coefficient on the SPA06 (c40), 21 bytes total (adds c31/c40)
#define SPL06_REG_COEF_SRCE                    0x28   // Calibration coefficients temperature sensor source
#define SPL06_COEF_SRCE_BIT_TMP_COEF_SRCE      (1<<7) // Mirror this bit into TMP_CFG bit 7 (TMP_EXT)

// PRESSURE_CFG_REG
// Background mode measurement rate, bits [6:4] (PM_RATE), rate code = log2(measurements/s).
// Keep the total conversion time of pressure + temperature below 1s per second.
#define SPL06_PRES_RATE_1HZ                    (0x00 << 4) // 1 pressure measurement per second
#define SPL06_PRES_RATE_4HZ                    (0x02 << 4) // 4 pressure measurements per second
#define SPL06_PRES_RATE_8HZ                    (0x03 << 4) // 8 pressure measurements per second
#define SPL06_PRES_RATE_32HZ                   (0x05 << 4) // 32 pressure measurements per second

// TEMPERATURE_CFG_REG
// Rate in bits [6:4] (TMP_RATE), same coding and 1s/s conversion time budget as PM_RATE above.
#define SPL06_TEMP_USE_EXT_SENSOR              (1<<7)      // TMP_EXT: measure with the external (MEMS) sensor; must match TMP_COEF_SRCE
#define SPL06_TEMP_RATE_1HZ                    (0x00 << 4) // 1 temperature measurement per second
#define SPL06_TEMP_RATE_4HZ                    (0x02 << 4) // 4 temperature measurements per second
#define SPL06_TEMP_RATE_32HZ                   (0x05 << 4) // 32 temperature measurements per second

// MODE_AND_STATUS_REG
// Measurement control in bits [2:0] (MEAS_CTRL), status flags in bits [7:4].
#define SPL06_MEAS_PRESSURE                    (1<<0) // one-shot pressure measurement command
#define SPL06_MEAS_TEMPERATURE                 (1<<1) // one-shot temperature measurement command
#define SPL06_MEAS_CON_PRE_TEM                 0x07   // MEAS_CTRL 111: continuous pressure and temperature measurement

#define SPL06_MEAS_CFG_CONTINUOUS              (1<<2) // background (continuous) measurement mode bit
#define SPL06_MEAS_CFG_PRESSURE_RDY            (1<<4) // new pressure result available, cleared on read
#define SPL06_MEAS_CFG_TEMPERATURE_RDY         (1<<5) // new temperature result available, cleared on read
#define SPL06_MEAS_CFG_SENSOR_RDY              (1<<6) // sensor initialization complete after power-up/reset
#define SPL06_MEAS_CFG_COEFFS_RDY              (1<<7) // calibration coefficients valid (~40ms after power-up)

// INT_AND_FIFO_CFG_REG
#define SPL06_PRESSURE_RESULT_BIT_SHIFT        (1<<2) // necessary for pressure oversampling > 8
#define SPL06_TEMPERATURE_RESULT_BIT_SHIFT     (1<<3) // necessary for temperature oversampling > 8

// Oversampling codes for TMP_CFG bits [3:0] (TMP_PRC).
// Each code has its own compensation scale factor, see raw_value_scale_factor().
#define SPL06_OVERSAMPLING_1X_T                0x00   // single. (Default) - Measurement time 3.6 ms
#define SPL06_OVERSAMPLING_2X_T                0x01   // 2 times
#define SPL06_OVERSAMPLING_4X_T                0x02   // 4 times
#define SPL06_OVERSAMPLING_8X_T                0x03   // 8 times.
#define SPL06_OVERSAMPLING_16X_T               0x04   // 16 times
#define SPL06_OVERSAMPLING_32X_T               0x05   // 32 times  
#define SPL06_OVERSAMPLING_64X_T               0x06   // 64 times 
#define SPL06_OVERSAMPLING_128X_T              0x07   // 128 times 

// Oversampling codes for PRS_CFG bits [3:0] (PM_PRC). Measurement times:
// 1x=3.6ms 2x=5.2ms 4x=8.4ms 8x=14.8ms 16x=27.6ms 32x=53.2ms 64x=104.4ms 128x=206.8ms
#define SPL06_OVERSAMPLING_1X_P                0x00   // Single. (Low Precision)
#define SPL06_OVERSAMPLING_2X_P                0x01   // 2 times (Low Power)
#define SPL06_OVERSAMPLING_4X_P                0x02   // 4 times
#define SPL06_OVERSAMPLING_8X_P                0x03   // 8 times.
#define SPL06_OVERSAMPLING_16X_P               0x04   // 16 times (Standard) Use in combination with a bit shift
#define SPL06_OVERSAMPLING_32X_P               0x05   // 32 times  Use in combination with a bit shift
#define SPL06_OVERSAMPLING_64X_P               0x06   // 64 times (High Precision) Use in combination with a bit shift
#define SPL06_OVERSAMPLING_128X_P              0x07   // 128 times Use in combination with a bit shift

#endif /* SPA06_REGS_H */
