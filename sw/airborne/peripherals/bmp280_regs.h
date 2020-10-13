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
 */

/**
 * @file peripherals/bmp3_regs.h
 * @brief Sensor driver for BMP280 sensor register definition
 *
 * Modified for Paparazzi from SDP3 driver from BoshSensortec
 * see https://github.com/BoschSensortec/BMP280-Sensor-API
 * for original code and license
 *
 */

#ifndef BMP280_REGS_H
#define BMP280_REGS_H

#include "std.h"

// I2C addresses (8 bits)
#define BMP280_I2C_ADDR             0xEC
#define BMP280_I2C_ADDR_ALT         0xEE

/**\name BMP280 chip identifier */
#define BMP280_CHIP_ID              0x50

// CALIBRATION REGISTER ADDRESSES
#define BMP280_CALIB_LSB_DATA_ADDR  0x88
#define BMP280_CALIB_DATA_LEN       24

#define BMP280_DIG_T1_UINT          0x88
#define BMP280_DIG_T2_INT       0x8A
#define BMP280_DIG_T3_INT       0x8C
#define BMP280_DIG_P1_UINT          0x8E
#define BMP280_DIG_P2_INT           0x90
#define BMP280_DIG_P3_INT       0x92
#define BMP280_DIG_P4_INT       0x94
#define BMP280_DIG_P5_INT           0x96
#define BMP280_DIG_P6_INT           0x98
#define BMP280_DIG_P7_INT       0x9A
#define BMP280_DIG_P8_INT       0x9C
#define BMP280_DIG_P9_INT           0x9E

// CONTROL AND VALUES REGISTER ADDRESSES
#define BMP280_CONFIG_ADDR      0xF4
#define BMP280_CONFIG_LEN     0x02
#define BMP280_DATA_START_REG_ADDR  0xF7
#define BMP280_P_T_DATA_LEN         6
#define BMP280_P_DATA_LEN           3
#define BMP280_T_DATA_LEN           3

#define BMP280_CHIP_ID_REG_ADDR     0xD0
#define BMP280_RESET_REG_ADDR       0xF3
#define BMP280_STATUS_REG_ADDR      0xF3
#define BMP280_CTRL_MEAS_REG_ADDR   0xF4
#define BMP280_CONFIG_REG_ADDR      0xF5
#define BMP280_P_MSB_REG_ADDR       0xF7
#define BMP280_P_LSB_REG_ADDR       0xF8
#define BMP280_P_XLSB_REG_ADDR      0xF9
#define BMP280_T_MSB_REG_ADDR       0xFA
#define BMP280_T_LSB_REG_ADDR       0xFB
#define BMP280_T_XLSB_REG_ADDR      0xFC

// BMP280 ID
#define BMP280_ID_NB        0x58
//BMP280 RESET COMMAND VALUE
#define BMP280_RESET_VAL      0xB6

#define BMP280_EOC_BIT        (1<<3)
#define BMP280_NVRAM_COPY_BIT     (1<<0)

// BMP280 CONTROL MEASUREMENT REGISTER BIT VALUES.
#define BMP280_NO_OVERSAMPLING_T    (0x00<<5)
#define BMP280_OVERSAMPLING_1X_T    (0x01<<5)
#define BMP280_OVERSAMPLING_2X_T    (0x02<<5)
#define BMP280_OVERSAMPLING_4X_T    (0x03<<5)
#define BMP280_OVERSAMPLING_8X_T    (0x04<<5)
#define BMP280_OVERSAMPLING_16X_T   (0x05<<5)

#define BMP280_NO_OVERSAMPLING_P    (0x00<<2)
#define BMP280_OVERSAMPLING_1X_P    (0x01<<2)
#define BMP280_OVERSAMPLING_2X_P    (0x02<<2)
#define BMP280_OVERSAMPLING_4X_P    (0x03<<2)
#define BMP280_OVERSAMPLING_8X_P    (0x04<<2)
#define BMP280_OVERSAMPLING_16X_P   (0x05<<2)

#define BMP280_POWER_SLEEP_MODE     (0x00)
#define BMP280_POWER_FORCED_MODE    (0x01) // OX02 IS EXACTLY THE SAME
#define BMP280_POWER_NORMAL_MODE    (0x03)

// BMP280 CONFIG REGISTER BIT VALUES
#define BMP280_INACTIVITY_HALF_MS   (0x00<<5)
#define BMP280_INACTIVITY_62_5_MS   (0x01<<5)
#define BMP280_INACTIVITY_125_MS    (0x02<<5)
#define BMP280_INACTIVITY_250_MS    (0x03<<5)
#define BMP280_INACTIVITY_500_MS    (0x04<<5)
#define BMP280_INACTIVITY_1000_MS   (0x05<<5)
#define BMP280_INACTIVITY_2000_MS   (0x06<<5)
#define BMP280_INACTIVITY_4000_MS   (0x07<<5)

#define BMP280_IIR_FILTER_COEFF_1   (0x00<<2)
#define BMP280_IIR_FILTER_COEFF_2   (0x01<<2)
#define BMP280_IIR_FILTER_COEFF_4   (0x02<<2)
#define BMP280_IIR_FILTER_COEFF_8   (0x03<<2)
#define BMP280_IIR_FILTER_COEFF_16  (0x04<<2)

#define BMP280_DISABLE_SPI_IF       (0x00) // DEFAULT
#define BMP280_ENABLE_SPI_IF        (0x01)


/**\name Power mode macros */
#define BMP280_SLEEP_MODE           0x00
#define BMP280_FORCED_MODE          0x01
#define BMP280_NORMAL_MODE          0x03


/**\name Sensor component selection macros */
#define BMP280_PRESS                0x01
#define BMP280_TEMP                 0x02
#define BMP280_ALL                  0x03

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP280_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#ifndef BMP280_COMPENSATION
#define BMP280_COMPENSATION BMP280_DOUBLE_PRECISION_COMPENSATION
#endif

/**
 * @brief Status enum
 */
enum Bmp280Status {
  BMP280_STATUS_UNINIT,
  BMP280_STATUS_GET_CALIB,
  BMP280_STATUS_CONFIGURE,
  BMP280_STATUS_READ_STATUS_REG,
  BMP280_STATUS_READ_DATA_REGS
};

// brief Register Trim Variables
struct bmp280_reg_calib_data {
  uint16_t dig_t1;
  int16_t dig_t2;
  int16_t dig_t3;
  uint16_t dig_p1;
  int16_t dig_p2;
  int16_t dig_p3;
  int16_t dig_p4;
  int16_t dig_p5;
  int16_t dig_p6;
  int16_t dig_p7;
  int16_t dig_p8;
  int16_t dig_p9;
  int32_t t_fine;
};


#endif /* BMP280_REGS_H */
