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
 * @file peripherals/bmp3_regs.h
 * @brief Sensor driver for BMP3 sensor register definition
 *
 * Modified for Paparazzi from SDP3 driver from BoshSensortec
 * see https://github.com/BoschSensortec/BMP3-Sensor-API
 * for original code and license
 *
 */

#ifndef BMP3_REGS_H
#define BMP3_REGS_H

#include "std.h"

/** I2C addresses (8 bits) */
#define BMP3_I2C_ADDR             0xEC
#define BMP3_I2C_ADDR_ALT         0xEE

/**\name BMP3 chip identifier */
#define BMP3_CHIP_ID              0x50

/**\name Register Address */
#define BMP3_CHIP_ID_ADDR         0x00
#define BMP3_ERR_REG_ADDR         0x02
#define BMP3_SENS_STATUS_REG_ADDR 0x03
#define BMP3_DATA_ADDR            0x04
#define BMP3_EVENT_ADDR           0x10
#define BMP3_INT_STATUS_REG_ADDR  0x11
#define BMP3_FIFO_LENGTH_ADDR     0x12
#define BMP3_FIFO_DATA_ADDR       0x14
#define BMP3_FIFO_WM_ADDR         0x15
#define BMP3_FIFO_CONFIG_1_ADDR   0x17
#define BMP3_FIFO_CONFIG_2_ADDR   0x18
#define BMP3_INT_CTRL_ADDR        0x19
#define BMP3_IF_CONF_ADDR         0x1A
#define BMP3_PWR_CTRL_ADDR        0x1B
#define BMP3_OSR_ADDR             0X1C
#define BMP3_ODR_ADDR             0X1D
#define BMP3_CONFIG_ADDR          0X1F
#define BMP3_CALIB_DATA_ADDR      0x31
#define BMP3_CMD_ADDR             0x7E

/**\name Power mode macros */
#define BMP3_SLEEP_MODE           0x00
#define BMP3_FORCED_MODE          0x01
#define BMP3_NORMAL_MODE          0x03

/**\name Over sampling macros */
#define BMP3_NO_OVERSAMPLING      0x00
#define BMP3_OVERSAMPLING_2X      0x01
#define BMP3_OVERSAMPLING_4X      0x02
#define BMP3_OVERSAMPLING_8X      0x03
#define BMP3_OVERSAMPLING_16X     0x04
#define BMP3_OVERSAMPLING_32X     0x05

/**\name Filter setting macros */
#define BMP3_IIR_FILTER_DISABLE   0x00
#define BMP3_IIR_FILTER_COEFF_1   0x01
#define BMP3_IIR_FILTER_COEFF_3   0x02
#define BMP3_IIR_FILTER_COEFF_7   0x03
#define BMP3_IIR_FILTER_COEFF_15  0x04
#define BMP3_IIR_FILTER_COEFF_31  0x05
#define BMP3_IIR_FILTER_COEFF_63  0x06
#define BMP3_IIR_FILTER_COEFF_127 0x07

/**\name Odr setting macros */
#define BMP3_ODR_200_HZ           0x00
#define BMP3_ODR_100_HZ           0x01
#define BMP3_ODR_50_HZ            0x02
#define BMP3_ODR_25_HZ            0x03
#define BMP3_ODR_12_5_HZ          0x04
#define BMP3_ODR_6_25_HZ          0x05
#define BMP3_ODR_3_1_HZ           0x06
#define BMP3_ODR_1_5_HZ           0x07
#define BMP3_ODR_0_78_HZ          0x08
#define BMP3_ODR_0_39_HZ          0x09
#define BMP3_ODR_0_2_HZ           0x0A
#define BMP3_ODR_0_1_HZ           0x0B
#define BMP3_ODR_0_05_HZ          0x0C
#define BMP3_ODR_0_02_HZ          0x0D
#define BMP3_ODR_0_01_HZ          0x0E
#define BMP3_ODR_0_006_HZ         0x0F
#define BMP3_ODR_0_003_HZ         0x10
#define BMP3_ODR_0_001_HZ         0x11

/**\name Sensor component selection macros */
#define BMP3_PRESS                0x01
#define BMP3_TEMP                 0x02
#define BMP3_ALL                  0x03

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP3_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

/**\name Macros related to size */
#define BMP3_CALIB_DATA_LEN           21
#define BMP3_P_AND_T_HEADER_DATA_LEN  7
#define BMP3_P_OR_T_HEADER_DATA_LEN   4
#define BMP3_P_T_DATA_LEN             6
#define BMP3_P_DATA_LEN               3
#define BMP3_T_DATA_LEN               3
#define BMP3_SENSOR_TIME_LEN          3

/**
 * @brief Use double, single (float) or integer for temperature and pressure compensation
 */
#define BMP3_DOUBLE_PRECISION_COMPENSATION 1
#define BMP3_SINGLE_PRECISION_COMPENSATION 2
#define BMP3_INTEGER_COMPENSATION 3

/**
 * @brief By default use single precision compensation
 */
#ifndef BMP3_COMPENSATION
#define BMP3_COMPENSATION BMP3_SINGLE_PRECISION_COMPENSATION
#endif

/**
 * @brief Status enum
 */
enum Bmp3Status {
  BMP3_STATUS_UNINIT,
  BMP3_STATUS_GET_CALIB,
  BMP3_STATUS_CONFIGURE,
  BMP3_STATUS_READ_DATA
};

/**
 * @brief Register Trim Variables
 */
struct bmp3_reg_calib_data {
  uint16_t par_t1;
  uint16_t par_t2;
  int8_t par_t3;
  int16_t par_p1;
  int16_t par_p2;
  int8_t par_p3;
  int8_t par_p4;
  uint16_t par_p5;
  uint16_t par_p6;
  int8_t par_p7;
  int8_t par_p8;
  int16_t par_p9;
  int8_t par_p10;
  int8_t par_p11;
  int64_t t_lin;
};

#if BMP3_COMPENSATION == BMP3_DOUBLE_PRECISION_COMPENSATION
/**
 * @brief Quantized Trim Variables
 */
struct bmp3_quantized_calib_data {
  double par_t1;
  double par_t2;
  double par_t3;
  double par_p1;
  double par_p2;
  double par_p3;
  double par_p4;
  double par_p5;
  double par_p6;
  double par_p7;
  double par_p8;
  double par_p9;
  double par_p10;
  double par_p11;
  double t_lin;
};

#elif BMP3_COMPENSATION == BMP3_SINGLE_PRECISION_COMPENSATION
/**
 * @brief Quantized Trim Variables
 */
struct bmp3_quantized_calib_data {
  float par_t1;
  float par_t2;
  float par_t3;
  float par_p1;
  float par_p2;
  float par_p3;
  float par_p4;
  float par_p5;
  float par_p6;
  float par_p7;
  float par_p8;
  float par_p9;
  float par_p10;
  float par_p11;
  float t_lin;
};

#endif

#endif /* BMP3_REGS_H */

