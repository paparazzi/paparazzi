/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/lsm303d_regs.h
 * Register defs for ST LSM303D 3D accelerometer and magnetometer.
 *
 * Has an I2C and SPI interface.
 * The LSM303D has linear acceleration full scales of ±2g / ±4g / ±6g / ±8g / ±16g
 * and a magnetic field full scale of ±2 / ±4 / ±8 / ±12 gauss.
 */

#ifndef LSM303D_REGS_H
#define LSM303D_REGS_H

/* Registers */
#define LSM303D_REG_TEMP_OUT_L   0x05
#define LSM303D_REG_TEMP_OUT_H   0x06
#define LSM303D_REG_STATUS_M     0x07
#define LSM303D_REG_OUT_X_L_M    0x08
#define LSM303D_REG_OUT_X_H_M    0x09
#define LSM303D_REG_OUT_Y_L_M    0x0A
#define LSM303D_REG_OUT_Y_H_M    0x0B
#define LSM303D_REG_OUT_Z_L_M    0x0C
#define LSM303D_REG_OUT_Z_H_M    0x0D

#define LSM303D_REG_WHO_AM_I     0x0F

#define LSM303D_REG_INT_CTRL_M   0x12
#define LSM303D_REG_INT_SRC_M    0x13
#define LSM303D_REG_INT_THS_L_M  0x14
#define LSM303D_REG_INT_THS_H_M  0x15
#define LSM303D_REG_OFFSET_X_L_M 0x16
#define LSM303D_REG_OFFSET_X_H_M 0x17
#define LSM303D_REG_OFFSET_Y_L_M 0x18
#define LSM303D_REG_OFFSET_Y_H_M 0x19
#define LSM303D_REG_OFFSET_Z_L_M 0x1A
#define LSM303D_REG_OFFSET_Z_H_M 0x1B
#define LSM303D_REG_REFERENCE_X  0x1C
#define LSM303D_REG_REFERENCE_Y  0x1D
#define LSM303D_REG_REFERENCE_Z  0x1E
#define LSM303D_REG_CTRL0        0x1F
#define LSM303D_REG_CTRL1        0x20
#define LSM303D_REG_CTRL2        0x21
#define LSM303D_REG_CTRL3        0x22
#define LSM303D_REG_CTRL4        0x23
#define LSM303D_REG_CTRL5        0x24
#define LSM303D_REG_CTRL6        0x25
#define LSM303D_REG_CTRL7        0x26
#define LSM303D_REG_STATUS_A     0x27
#define LSM303D_REG_OUT_X_L_A    0x28
#define LSM303D_REG_OUT_X_H_A    0x29
#define LSM303D_REG_OUT_Y_L_A    0x2A
#define LSM303D_REG_OUT_Y_H_A    0x2B
#define LSM303D_REG_OUT_Z_L_A    0x2C
#define LSM303D_REG_OUT_Z_H_A    0x2D
#define LSM303D_REG_FIFO_CTRL    0x2E
#define LSM303D_REG_FIFO_SRC     0x2F
#define LSM303D_REG_IG_CFG1      0x30
#define LSM303D_REG_IG_SRC1      0x31
#define LSM303D_REG_IG_THS1      0x32
#define LSM303D_REG_IG_DUR1      0x33
#define LSM303D_REG_IG_CFG2      0x34
#define LSM303D_REG_IG_SRC2      0x35
#define LSM303D_REG_IG_THS2      0x36
#define LSM303D_REG_IG_DUR2      0x37
#define LSM303D_REG_CLICK_CFG    0x38
#define LSM303D_REG_CLICK_SRC    0x39
#define LSM303D_REG_CLICK_THS    0x3A
#define LSM303D_REG_TIME_LIMIT   0x3B
#define LSM303D_REG_TIME_LATENCY 0x3C
#define LSM303D_REG_TIME_WINDOW  0x3D
#define LSM303D_REG_ACT_THS      0x3E
#define LSM303D_REG_ACT_DUR      0x3F

/** LSM303D device identifier in LSM303D_REG_WHO_AM_I */
#define LSM303D_WHO_AM_I         0x49

/** LSM303D acceleration data rate (bits 4-7 in LSM303D_REG_CTRL1) */
enum Lsm303dAccelRates {
  LSM303D_ACC_RATE_OFF      = 0x0,
  LSM303D_ACC_RATE_3_125HZ  = 0x1,
  LSM303D_ACC_RATE_6_25HZ   = 0x2,
  LSM303D_ACC_RATE_12_5HZ   = 0x3,
  LSM303D_ACC_RATE_25HZ     = 0x4,
  LSM303D_ACC_RATE_50HZ     = 0x5,
  LSM303D_ACC_RATE_100HZ    = 0x6,
  LSM303D_ACC_RATE_200HZ    = 0x7,
  LSM303D_ACC_RATE_400HZ    = 0x8,
  LSM303D_ACC_RATE_800HZ    = 0x9,
  LSM303D_ACC_RATE_1600HZ   = 0xA
};

/* Bit definitions for LSM303D_REG_CTRL1 */
#define LSM303D_AXEN (1 << 0)
#define LSM303D_AYEN (1 << 1)
#define LSM303D_AZEN (1 << 3)

/** LSM303D accelerometer anti-alias filter bandwidth (bits 6-7 in LSM303D_REG_CTRL2) */
enum Lsm303dAccelBandwidth {
  LSM303D_ACC_BW_773HZ = 0,
  LSM303D_ACC_BW_194HZ = 1,
  LSM303D_ACC_BW_362HZ = 2,
  LSM303D_ACC_BW_50HZ  = 3
};

/** LSM303D accelerometer anti-alias filter bandwidth (bits 3-5 in LSM303D_REG_CTRL2) */
enum Lsm303dAccelRanges {
  LSM303D_ACC_RANGE_2G  = 0x0,
  LSM303D_ACC_RANGE_4G  = 0x1,
  LSM303D_ACC_RANGE_6G  = 0x2,
  LSM303D_ACC_RANGE_8G  = 0x3,
  LSM303D_ACC_RANGE_16G = 0x4
};

/** LSM303D magnetic data rate (bits 2-4 in LSM303D_REG_CTRL5) */
enum Lsm303dMagRates {
  LSM303D_MAG_RATE_3_125HZ  = 0x0,
  LSM303D_MAG_RATE_6_25HZ   = 0x1,
  LSM303D_MAG_RATE_12_5HZ   = 0x2,
  LSM303D_MAG_RATE_25HZ     = 0x3,
  LSM303D_MAG_RATE_50HZ     = 0x4,
  LSM303D_MAG_RATE_100HZ    = 0x5
};

/** LSM303D magnetic range (bits 5-6 in LSM303D_REG_CTRL6) */
enum Lsm303dMagRange {
  LSM303D_MAG_RANGE_2GAUSS  = 0,
  LSM303D_MAG_RANGE_4GAUSS  = 1,
  LSM303D_MAG_RANGE_8GAUSS  = 2,
  LSM303D_MAG_RANGE_12GAUSS = 3,
};

#endif // LSM303D_REGS_H
