/*
 * Copyright (C) 2019 Alexis Cornard <alexiscornard@gmail.com>
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
 * @file peripherals/lsm6ds33_regs.h
 * Register defs for ST LSM6DS33 3D accelerometer and gyroscope.
 *
 * Has an I2C interface.
 * The LSM6DS33 has linear acceleration full scales of ±2g / ±4g / ±8g / ±16g
 * and an angular rate range of  ±245 / ±500 / ±1000 / ±2000 dps.
 */

#ifndef LSM6_REGS_H
#define LSM6_REGS_H

// I2C Address
#define LSM6_ADDR                  0xD6

/* Registers */
#define LSM6_REG_FUNC_CFG_ACCESS   0x01

#define LSM6_REG_FIFO_CTRL1        0x06
#define LSM6_REG_FIFO_CTRL2        0x07
#define LSM6_REG_FIFO_CTRL3        0x08
#define LSM6_REG_FIFO_CTRL4        0x09
#define LSM6_REG_FIFO_CTRL5        0x0A
#define LSM6_REG_ORIENT_CFG_G      0x0B

#define LSM6_REG_INT1_CTRL         0x0D
#define LSM6_REG_INT2_CTRL         0x0E
#define LSM6_REG_WHO_AM_I          0x0F
#define LSM6_REG_CTRL1_XL          0x10
#define LSM6_REG_CTRL2_G           0x11
#define LSM6_REG_CTRL3_C           0x12
#define LSM6_REG_CTRL4_C           0x13
#define LSM6_REG_CTRL5_C           0x14
#define LSM6_REG_CTRL6_C           0x15
#define LSM6_REG_CTRL7_G           0x16
#define LSM6_REG_CTRL8_XL          0x17
#define LSM6_REG_CTRL9_XL          0x18
#define LSM6_REG_CTRL10_C          0x19

#define LSM6_REG_WAKE_UP_SRC       0x1B
#define LSM6_REG_TAP_SRC           0x1C
#define LSM6_REG_D6D_SRC           0x1D
#define LSM6_REG_STATUS_REG        0x1E

#define LSM6_REG_OUT_TEMP_L        0x20
#define LSM6_REG_OUT_TEMP_H        0x21
#define LSM6_REG_OUTX_L_G          0x22
#define LSM6_REG_OUTX_H_G          0x23
#define LSM6_REG_OUTY_L_G          0x24
#define LSM6_REG_OUTY_H_G          0x25
#define LSM6_REG_OUTZ_L_G          0x26
#define LSM6_REG_OUTZ_H_G          0x27
#define LSM6_REG_OUTX_L_XL         0x28
#define LSM6_REG_OUTX_H_XL         0x29
#define LSM6_REG_OUTY_L_XL         0x2A
#define LSM6_REG_OUTY_H_XL         0x2B
#define LSM6_REG_OUTZ_L_XL         0x2C
#define LSM6_REG_OUTZ_H_XL         0x2D


#define LSM6_REG_FIFO_STATUS1      0x3A
#define LSM6_REG_FIFO_STATUS2      0x3B
#define LSM6_REG_FIFO_STATUS3      0x3C
#define LSM6_REG_FIFO_STATUS4      0x3D
#define LSM6_REG_FIFO_DATA_OUTL    0x3E
#define LSM6_REG_FIFO_DATA_OUTX    0x3F
#define LSM6_REG_TIMESTAMP0_REG    0x40
#define LSM6_REG_TIMESTAMP1_REG    0x41
#define LSM6_REG_TIMESTAMP2_REG    0x42

#define LSM6_REG_STEP_TIMESTAMP_L  0x49
#define LSM6_REG_STEP_TIMESTAMP_H  0x4A
#define LSM6_REG_STEP_COUNTER_L    0x4B
#define LSM6_REG_STEP_COUNTER_H    0x4C

#define LSM6_REG_FUNC_SRC          0x53

#define LSM6_REG_TAP_CFG           0x58
#define LSM6_REG_TAP_THS_6D        0x59
#define LSM6_REG_INT_DUR2          0x5A
#define LSM6_REG_WAKE_UP_THS       0x5B
#define LSM6_REG_WAKE_UP_DUR       0x5C
#define LSM6_REG_FREE_FALL         0x5D
#define LSM6_REG_MD1_CFG           0x5E
#define LSM6_REG_MD2_CFG           0x5F


/**
 * Selectable gyro range
 */
enum Lsm6GyroRanges {
  LSM6_FS_G_245  = 0x00,
  LSM6_FS_G_500  = 0x01,
  LSM6_FS_G_1000 = 0x02,
  LSM6_FS_G_2000 = 0x03
};

/**
 * Selectable gyro ODR
 */
enum Lsm6GyroODR {
  LSM6_ODR_G_PWR_DWN  = 0x00,
  LSM6_ODR_G_13HZ     = 0x01,
  LSM6_ODR_G_26HZ     = 0x02,
  LSM6_ODR_G_52HZ     = 0x03,
  LSM6_ODR_G_104HZ    = 0x04,
  LSM6_ODR_G_208HZ    = 0x05,
  LSM6_ODR_G_416HZ    = 0x06,
  LSM6_ODR_G_833HZ    = 0x07,
  LSM6_ODR_G_1_6KHZ   = 0x08
};


/**
 * Selectable accel range
 */
enum Lsm6AccelRanges {
  LSM6_FS_XL_2G  = 0x00,
  LSM6_FS_XL_4G  = 0x02,
  LSM6_FS_XL_8G  = 0x03,
  LSM6_FS_XL_16G = 0x01
};

/**
 * Selectable accel ODR
 */
enum Lsm6AccelODR {
  LSM6_ODR_XL_PWR_DWN  = 0x00,
  LSM6_ODR_XL_13HZ     = 0x01,
  LSM6_ODR_XL_26HZ     = 0x02,
  LSM6_ODR_XL_52HZ     = 0x03,
  LSM6_ODR_XL_104HZ    = 0x04,
  LSM6_ODR_XL_208HZ    = 0x05,
  LSM6_ODR_XL_416HZ    = 0x06,
  LSM6_ODR_XL_833HZ    = 0x07,
  LSM6_ODR_XL_1_6KHZ   = 0x08,
  LSM6_ODR_XL_3_3KHZ   = 0x09,
  LSM6_ODR_XL_6_6KHZ   = 0x0A
};

/**
 * Anti-aliasing filter bandwith
 */
enum Lsm6AccelBw {
  LSM6_BW_XL_400HZ = 0x00,
  LSM6_BW_XL_200HZ = 0x01,
  LSM6_BW_XL_100HZ = 0x02,
  LSM6_BW_XL_50HZ  = 0x03
};


/** LSM6 device identifier in LSM6_REG_WHO_AM_I */
#define LSM6_WHO_I_AM              0x69

#endif // LSM6_REGS_H
