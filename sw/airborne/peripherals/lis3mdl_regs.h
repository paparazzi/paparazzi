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
 * @file peripherals/lis3mdl_regs.h
 * Register defs for ST LIS3MDL 3D magnetometer.
 *
 * Has an I2C interface.
 */

#ifndef LIS3MDL_REGS_H
#define LIS3MDL_REGS_H

// I2C Address
#define LIS3MDL_ADDR               0x3C

/* Registers */
#define LIS3MDL_CTRL_REG1          0x20
#define LIS3MDL_CTRL_REG2          0x21
#define LIS3MDL_CTRL_REG3          0x22
#define LIS3MDL_CTRL_REG4          0x23

#define LIS3MDL_REG_OUTX_L         0x28
#define LIS3MDL_REG_OUTX_H         0x29
#define LIS3MDL_REG_OUTY_L         0x2A
#define LIS3MDL_REG_OUTY_H         0x2B
#define LIS3MDL_REG_OUTZ_L         0x2C
#define LIS3MDL_REG_OUTZ_H         0x2D

/* Disable/Enable Fast ODR
 * If enable, ODR depends of operating
 * mode value
 */
#define LIS3MDL_FODR_D             0x00
#define LIS3MDL_FODR_E             0x01

/**
 * Full scale configuration
 */
enum Lis3mdFS {
  LIS3MDL_FS_4G  = 0x00,
  LIS3MDL_FS_8G  = 0x01,
  LIS3MDL_FS_12G = 0x02,
  LIS3MDL_FS_16G = 0x03
};

/**
 * Selectable ODR magnetometer
 */
enum Lis3mdlODR {
  LIS3MDL_ODR_0_625HZ  = 0x00,
  LIS3MDL_ODR_1_25HZ   = 0x01,
  LIS3MDL_ODR_2_5HZ    = 0x02,
  LIS3MDL_ODR_5HZ      = 0x03,
  LIS3MDL_ODR_10HZ     = 0x04,
  LIS3MDL_ODR_20HZ     = 0x05,
  LIS3MDL_ODR_40HZ     = 0x06,
  LIS3MDL_ODR_80HZ     = 0x07
};

/**
 * X and Y axes operating mode selection
 */
enum Lis3mdlXyOm {
  LIS3MDL_XY_OM_LP   = 0x00,
  LIS3MDL_XY_OM_MP   = 0x01,
  LIS3MDL_XY_OM_HP   = 0x02,
  LIS3MDL_XY_OM_UHP  = 0x03
};

/**
 * Z axe operating mode selection
 */
enum Lis3mdlZOm {
  LIS3MDL_Z_OM_LP   = 0x00,
  LIS3MDL_Z_OM_MP   = 0x01,
  LIS3MDL_Z_OM_HP   = 0x02,
  LIS3MDL_Z_OM_UHP  = 0x03
};

#endif // LIS3MDL_REGS_H
