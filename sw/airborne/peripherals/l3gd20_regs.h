/*
 * Copyright (C)2014 Federico Ruiz Ugalde
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
 * @file peripherals/l3gd20_regs.h
 *
 * ST L3GD20 3-axis gyroscope register definitions.
 */

#ifndef L3GD20_REGS_H
#define L3GD20_REGS_H

/* Registers */
#define L3GD20_REG_WHO_AM_I   0x0F
#define L3GD20_REG_CTRL_REG1  0x20
#define L3GD20_REG_CTRL_REG2  0x21
#define L3GD20_REG_CTRL_REG3  0x22
#define L3GD20_REG_CTRL_REG4  0x23
#define L3GD20_REG_STATUS_REG 0x27
#define L3GD20_REG_OUT_X_L    0x28
#define L3GD20_REG_OUT_X_H    0x29
#define L3GD20_REG_OUT_Y_L    0x2A
#define L3GD20_REG_OUT_Y_H    0x2B
#define L3GD20_REG_OUT_Z_L    0x2C
#define L3GD20_REG_OUT_Z_H    0x2D

/** L3GD20 device identifier contained in L3GD20_REG_WHO_AM_I */
#define L3GD20_WHO_AM_I       0xD4

#define  L3GD20_DR_MASK 0xC0
#define  L3GD20_BW_MASK 0x30


#define L3GD20_PD (1 << 3)
#define L3GD20_Xen (1 << 0)
#define L3GD20_Yen (1 << 1)
#define L3GD20_Zen (1 << 2)

#define L3GD20_FS_MASK 0x30
#define L3GD20_BDU (1 << 7)

enum L3gd20DRBW {
  L3GD20_DRBW_95Hz_12_5BW,
  L3GD20_DRBW_95Hz_25BW,
  L3GD20_DRBW_95Hz_25BW2,
  L3GD20_DRBW_95Hz_25BW3,
  L3GD20_DRBW_190Hz_12_5BW,
  L3GD20_DRBW_190Hz_25BW,
  L3GD20_DRBW_190Hz_50BW,
  L3GD20_DRBW_190Hz_70BW,
  L3GD20_DRBW_380Hz_20BW,
  L3GD20_DRBW_380Hz_25BW,
  L3GD20_DRBW_380Hz_50BW,
  L3GD20_DRBW_380Hz_100BW,
  L3GD20_DRBW_760Hz_30BW,
  L3GD20_DRBW_760Hz_35BW,
  L3GD20_DRBW_760Hz_50BW,
  L3GD20_DRBW_760Hz_100BW
};

enum L3gd20FullScale {
  L3GD20_FS_250dps = 0,
  L3GD20_FS_500dps = 1,
  L3GD20_FS_2000dps = 2,
  L3GD20_FS_2000dps2 = 3,
};


#endif /* L3GD20_REGS_H */
