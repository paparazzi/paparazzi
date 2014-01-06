/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/lis302dl_regs.h
 *
 * ST LIS302DL 3-axis accelerometer register definitions.
 */

#ifndef LIS302DL_REGS_H
#define LIS302DL_REGS_H

/* Registers */
#define LIS302DL_REG_WHO_AM_I   0x0F
#define LIS302DL_REG_CTRL_REG1  0x20
#define LIS302DL_REG_CTRL_REG2  0x21
#define LIS302DL_REG_CTRL_REG3  0x22
#define LIS302DL_REG_STATUS     0x27
#define LIS302DL_REG_OUTX       0x29
#define LIS302DL_REG_OUTY       0x2B
#define LIS302DL_REG_OUTZ       0x2D

/** LIS302DL device identifier contained in LIS302DL_REG_WHO_AM_I */
#define LIS302DL_WHO_AM_I       0x3B

enum Lis302dlRates {
  LIS302DL_RATE_100HZ = 0,
  LIS302DL_RATE_400HZ = 1
};

enum Lis302dlRanges {
  LIS302DL_RANGE_2G = 0,
  LIS302DL_RANGE_8G = 1
};


#endif /* LIS302DL_REGS_H */
