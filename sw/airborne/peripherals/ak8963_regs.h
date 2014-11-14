/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file peripherals/ak8963.h
 *
 * Register and address definitions for AK8963 magnetometer.
 */

#ifndef AK8963_REGS_H
#define AK8963_REGS_H

#define AK8963_ADDR   0x1A

/* Compass device dependent definition */
#define AK8963_CNTL1_POWER_DOWN   0x10
#define AK8963_CNTL1_SNG_MEASURE  0x11
#define AK8963_CNTL1_CM_1         0x12
#define AK8963_CNTL1_CM_2         0x16
#define AK8963_CNTL1_EXT_TRIG     0x14
#define AK8963_CNTL1_SELF_TEST    0x18
#define AK8963_CNTL1_FUSE_ACCESS  0x1F

/* AK8963 register address */
#define AK8963_REG_WIA    0x00
#define AK8963_REG_INFO   0x01
#define AK8963_REG_ST1    0x02
#define AK8963_REG_HXL    0x03
#define AK8963_REG_HXH    0x04
#define AK8963_REG_HYL    0x05
#define AK8963_REG_HYH    0x06
#define AK8963_REG_HZL    0x07
#define AK8963_REG_HZH    0x08
#define AK8963_REG_ST2    0x09
#define AK8963_REG_CNTL1  0x0A
#define AK8963_REG_CNTL2  0x0B
#define AK8963_REG_ASTC   0x0C
#define AK8963_REG_TS1    0x0D
#define AK8963_REG_TS2    0x0E
#define AK8963_REG_I2CDIS 0x0F

/* AK8963 fuse-rom address */
#define AK8963_FUSE_ASAX  0x10
#define AK8963_FUSE_ASAY  0x11
#define AK8963_FUSE_ASAZ  0x12

#endif /* AK8963_REGS_H */
