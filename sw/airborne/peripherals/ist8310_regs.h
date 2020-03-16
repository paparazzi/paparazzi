/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file peripherals/ist8310_regs.h
 *
 * Register and address definitions for the IST8310 magnetometer.
 */

#ifndef IST8310_REGS_H
#define IST8310_REGS_H

#define IST8310_ADDR   0x1C

/* IST8310 register address */
#define IST8310_REG_WHO_AM_I    0x00
#define IST8310_REG_STAT1       0x02
#define IST8310_REG_DATA_XL     0x03
#define IST8310_REG_DATA_XH     0x04
#define IST8310_REG_DATA_YL     0x05
#define IST8310_REG_DATA_YM     0x06
#define IST8310_REG_DATA_ZL     0x07
#define IST8310_REG_DATA_ZM     0x08
#define IST8310_REG_STAT2       0x09
#define IST8310_REG_CNTL1       0x0A
#define IST8310_REG_CNTL2       0x0B
#define IST8310_REG_STR         0x0C
#define IST8310_REG_TEMPL       0x1C
#define IST8310_REG_TEMPH       0x1D
#define IST8310_REG_CNTL3       0x41
#define IST8310_REG_CNTL4       0x42
#define IST8310_REG_Y11L        0x9C
#define IST8310_REG_Y11H        0x9D
#define IST8310_REG_Y12L        0x9E
#define IST8310_REG_Y12H        0x9F
#define IST8310_REG_Y13L        0xA0
#define IST8310_REG_Y13H        0xA1
#define IST8310_REG_Y21L        0xA2
#define IST8310_REG_Y21H        0xA3
#define IST8310_REG_Y22L        0xA4
#define IST8310_REG_Y22H        0xA5
#define IST8310_REG_Y23L        0xA6
#define IST8310_REG_Y23H        0xA7
#define IST8310_REG_Y31L        0xA8
#define IST8310_REG_Y31H        0xA9
#define IST8310_REG_Y32L        0xAA
#define IST8310_REG_Y32H        0xAB
#define IST8310_REG_Y33L        0xAC
#define IST8310_REG_Y33H        0xAD

/* STAT1 register values */
#define IST8310_STAT1_DRDY          (1 << 0)
#define IST8310_STAT1_DOR           (1 << 1)

/* STAT2 register values */
#define IST8310_STAT2_INT           (1 << 3)

/* CNTL1 register options */
#define IST8310_CNTL1_ODR_SINGLE    0x01
#define IST8310_CNTL1_ODR_10HZ      0x03
#define IST8310_CNTL1_ODR_20HZ      0x05
#define IST8310_CNTL1_ODR_50HZ      0x07
#define IST8310_CNTL1_ODR_100HZ     0x06

/* CNTL2 register settings */
#define IST8310_CNTL2_SRST          (1 << 0)
#define IST8310_CNTL2_DRP           (1 << 2)
#define IST8310_CNTL2_DREN          (1 << 3)

/* STR register settings (Self test) */
#define IST8310_STR_ENABLE          (1 << 6)

/* CNTL3 register options (Amount of samples to average) */
#define IST8310_CNTL3_SAMPAVG_16    0x24
#define IST8310_CNTL3_SAMPAVG_8     0x1B
#define IST8310_CNTL3_SAMPAVG_4     0x12
#define IST8310_CNTL3_SAMPAVG_2     0x09
#define IST8310_CNTL3_SAMPAVG_0     0x00

/* CNTL4 register options (Set reset pulse duration) */
#define IST8310_CNTL4_SRPD          0xC0

#endif /* IST8310_REGS_H */
