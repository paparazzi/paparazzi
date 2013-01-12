/*
 * Copyright (C) 2010-2013 The Paparazzi Team
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
 * @file peripherals/adxl345_regs.h
 *
 * Register and address definitions for ADXL345 accelerometer.
 */

#ifndef ADXL345_REGS_H
#define ADXL345_REGS_H

/** default I2C address */
#define ADXL345_ADDR            0xA6
#define ADXL345_ADDR_ALT        0x3A

/* Registers */
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_INT_ENABLE  0x2E
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATA_X0     0x32
#define ADXL345_REG_DATA_X1     0x33
#define ADXL345_REG_DATA_Y0     0x34
#define ADXL345_REG_DATA_Y1     0x35
#define ADXL345_REG_DATA_Z0     0x36
#define ADXL345_REG_DATA_Z1     0x37

/**
 * Selectable data rates in ADXL345_REG_BW_RATE
 * bandwith is always half of data rate
 */
enum Adxl345Rates {
  ADXL345_RATE_25HZ   = 0x08,
  ADXL345_RATE_50HZ   = 0x09,
  ADXL345_RATE_100HZ  = 0x0A,
  ADXL345_RATE_200HZ  = 0x0B,
  ADXL345_RATE_400HZ  = 0x0C,
  ADXL345_RATE_800HZ  = 0x0D,
  ADXL345_RATE_1600HZ = 0x0E,
  ADXL345_RATE_3200HZ = 0x0F
};

/**
 * Selectable range in ADXL345_REG_DATA_FORMAT
 */
enum Adxl345Ranges {
  ADXL345_RANGE_2G  = 0x00,
  ADXL345_RANGE_4G  = 0x01,
  ADXL345_RANGE_8G  = 0x02,
  ADXL345_RANGE_16G = 0x03
};

/* data format bits */
#define ADXL345_INT_INVERT  0x20
#define ADXL345_FULL_RES    0x08
#define ADXL345_JUSTIFY_MSB 0x04


#endif /* ADXL345_REGS_H */
