/*
 * Copyright (C) 2020 Paparazzi team
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
 * @file modules/sensors/mag_qmc5883_regs.h
 * Register defenitions for the QMC5883 magnetometer.
 */

#ifndef QMC5883_REGS_H
#define QMC5883_REGS_H

/* read and write I2C addresses based on default 0x0D */
#define QMC5883_READ_ADDR  0x1B
#define QMC5883_WRITE_ADDR 0x1A

/* Registers Axis X,Y,Z */
#define QMC5883_REG_DATXL  0x00
#define QMC5883_REG_DATXM  0x01
#define QMC5883_REG_DATYL  0x02
#define QMC5883_REG_DATYM  0x03
#define QMC5883_REG_DATZL  0x04
#define QMC5883_REG_DATZM  0x05

/* Register Status */
#define QMC5883_REG_STATUS 0x06

/*  Registers Temperature */
// #define QMC5883_REG_TEMPM  0x07  /* Not so useful ATM, therefore not implemented */
// #define QMC5883_REG_TEMPL  0x08  /* Not so useful ATM, therefore not implemented */

/* Registers Config */
#define QMC5883_REG_CONTROL_1    0x09  /* settings for MODE */
#define QMC5883_REG_CONTROL_2    0x0A  /* settings for INT_ENB */
#define QMC5883_REG_RESET_PERIOD 0x0B

//#define QMC5883_REG_IDC    0x0C  /* reserved */
//#define QMC5883_REG_IDD    0x0D  /* reserved */

/* Settings for CONTROL_1 */

// MODE CONTROL (MODE)
#define QMC5883_MODE_STBY 0x00
#define QMC5883_MODE_CONT 0x01

// OUTPUT DATA RATE IN HZ (ODR)
#define QMC5883_ODR_10  0x00
#define QMC5883_ODR_50  0x04
#define QMC5883_ODR_100 0x08
#define QMC5883_ODR_200 0x0C

// FULL SCALE (RNG)
#define QMC5883_RNG_2G 0x00
#define QMC5883_RNG_8G 0x10

// OVER SAMPLE RATIO (OSR)
#define QMC5883_OSR_512 0x00
#define QMC5883_OSR_256 0x40
#define QMC5883_OSR_128 0x80
#define QMC5883_OSR_64  0xC0

#endif // QMC5883_REGS_H
