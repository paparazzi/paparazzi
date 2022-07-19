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
 * @file peripherals/qmc5883_regs.h
 * Register defenitions for the QMC5883 magnetometer.
 */

#ifndef QMC5883_REGS_H
#define QMC5883_REGS_H

/* default I2C address */
// #define QMC5883_ADDR 0x1B
#define QMC5883_ADDR 0x0D

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

/*
16 bits temperature sensor output is in 2’s complement.

Temperature sensor gain is factory-calibrated, 
but its offset has not been compensated,
only relative temperature value is accurate. 

The temperature coefficient is about 100 LSB/°C
*/

//#define QMC5883_REG_TEMPM  0x07  /* Not so useful ATM, therefore not implemented */
//#define QMC5883_REG_TEMPL  0x08  /* Not so useful ATM, therefore not implemented */

/* Registers Config */
#define QMC5883_REG_CONTROL_1    0x09 
#define QMC5883_REG_CONTROL_2    0x0A
#define QMC5883_REG_RESET_PERIOD 0x0B

//#define QMC5883_REG_IDC    0x0C  /* reserved */
//#define QMC5883_REG_IDD    0x0D  /* reserved */

#endif // QMC5883_REGS_H
