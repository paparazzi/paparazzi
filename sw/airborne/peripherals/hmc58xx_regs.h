/*
 * Copyright (C) 2011 Gautier Hattenberger
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
 * @file peripherals/hmc58xx_regs.h
 * Register defs for Honeywell HMC5843 and HMC5883 magnetometers.
 */

#ifndef HMC58XX_REGS_H
#define HMC58XX_REGS_H

/* default I2C address */
#define HMC58XX_ADDR 0x3C

/* Registers */
#define HMC58XX_REG_CFGA   0x00
#define HMC58XX_REG_CFGB   0x01
#define HMC58XX_REG_MODE   0x02
#define HMC58XX_REG_DATXM  0x03
#define HMC58XX_REG_DATXL  0x04

/* Warning!
 * The HMC5843 and HMC5883 differ here.
 * - HMC5843 order: Y,Z
 * - HMC5883 order: Z,Y
 * So we make defines for each version explicitly.
 */
#define HMC5843_REG_DATYM  0x05
#define HMC5843_REG_DATYL  0x06
#define HMC5843_REG_DATZM  0x07
#define HMC5843_REG_DATZL  0x08

#define HMC5883_REG_DATZM  0x05
#define HMC5883_REG_DATZL  0x06
#define HMC5883_REG_DATYM  0x07
#define HMC5883_REG_DATYL  0x08


#define HMC58XX_REG_STATUS 0x09
#define HMC58XX_REG_IDA    0x0A
#define HMC58XX_REG_IDB    0x0B
#define HMC58XX_REG_IDC    0x0C

#endif // HMC58XX_REGS_H
