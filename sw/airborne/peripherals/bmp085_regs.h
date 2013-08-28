/*
 * Copyright (C) 2010 Martin Mueller
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

/** @file peripherals/bmp085_regs.h
 *  Bosch BMP085 register definitions.
 */

#ifndef BMP085_REGS_H
#define BMP085_REGS_H

#define BMP085_EEPROM_AC1   0xAA
#define BMP085_EEPROM_AC2   0xAC
#define BMP085_EEPROM_AC3   0xAE
#define BMP085_EEPROM_AC4   0xB0
#define BMP085_EEPROM_AC5   0xB2
#define BMP085_EEPROM_AC6   0xB4
#define BMP085_EEPROM_B1    0xB6
#define BMP085_EEPROM_B2    0xB8
#define BMP085_EEPROM_MB    0xBA
#define BMP085_EEPROM_MC    0xBC
#define BMP085_EEPROM_MD    0xBE

#define BMP085_CTRL_REG     0xF4

#define BMP085_START_TEMP   0x2E
#define BMP085_START_P0     0x34
#define BMP085_START_P1     0x74
#define BMP085_START_P2     0xB4
#define BMP085_START_P3     0xF4

#define BMP085_DAT_MSB      0xF6
#define BMP085_DAT_LSB      0xF7
#define BMP085_DAT_XLSB     0xF8

/// Over sample setting (0-3)
#define BMP085_OSS 3

#define BMP085_SLAVE_ADDR 0xEE

#endif
