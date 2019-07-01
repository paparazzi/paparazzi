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
 * @file peripherals/lp25h_regs.h
 * Register defs for ST LPS25H barometer.
 *
 * Has an I2C and SPI interface.
 */

#ifndef LPS25H_REGS_H
#define LPS25H_REGS_H

// I2C Address
#define LPS25H_ADDR              0xBA

/* Registers */
#define LPS25H_CTRL_REG1         0x20

#define LPS25H_REG_OUT_XL        0x28
#define LPS25H_REG_OUT_L         0x29
#define LPS25H_REG_OUT_H         0x30

#endif // LPS25H_REGS_H
