/*
 * Copyright (C) 2014 Federico Ruiz Ugalde
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
 * @file peripherals/lsm303dlhc_regs.h
 * Register defs for ST LSM303DLHC 3D accelerometer and magnetometer.
 */

#ifndef LSM303DLHC_REGS_H
#define LSM303DLHC_REGS_H

/* default I2C address */
#define LSM303DLHC_ACC_ADDR 0x32
#define LSM303DLHC_MAG_ADDR 0x3C

/* Registers */

//checked:
#define LSM303DLHC_REG_TEMP_OUT_L_M 0x05
#define LSM303DLHC_REG_TEMP_OUT_H_M 0x06

#define LSM303DLHC_REG_STATUS_REG_M 0x07
#define LSM303DLHC_REG_OUT_X_L_M 0x08
#define LSM303DLHC_REG_OUT_X_H_M 0x09
#define LSM303DLHC_REG_OUT_Y_L_M 0x0A
#define LSM303DLHC_REG_OUT_Y_H_M 0x0B
#define LSM303DLHC_REG_OUT_Z_L_M 0x0C
#define LSM303DLHC_REG_OUT_Z_H_M 0x0D

#define LSM303DLHC_REG_WHO_AM_I 0x0F

#define LSM303DLHC_REG_INT_CTRL_M 0x12
#define LSM303DLHC_REG_INT_SRC_M 0x13
#define LSM303DLHC_REG_INT_THS_L_M 0x14
#define LSM303DLHC_REG_INT_THS_H_M 0x15
#define LSM303DLHC_REG_OFFSET_X_L_M 0x16
#define LSM303DLHC_REG_OFFSET_X_H_M 0x17
#define LSM303DLHC_REG_OFFSET_Y_L_M 0x18
#define LSM303DLHC_REG_OFFSET_Y_H_M 0x19
#define LSM303DLHC_REG_OFFSET_Z_L_M 0x1A
#define LSM303DLHC_REG_OFFSET_Z_H_M 0x1B
#define LSM303DLHC_REG_REFERENCE_X 0x1C
#define LSM303DLHC_REG_REFERENCE_Y 0x1D
#define LSM303DLHC_REG_REFERENCE_Z 0x1E
#define LSM303DLHC_REG_CTRL0 0x1F
#define LSM303DLHC_REG_CTRL1 0x20
#define LSM303DLHC_REG_CTRL2 0x21
#define LSM303DLHC_REG_CTRL3 0x22
#define LSM303DLHC_REG_CTRL4 0x23
#define LSM303DLHC_REG_CTRL5 0x24
#define LSM303DLHC_REG_CTRL6 0x25
#define LSM303DLHC_REG_CTRL7 0x26

#define LSM303DLHC_REG_STATUS_REG_A 0x27
#define LSM303DLHC_REG_OUT_X_L_A 0x28
#define LSM303DLHC_REG_OUT_X_H_A 0x29
#define LSM303DLHC_REG_OUT_Y_L_A 0x2A
#define LSM303DLHC_REG_OUT_Y_H_A 0x2B
#define LSM303DLHC_REG_OUT_Z_L_A 0x2C
#define LSM303DLHC_REG_OUT_Z_H_A 0x2D

#define LSM303DLHC_REG_FIFO_CTRL 0x2E
#define LSM303DLHC_REG_FIFO_SRC 0x2F
#define LSM303DLHC_REG_INT_CFG 0x30
#define LSM303DLHC_REG_INT_SRC1 0x31
#define LSM303DLHC_REG_INT_THS1 0x32
#define LSM303DLHC_REG_INT_DUR1 0x33
#define LSM303DLHC_REG_INT_CFG2 0x34
#define LSM303DLHC_REG_INT_SRC2 0x35
#define LSM303DLHC_REG_INT_THS2 0x36
#define LSM303DLHC_REG_INT_DUR2 0x37
#define LSM303DLHC_REG_CLICK_CFG 0x38
#define LSM303DLHC_REG_CLICK_SRC 0x39
#define LSM303DLHC_REG_CLICK_THS 0x3A
#define LSM303DLHC_REG_TIME_LIMIT 0x3B
#define LSM303DLHC_REG_TIME_LATENCY 0x3C
#define LSM303DLHC_REG_TIME_WINDOW 0x3D
#define LSM303DLHC_ACT_THS 0x3E
#define LSM303DLHC_ACT_DUR 0x3F

#define LSM303DLHC_REG_STATUS_ZYXADA 0x08
#define LSM303DLHC_REG_STATUS_ZYXMDA 0x08

#define LSM303DLHC_WHO_I_AM 0x49

/* Bit definitions */
//CTRL1
#define LSM303DLHC_Xen (0x01 << 0)
#define LSM303DLHC_Yen (0x01 << 1)
#define LSM303DLHC_Zen (0x01 << 2)
#define LSM303DLHC_BDU (0x01 << 3)
#define LSM303DLHC_AODR_MASK (0x0F << 4)

//CTRL2
#define LSM303DLHC_SIM (0x01 << 0)
#define LSM303DLHC_AST (0x01 << 1)
#define LSM303DLHC_FS_MASK (0x07 << 3)
#define LSM303DLHC_ABW_MASK (0x03 << 6)

//CTRL3
#define LSM303DLHC_I1_DRDY_A (0x01 << 2)
//TODO: more CTRL3 regs

//CTRL4
#define LSM303DLHC_I2_DRDY_A (0x01 << 3)
#define LSM303DLHC_I2_DRDY_M (0x01 << 2)
//TODO: more CTRL4 regs

//CTRL5
#define LSM303DLHC_TEMP_EN (0x01 << 7)
#define LSM303DLHC_M_RES (0x07 << 5) // only two modes, so no mask
#define LSM303DLHC_M_ODR_MASK (0x15 << 2)
#define LSM303DLHC_M_LIR_MASK (0x7 << 0)

//CTRL6
#define LSM303DLHC_MFS_MASK (0x07 << 5)

//CTRL7
#define LSM303DLHC_AHPM_MASK (0x07 << 6)
#define LSM303DLHC_AFDS (0x01 << 5)
#define LSM303DLHC_T_ONLY (0x01 << 4)
#define LSM303DLHC_MLP (0x01 << 2)
#define LSM303DLHC_MD_MASK (0x07 << 0)

#endif // LSM303DLHC_REGS_H
