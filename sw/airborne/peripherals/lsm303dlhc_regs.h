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
#define LSM303DLHC_REG_CTRL_REG1_A 0x20
#define LSM303DLHC_REG_CTRL_REG2_A 0x21
#define LSM303DLHC_REG_CTRL_REG3_A 0x22
#define LSM303DLHC_REG_CTRL_REG4_A 0x23
#define LSM303DLHC_REG_CTRL_REG5_A 0x24
#define LSM303DLHC_REG_CTRL_REG6_A 0x25
#define LSM303DLHC_REG_REF_DATA_CAP_A 0x26
#define LSM303DLHC_REG_STATUS_REG_A 0x27
#define LSM303DLHC_REG_OUT_X_L_A 0x28
#define LSM303DLHC_REG_OUT_X_H_A 0x29
#define LSM303DLHC_REG_OUT_Y_L_A 0x2A
#define LSM303DLHC_REG_OUT_Y_H_A 0x2B
#define LSM303DLHC_REG_OUT_Z_L_A 0x2C
#define LSM303DLHC_REG_OUT_Z_H_A 0x2D
#define LSM303DLHC_REG_FIFO_CTRL_REG_A 0x2E
#define LSM303DLHC_REG_FIFO_SRC_REG_A 0x2F
#define LSM303DLHC_REG_INT1_CFG_A 0x30
#define LSM303DLHC_REG_INT1_SRC_A 0x31
#define LSM303DLHC_REG_INT1_THS_A 0x32
#define LSM303DLHC_REG_INT1_DURATION_A 0x33
#define LSM303DLHC_REG_INT2_CFG_A 0x34
#define LSM303DLHC_REG_INT2_SRC_A 0x35
#define LSM303DLHC_REG_INT2_THS_A 0x36
#define LSM303DLHC_REG_INT2_DURATION_A 0x37
#define LSM303DLHC_REG_CLICK_CFG_A 0x38
#define LSM303DLHC_REG_CLICK_SRC_A 0x39
#define LSM303DLHC_REG_CLICK_THS_A 0x3A
#define LSM303DLHC_REG_TIME_LIMIT_A 0x3B
#define LSM303DLHC_REG_TIME_LATENCY_A 0x3C
#define LSM303DLHC_REG_TIME_WINDOW_A 0x3D
#define LSM303DLHC_REG_CRA_REG_M 0x00
#define LSM303DLHC_REG_CRB_REG_M 0x01
#define LSM303DLHC_REG_MR_REG_M 0x02
#define LSM303DLHC_REG_OUT_X_H_M 0x03
#define LSM303DLHC_REG_OUT_X_L_M 0x04
#define LSM303DLHC_REG_OUT_Z_H_M 0x05
#define LSM303DLHC_REG_OUT_Z_L_M 0x06
#define LSM303DLHC_REG_OUT_Y_H_M 0x07
#define LSM303DLHC_REG_OUT_Y_L_M 0x08
#define LSM303DLHC_REG_SR_REG_M 0x09
#define LSM303DLHC_REG_IRA_REG_M 0x0A
#define LSM303DLHC_REG_IRB_REG_M 0x0B
#define LSM303DLHC_REG_IRC_REG_M 0x0C
#define LSM303DLHC_REG_TEMP_OUT_H_M 0x31
#define LSM303DLHC_REG_TEMP_OUT_L_M 0x32

/* Bit definitions */
#define LSM303DLHC_ODR_MASK 0xF0
#define LSM303DLHC_LPen (1 << 3)
#define LSM303DLHC_Xen (1 << 0)
#define LSM303DLHC_Yen (1 << 1)
#define LSM303DLHC_Zen (1 << 2)

#define LSM303DLHC_FS_MASK 0x30
#define LSM303DLHC_HR (1 << 3)
#define LSM303DLHC_BDU (1 << 7)

#define LSM303DLHC_I1_DRDY1 (1 << 4)

#define LSM303DLHC_DO0_MASK 0x1C
#define LSM303DLHC_GN_MASK 0xE0
#define LSM303DLHC_MD_MASK 0x03



#endif // LSM303DLHC_REGS_H
