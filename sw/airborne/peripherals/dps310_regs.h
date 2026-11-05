/*
 * Copyright (C) 2026 OpenUAS
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#ifndef DPS310_REGS_H
#define DPS310_REGS_H

/**
 * @file peripherals/dps310_regs.h
 *
 * Register definitions for Infineon DPS310 pressure sensor.
 */

#include "std.h"

/* Default I2C address */
#define DPS310_I2C_ADDR              0xEE // 0x77 << 1
#define DPS310_I2C_ADDR_ALT          0xEC // 0x76 << 1

#define DPS310_CHIP_ID               0x10

/* Registers */ 
#define DPS310_REG_PSR_B2            0x00
#define DPS310_REG_PSR_B1            0x01
#define DPS310_REG_PSR_B0            0x02
#define DPS310_REG_TMP_B2            0x03
#define DPS310_REG_TMP_B1            0x04
#define DPS310_REG_TMP_B0            0x05
#define DPS310_REG_PRS_CFG           0x06
#define DPS310_REG_TMP_CFG           0x07
#define DPS310_REG_MEAS_CFG          0x08
#define DPS310_REG_CFG_REG           0x09
#define DPS310_REG_INT_STS           0x0A
#define DPS310_REG_FIFO_STS          0x0B
#define DPS310_REG_RESET             0x0C
#define DPS310_REG_ID                0x0D

#define DPS310_REG_COEF              0x10
#define DPS310_REG_COEF_SRCE         0x28

#define DPS310_RESET_CMD             0x09

#define DPS310_MEAS_CFG_COEF_RDY     (1 << 7)
#define DPS310_MEAS_CFG_SENSOR_RDY   (1 << 6)
#define DPS310_MEAS_CFG_TMP_RDY      (1 << 5)
#define DPS310_MEAS_CFG_PRS_RDY      (1 << 4)

/* Measurement rates and oversampling */
#define DPS310_PRS_CFG_PM_RATE_32HZ  (5 << 4)
#define DPS310_PRS_CFG_PM_RATE_16HZ  (4 << 4)
#define DPS310_PRS_CFG_PM_RATE_8HZ   (3 << 4)
#define DPS310_PRS_CFG_PM_PRC_16     (4 << 0)
#define DPS310_TMP_CFG_TMP_EXT       (1 << 7)
#define DPS310_TMP_CFG_TMP_RATE_32HZ (5 << 4)
#define DPS310_TMP_CFG_TMP_RATE_16HZ (4 << 4)
#define DPS310_TMP_CFG_TMP_RATE_8HZ  (3 << 4)
#define DPS310_TMP_CFG_TMP_PRC_16    (4 << 0)

#define DPS310_CFG_REG_P_SHIFT       (1 << 2)
#define DPS310_CFG_REG_T_SHIFT       (1 << 3)

#define DPS310_MEAS_CTRL_CONT               0x07
#define DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE  0x80

struct dps310_reg_calib_data {
  int16_t c0;     // 12bit
  int16_t c1;     // 12bit

  int32_t c00;    // 20bit
  int32_t c10;    // 20bit

  int16_t c01;    // 16bit
  int16_t c11;    // 16bit
  int16_t c20;    // 16bit
  int16_t c21;    // 16bit
  int16_t c30;    // 16bit

  int16_t c31;    // 12bit
  int16_t c40;    // 12bit
};

#endif /* DPS310_REGS_H */
