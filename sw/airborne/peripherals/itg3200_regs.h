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
 * @file peripherals/itg3200_regs.h
 * Register defs for ITG3200 and IMU3000 gyros.
 */

#ifndef ITG3200_REGS_H
#define ITG3200_REGS_H

/* default I2C address */
#define ITG3200_ADDR            0xD0
#define ITG3200_ADDR_ALT        0xD2

/* Registers */
#define ITG3200_REG_WHO_AM_I    0X00
#define ITG3200_REG_SMPLRT_DIV  0X15
#define ITG3200_REG_DLPF_FS     0X16
#define ITG3200_REG_INT_CFG     0X17
#define ITG3200_REG_INT_STATUS  0X1A
#define ITG3200_REG_TEMP_OUT_H  0X1B
#define ITG3200_REG_TEMP_OUT_L  0X1C
#define ITG3200_REG_GYRO_XOUT_H 0X1D
#define ITG3200_REG_GYRO_XOUT_L 0X1E
#define ITG3200_REG_GYRO_YOUT_H 0X1F
#define ITG3200_REG_GYRO_YOUT_L 0X20
#define ITG3200_REG_GYRO_ZOUT_H 0X21
#define ITG3200_REG_GYRO_ZOUT_L 0X22
#define ITG3200_REG_PWR_MGM     0X3E

/** Digital Low Pass Filter Options */
enum Itg3200DLPF {
  ITG3200_DLPF_256HZ = 0x0,  // internal sampling rate 8kHz
  ITG3200_DLPF_188HZ = 0x1,  // internal sampling rate 1kHz
  ITG3200_DLPF_98HZ  = 0x2,
  ITG3200_DLPF_42HZ  = 0x3,
  ITG3200_DLPF_20HZ  = 0x4,
  ITG3200_DLPF_10HZ  = 0x5,
  ITG3200_DLPF_05HZ  = 0x6
};

#endif /* ITG3200_REGS_H */
