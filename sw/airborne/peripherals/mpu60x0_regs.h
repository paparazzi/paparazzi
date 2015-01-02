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
 * @file peripherals/mpu60x0_regs.h
 *
 * Register and address definitions for MPU-6000 and MPU-6050.
 */

#ifndef MPU60X0_REGS_H
#define MPU60X0_REGS_H

/* default I2C address */
#define MPU60X0_ADDR                0xD0
#define MPU60X0_ADDR_ALT            0xD2

#define MPU60X0_SPI_READ            0x80

// Power and Interface
#define MPU60X0_REG_AUX_VDDIO       0x01  // Must be set to 0 on MPU6000
#define MPU60X0_REG_USER_CTRL       0x6A
#define MPU60X0_REG_PWR_MGMT_1      0x6B
#define MPU60X0_REG_PWR_MGMT_2      0x6C

// FIFO
#define MPU60X0_REG_FIFO_EN         0x23
#define MPU60X0_REG_FIFO_COUNT_H    0x72
#define MPU60X0_REG_FIFO_COUNT_L  0x73
#define MPU60X0_REG_FIFO_R_W    0x74

// Measurement Settings
#define MPU60X0_REG_SMPLRT_DIV      0x19
#define MPU60X0_REG_CONFIG          0x1A
#define MPU60X0_REG_GYRO_CONFIG     0x1B
#define MPU60X0_REG_ACCEL_CONFIG    0x1C

// I2C Slave settings
#define MPU60X0_REG_I2C_MST_CTRL    0x24
#define MPU60X0_REG_I2C_MST_STATUS  0x36
#define MPU60X0_REG_I2C_MST_DELAY   0x67
// Slave 0
#define MPU60X0_REG_I2C_SLV0_ADDR 0X25  // i2c addr
#define MPU60X0_REG_I2C_SLV0_REG  0X26  // slave reg
#define MPU60X0_REG_I2C_SLV0_CTRL 0X27  // set-bits
#define MPU60X0_REG_I2C_SLV0_DO   0X63  // DO
// Slave 1
#define MPU60X0_REG_I2C_SLV1_ADDR 0X28  // i2c addr
#define MPU60X0_REG_I2C_SLV1_REG  0X29  // slave reg
#define MPU60X0_REG_I2C_SLV1_CTRL 0X2A  // set-bits
#define MPU60X0_REG_I2C_SLV1_DO   0X64  // DO
// Slave 2
#define MPU60X0_REG_I2C_SLV2_ADDR 0X2B  // i2c addr
#define MPU60X0_REG_I2C_SLV2_REG  0X2C  // slave reg
#define MPU60X0_REG_I2C_SLV2_CTRL 0X2D  // set-bits
#define MPU60X0_REG_I2C_SLV2_DO   0X65  // DO
// Slave 3
#define MPU60X0_REG_I2C_SLV3_ADDR 0X2E  // i2c addr
#define MPU60X0_REG_I2C_SLV3_REG  0X2F  // slave reg
#define MPU60X0_REG_I2C_SLV3_CTRL 0X30  // set-bits
#define MPU60X0_REG_I2C_SLV3_DO   0X66  // DO
// Slave 4 - special
#define MPU60X0_REG_I2C_SLV4_ADDR 0X31  // i2c addr
#define MPU60X0_REG_I2C_SLV4_REG  0X32  // slave reg
#define MPU60X0_REG_I2C_SLV4_DO   0X33  // DO
#define MPU60X0_REG_I2C_SLV4_CTRL 0X34  // set-bits
#define MPU60X0_REG_I2C_SLV4_DI   0X35  // DI

// Interrupt
#define MPU60X0_REG_INT_PIN_CFG     0x37
#define MPU60X0_REG_INT_ENABLE      0x38
#define MPU60X0_REG_INT_STATUS      0x3A

// Accelero
#define MPU60X0_REG_ACCEL_XOUT_H    0x3B
#define MPU60X0_REG_ACCEL_XOUT_L    0x3C
#define MPU60X0_REG_ACCEL_YOUT_H    0x3D
#define MPU60X0_REG_ACCEL_YOUT_L    0x3E
#define MPU60X0_REG_ACCEL_ZOUT_H    0x3F
#define MPU60X0_REG_ACCEL_ZOUT_L    0x40

// Temperature
#define MPU60X0_REG_TEMP_OUT_H      0x41
#define MPU60X0_REG_TEMP_OUT_L      0x42

// Gyro
#define MPU60X0_REG_GYRO_XOUT_H     0x43
#define MPU60X0_REG_GYRO_XOUT_L     0x44
#define MPU60X0_REG_GYRO_YOUT_H     0x45
#define MPU60X0_REG_GYRO_YOUT_L     0x46
#define MPU60X0_REG_GYRO_ZOUT_H     0x47
#define MPU60X0_REG_GYRO_ZOUT_L     0x48

// External Sensor Data
#define MPU60X0_EXT_SENS_DATA       0x49
#define MPU60X0_EXT_SENS_DATA_SIZE  24


#define MPU60X0_REG_WHO_AM_I        0x75
#define MPU60X0_WHOAMI_REPLY      0x68

// Bit positions
#define MPU60X0_I2C_BYPASS_EN       1

// in MPU60X0_REG_USER_CTRL
#define MPU60X0_SIG_COND_RESET      0
#define MPU60X0_I2C_MST_RESET       1
#define MPU60X0_FIFO_RESET          2
#define MPU60X0_I2C_IF_DIS          4
#define MPU60X0_I2C_MST_EN        5
#define MPU60X0_FIFO_EN             6

// in MPU60X0_REG_I2C_MST_STATUS
#define MPU60X0_I2C_SLV4_DONE       6

/** Digital Low Pass Filter Options
 *  DLFP is affecting both gyro and accels,
 *  with slightly different bandwidth
 */
enum Mpu60x0DLPF {
  MPU60X0_DLPF_256HZ = 0x0,  // internal sampling rate 8kHz
  MPU60X0_DLPF_188HZ = 0x1,  // internal sampling rate 1kHz
  MPU60X0_DLPF_98HZ  = 0x2,
  MPU60X0_DLPF_42HZ  = 0x3,
  MPU60X0_DLPF_20HZ  = 0x4,
  MPU60X0_DLPF_10HZ  = 0x5,
  MPU60X0_DLPF_05HZ  = 0x6
};

/**
 * Selectable gyro range
 */
enum Mpu60x0GyroRanges {
  MPU60X0_GYRO_RANGE_250  = 0x00,
  MPU60X0_GYRO_RANGE_500  = 0x01,
  MPU60X0_GYRO_RANGE_1000 = 0x02,
  MPU60X0_GYRO_RANGE_2000 = 0x03
};

/**
 * Selectable accel range
 */
enum Mpu60x0AccelRanges {
  MPU60X0_ACCEL_RANGE_2G  = 0x00,
  MPU60X0_ACCEL_RANGE_4G  = 0x01,
  MPU60X0_ACCEL_RANGE_8G  = 0x02,
  MPU60X0_ACCEL_RANGE_16G = 0x03
};

/**
 * I2C Master clock
 */
enum Mpu60x0MstClk {
  MPU60X0_MST_CLK_500KHZ = 0x9,
  MPU60X0_MST_CLK_471KHZ = 0xA,
  MPU60X0_MST_CLK_444KHZ = 0xB,
  MPU60X0_MST_CLK_421KHZ = 0xC,
  MPU60X0_MST_CLK_400KHZ = 0xD,
  MPU60X0_MST_CLK_381KHZ = 0xE,
  MPU60X0_MST_CLK_364KHZ = 0xF,
  MPU60X0_MST_CLK_348KHZ = 0x0,
  MPU60X0_MST_CLK_333KHZ = 0x1,
  MPU60X0_MST_CLK_320KHZ = 0x2,
  MPU60X0_MST_CLK_308KHZ = 0x3,
  MPU60X0_MST_CLK_296KHZ = 0x4,
  MPU60X0_MST_CLK_286KHZ = 0x5,
  MPU60X0_MST_CLK_276KHZ = 0x6,
  MPU60X0_MST_CLK_267KHZ = 0x7,
  MPU60X0_MST_CLK_254KHZ = 0x8
};

#endif /* MPU60X0_REGS_H */
