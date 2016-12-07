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
 * @file peripherals/mpu9250_regs.h
 *
 * Register and address definitions for MPU-9250.
 */

#ifndef MPU9250_REGS_H
#define MPU9250_REGS_H

/* default I2C address */
#define MPU9250_ADDR                0xD0
#define MPU9250_ADDR_ALT            0xD2

#define MPU9250_MAG_ADDR            0x18

#define MPU9250_SPI_READ            0x80

// Power and Interface
#define MPU9250_REG_AUX_VDDIO       0x01  // Must be set to 0 on MPU6000
#define MPU9250_REG_USER_CTRL       0x6A
#define MPU9250_REG_PWR_MGMT_1      0x6B
#define MPU9250_REG_PWR_MGMT_2      0x6C

// FIFO
#define MPU9250_REG_FIFO_EN         0x23
#define MPU9250_REG_FIFO_COUNT_H    0x72
#define MPU9250_REG_FIFO_COUNT_L    0x73
#define MPU9250_REG_FIFO_R_W        0x74

// Measurement Settings
#define MPU9250_REG_SMPLRT_DIV      0x19
#define MPU9250_REG_CONFIG          0x1A
#define MPU9250_REG_GYRO_CONFIG     0x1B
#define MPU9250_REG_ACCEL_CONFIG    0x1C
#define MPU9250_REG_ACCEL_CONFIG_2  0x1D


// I2C Slave settings
#define MPU9250_REG_I2C_MST_CTRL    0x24
#define MPU9250_REG_I2C_MST_STATUS  0x36
#define MPU9250_REG_I2C_MST_DELAY   0x67
// Slave 0
#define MPU9250_REG_I2C_SLV0_ADDR   0X25  // i2c addr
#define MPU9250_REG_I2C_SLV0_REG    0X26  // slave reg
#define MPU9250_REG_I2C_SLV0_CTRL   0X27  // set-bits
#define MPU9250_REG_I2C_SLV0_DO     0X63  // DO
// Slave 1
#define MPU9250_REG_I2C_SLV1_ADDR   0X28  // i2c addr
#define MPU9250_REG_I2C_SLV1_REG    0X29  // slave reg
#define MPU9250_REG_I2C_SLV1_CTRL   0X2A  // set-bits
#define MPU9250_REG_I2C_SLV1_DO     0X64  // DO
// Slave 2
#define MPU9250_REG_I2C_SLV2_ADDR   0X2B  // i2c addr
#define MPU9250_REG_I2C_SLV2_REG    0X2C  // slave reg
#define MPU9250_REG_I2C_SLV2_CTRL   0X2D  // set-bits
#define MPU9250_REG_I2C_SLV2_DO     0X65  // DO
// Slave 3
#define MPU9250_REG_I2C_SLV3_ADDR   0X2E  // i2c addr
#define MPU9250_REG_I2C_SLV3_REG    0X2F  // slave reg
#define MPU9250_REG_I2C_SLV3_CTRL   0X30  // set-bits
#define MPU9250_REG_I2C_SLV3_DO     0X66  // DO
// Slave 4 - special
#define MPU9250_REG_I2C_SLV4_ADDR   0X31  // i2c addr
#define MPU9250_REG_I2C_SLV4_REG    0X32  // slave reg
#define MPU9250_REG_I2C_SLV4_DO     0X33  // DO
#define MPU9250_REG_I2C_SLV4_CTRL   0X34  // set-bits
#define MPU9250_REG_I2C_SLV4_DI     0X35  // DI

// Interrupt
#define MPU9250_REG_INT_PIN_CFG     0x37
#define MPU9250_REG_INT_ENABLE      0x38
#define MPU9250_REG_INT_STATUS      0x3A

// Accelero
#define MPU9250_REG_ACCEL_XOUT_H    0x3B
#define MPU9250_REG_ACCEL_XOUT_L    0x3C
#define MPU9250_REG_ACCEL_YOUT_H    0x3D
#define MPU9250_REG_ACCEL_YOUT_L    0x3E
#define MPU9250_REG_ACCEL_ZOUT_H    0x3F
#define MPU9250_REG_ACCEL_ZOUT_L    0x40

// Temperature
#define MPU9250_REG_TEMP_OUT_H      0x41
#define MPU9250_REG_TEMP_OUT_L      0x42

// Gyro
#define MPU9250_REG_GYRO_XOUT_H     0x43
#define MPU9250_REG_GYRO_XOUT_L     0x44
#define MPU9250_REG_GYRO_YOUT_H     0x45
#define MPU9250_REG_GYRO_YOUT_L     0x46
#define MPU9250_REG_GYRO_ZOUT_H     0x47
#define MPU9250_REG_GYRO_ZOUT_L     0x48

// External Sensor Data
#define MPU9250_EXT_SENS_DATA       0x49
#define MPU9250_EXT_SENS_DATA_SIZE  24


#define MPU9250_REG_WHO_AM_I        0x75
#define MPU9250_WHOAMI_REPLY        0x71

// Bit positions
#define MPU9250_I2C_BYPASS_EN       1

// in MPU9250_REG_USER_CTRL
#define MPU9250_SIG_COND_RESET      0
#define MPU9250_I2C_MST_RESET       1
#define MPU9250_FIFO_RESET          2
#define MPU9250_I2C_IF_DIS          4
#define MPU9250_I2C_MST_EN          5
#define MPU9250_FIFO_EN             6

// in MPU9250_REG_I2C_MST_STATUS
#define MPU9250_I2C_SLV4_DONE       6

/** Digital Low Pass Filter Options
 */
enum Mpu9250DLPFGyro {
  MPU9250_DLPF_GYRO_250HZ = 0x0,  // internal sampling rate 8kHz
  MPU9250_DLPF_GYRO_184HZ = 0x1,  // internal sampling rate 1kHz
  MPU9250_DLPF_GYRO_92HZ  = 0x2,
  MPU9250_DLPF_GYRO_41HZ  = 0x3,
  MPU9250_DLPF_GYRO_20HZ  = 0x4,
  MPU9250_DLPF_GYRO_10HZ  = 0x5,
  MPU9250_DLPF_GYRO_05HZ  = 0x6
};

enum Mpu9250DLPFAccel {
  MPU9250_DLPF_ACCEL_460HZ = 0x0,  // internal sampling rate 8kHz
  MPU9250_DLPF_ACCEL_184HZ = 0x1,  // internal sampling rate 1kHz
  MPU9250_DLPF_ACCEL_92HZ  = 0x2,
  MPU9250_DLPF_ACCEL_41HZ  = 0x3,
  MPU9250_DLPF_ACCEL_20HZ  = 0x4,
  MPU9250_DLPF_ACCEL_10HZ  = 0x5,
  MPU9250_DLPF_ACCEL_05HZ  = 0x6
};

/**
 * Selectable gyro range
 */
enum Mpu9250GyroRanges {
  MPU9250_GYRO_RANGE_250  = 0x00,
  MPU9250_GYRO_RANGE_500  = 0x01,
  MPU9250_GYRO_RANGE_1000 = 0x02,
  MPU9250_GYRO_RANGE_2000 = 0x03
};

/**
 * Selectable accel range
 */
enum Mpu9250AccelRanges {
  MPU9250_ACCEL_RANGE_2G  = 0x00,
  MPU9250_ACCEL_RANGE_4G  = 0x01,
  MPU9250_ACCEL_RANGE_8G  = 0x02,
  MPU9250_ACCEL_RANGE_16G = 0x03
};

/**
 * I2C Master clock
 */
enum Mpu9250MstClk {
  MPU9250_MST_CLK_500KHZ = 0x9,
  MPU9250_MST_CLK_471KHZ = 0xA,
  MPU9250_MST_CLK_444KHZ = 0xB,
  MPU9250_MST_CLK_421KHZ = 0xC,
  MPU9250_MST_CLK_400KHZ = 0xD,
  MPU9250_MST_CLK_381KHZ = 0xE,
  MPU9250_MST_CLK_364KHZ = 0xF,
  MPU9250_MST_CLK_348KHZ = 0x0,
  MPU9250_MST_CLK_333KHZ = 0x1,
  MPU9250_MST_CLK_320KHZ = 0x2,
  MPU9250_MST_CLK_308KHZ = 0x3,
  MPU9250_MST_CLK_296KHZ = 0x4,
  MPU9250_MST_CLK_286KHZ = 0x5,
  MPU9250_MST_CLK_276KHZ = 0x6,
  MPU9250_MST_CLK_267KHZ = 0x7,
  MPU9250_MST_CLK_258KHZ = 0x8
};

#endif /* MPU9250_REGS_H */
