/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file peripherals/invensense3.h
 *
 * Driver for the Invensense V3 IMUs
 * ICM40605, ICM40609, ICM42605, IIM42652 and ICM42688
 */

#ifndef INVENSENSE3_H
#define INVENSENSE3_H

#include "std.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/i2c.h"

/* This sensor has an option to request little-endian data */
// Hold 22 measurements + 3 for the register address and length
#define INVENSENSE3_FIFO_BUFFER_LEN 22
#define INVENSENSE3_BUFFER_SIZE INVENSENSE3_FIFO_BUFFER_LEN * 20 + 3 // 20 bytes is the maximum sample size

/* Invensense v3 SPI peripheral */
struct invensense3_spi_t {
  struct spi_periph *p;                             ///< Peripheral device for communication
  uint8_t slave_idx;                                ///< Slave index used for Slave Select
  struct spi_transaction trans;                     ///< Transaction used during configuration and measurements

  uint8_t tx_buf[2];                       ///< Transmit buffer
  uint8_t rx_buf[INVENSENSE3_BUFFER_SIZE]; ///< Receive buffer
};

/* Invensense v3 I2C peripheral */
struct invensense3_i2c_t {
  struct i2c_periph *p;           ///< Peripheral device for communication
  uint8_t slave_addr;             ///< The I2C slave address on the bus
  struct i2c_transaction trans;   ///< TRansaction used during configuration and measurements
};

/* Possible communication busses for the invense V3 driver */
enum invensense3_bus_t {
  INVENSENSE3_SPI,
  INVENSENSE3_I2C
};

/* Different states the invensense v3 driver can be in */
enum invensense3_status_t {
  INVENSENSE3_IDLE,
  INVENSENSE3_CONFIG,
  INVENSENSE3_RUNNING
};

/* Different parsers of the invensense v3 driver */
enum invensense3_parser_t {
  INVENSENSE3_PARSER_REGISTERS,
  INVENSENSE3_PARSER_FIFO
};

enum invensense3_fifo_packet_t {
  INVENSENSE3_SAMPLE_SIZE_PK1,
  INVENSENSE3_SAMPLE_SIZE_PK2,
  INVENSENSE3_SAMPLE_SIZE_PK3,
  INVENSENSE3_SAMPLE_SIZE_PK4
};

/* Different device types compatible with the invensense v3 driver */
enum invensense3_device_t {
  INVENSENSE3_UNKOWN,
  INVENSENSE3_ICM40605,
  INVENSENSE3_ICM40609,
  INVENSENSE3_ICM42605,
  INVENSENSE3_IIM42652,
  INVENSENSE3_ICM42688
};

/* The gyro output data rate configuration */
enum invensense3_gyro_odr_t {
  INVENSENSE3_GYRO_ODR_32KHZ = 1, ///< Not possible for ICM40605 and ICM42605
  INVENSENSE3_GYRO_ODR_16KHZ,     ///< Not possible for ICM40605 and ICM42605
  INVENSENSE3_GYRO_ODR_8KHZ,
  INVENSENSE3_GYRO_ODR_4KHZ,
  INVENSENSE3_GYRO_ODR_2KHZ,
  INVENSENSE3_GYRO_ODR_1KHZ,
  INVENSENSE3_GYRO_ODR_200HZ,
  INVENSENSE3_GYRO_ODR_100HZ,
  INVENSENSE3_GYRO_ODR_50HZ,
  INVENSENSE3_GYRO_ODR_25HZ,
  INVENSENSE3_GYRO_ODR_12_5HZ,
  INVENSENSE3_GYRO_ODR_6_25HZ,
  INVENSENSE3_GYRO_ODR_3_125HZ,
  INVENSENSE3_GYRO_ODR_1_5625HZ,
  INVENSENSE3_GYRO_ODR_500HZ
};

/* The gyro range in degrees per second(dps) */
enum invensense3_gyro_range_t {
  INVENSENSE3_GYRO_RANGE_2000DPS,
  INVENSENSE3_GYRO_RANGE_1000DPS,
  INVENSENSE3_GYRO_RANGE_500DPS,
  INVENSENSE3_GYRO_RANGE_250DPS,
  INVENSENSE3_GYRO_RANGE_125DPS,
  INVENSENSE3_GYRO_RANGE_62_5DPS,
  INVENSENSE3_GYRO_RANGE_31_25DPS,
  INVENSENSE3_GYRO_RANGE_15_625DPS
};

/* The accelerometer output data rate configuration */
enum invensense3_accel_odr_t {
  INVENSENSE3_ACCEL_ODR_32KHZ = 1,  ///< Not possible for ICM40605 and ICM42605
  INVENSENSE3_ACCEL_ODR_16KHZ,      ///< Not possible for ICM40605 and ICM42605
  INVENSENSE3_ACCEL_ODR_8KHZ,
  INVENSENSE3_ACCEL_ODR_4KHZ,
  INVENSENSE3_ACCEL_ODR_2KHZ,
  INVENSENSE3_ACCEL_ODR_1KHZ,
  INVENSENSE3_ACCEL_ODR_200HZ,
  INVENSENSE3_ACCEL_ODR_100HZ,
  INVENSENSE3_ACCEL_ODR_50HZ,
  INVENSENSE3_ACCEL_ODR_25HZ,
  INVENSENSE3_ACCEL_ODR_12_5HZ,
  INVENSENSE3_ACCEL_ODR_6_25HZ,
  INVENSENSE3_ACCEL_ODR_3_125HZ,
  INVENSENSE3_ACCEL_ODR_1_5625HZ,
  INVENSENSE3_ACCEL_ODR_500HZ
};

/* The accelerometer range in G */
enum invensense3_accel_range_t {
  INVENSENSE3_ACCEL_RANGE_32G,       ///< Only possible for ICM40609
  INVENSENSE3_ACCEL_RANGE_16G,
  INVENSENSE3_ACCEL_RANGE_8G,
  INVENSENSE3_ACCEL_RANGE_4G,
  INVENSENSE3_ACCEL_RANGE_2G
};

/* Main invensense v3 device structure */
struct invensense3_t {
  uint8_t abi_id;                     ///< The ABI id used to broadcast the device measurements
  enum invensense3_status_t status;   ///< Status of the invensense v3 device
  enum invensense3_device_t device;   ///< The device type detected
  enum invensense3_parser_t parser;   ///< Parser of the device

  enum invensense3_bus_t bus;         ///< The communication bus used to connect the device SPI/I2C
  union {
    struct invensense3_spi_t spi;     ///< SPI specific configuration
    struct invensense3_i2c_t i2c;     ///< I2C specific configuration
  };
  uint8_t* rx_buffer;
  uint8_t* tx_buffer;
  uint16_t* rx_length;

  uint8_t register_bank;              ///< The current register bank communicating with
  uint8_t config_idx;                 ///< The current configuration index
  uint32_t timer;                     ///< Used to time operations during configuration (samples left during measuring)

  enum invensense3_gyro_odr_t gyro_odr;         ///< Gyro Output Data Rate configuration
  enum invensense3_gyro_range_t gyro_range;     ///< Gyro range configuration
  uint16_t gyro_aaf;                            ///< Gyro Anti-alias filter 3dB Bandwidth configuration [Hz]
  uint16_t gyro_aaf_regs[4];                    ///< Gyro Anti-alias filter register values
  enum invensense3_accel_odr_t accel_odr;       ///< Accelerometer Output Data Rate configuration
  enum invensense3_accel_range_t accel_range;   ///< Accelerometer range configuration
  uint16_t accel_aaf;                           ///< Accelerometer Anti-alias filter 3dB Bandwidth configuration [Hz]
  uint16_t accel_aaf_regs[4];                   ///< Accelerometer Anti-alias filter register values
  enum invensense3_fifo_packet_t sample_size;   ///< FIFO packet size
  int sample_numbers;                           ///< expected FIFO packet number, assuming reading at PERIODIC_FREQUENCY
};

/* External functions */
void invensense3_init(struct invensense3_t *inv);
void invensense3_periodic(struct invensense3_t *inv);
void invensense3_event(struct invensense3_t *inv);

#endif // INVENSENSE3_H
