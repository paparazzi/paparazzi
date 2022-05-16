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
 * @file peripherals/invensense2.h
 *
 * Driver for the Invensense V2 IMUs
 * ICM20948, ICM20648 and ICM20649
 */

#ifndef INVENSENSE2_H
#define INVENSENSE2_H

#include "std.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/i2c.h"

// Hold 22 measurements and 3 for the register address and length
#define INVENSENSE2_SAMPLE_CNT    22
#define INVENSENSE2_SAMPLE_SIZE   14
#define INVENSENSE2_BUFFER_SIZE   ((INVENSENSE2_SAMPLE_SIZE*INVENSENSE2_SAMPLE_CNT) + 3)

/* Invensense v2 SPI peripheral */
struct invensense2_spi_t {
  struct spi_periph *p;                             ///< Peripheral device for communication
  uint8_t slave_idx;                                ///< Slave index used for Slave Select
  struct spi_transaction trans;                     ///< Transaction used during configuration and measurements

  volatile uint8_t tx_buf[2];                       ///< Transmit buffer
  volatile uint8_t rx_buf[INVENSENSE2_BUFFER_SIZE]; ///< Receive buffer
};

/* Invensense v2 I2C peripheral */
struct invensense2_i2c_t {
  struct i2c_periph *p;           ///< Peripheral device for communication
  uint8_t slave_addr;             ///< The I2C slave address on the bus
  struct i2c_transaction trans;   ///< TRansaction used during configuration and measurements
};

/* Possible communication busses for the invense V2 driver */
enum invensense2_bus_t {
  INVENSENSE2_SPI,
  INVENSENSE2_I2C
};

/* Different states the invensense driver can be in */
enum invensense2_status_t {
  INVENSENSE2_IDLE,
  INVENSENSE2_CONFIG,
  INVENSENSE2_RUNNING
};

/* Different device types compatible with the invensense V2 driver */
enum invensense2_device_t {
  INVENSENSE2_UNKOWN,
  INVENSENSE2_ICM20648,
  INVENSENSE2_ICM20649,
  INVENSENSE2_ICM20948
};

/* The gyro digital low pass filter bandwidth configuration */
enum invensense2_gyro_dlpf_t {
  INVENSENSE2_GYRO_DLPF_OFF,
  INVENSENSE2_GYRO_DLPF_229HZ,
  INVENSENSE2_GYRO_DLPF_188HZ,
  INVENSENSE2_GYRO_DLPF_154HZ,
  INVENSENSE2_GYRO_DLPF_73HZ,
  INVENSENSE2_GYRO_DLPF_36HZ,
  INVENSENSE2_GYRO_DLPF_18HZ,
  INVENSENSE2_GYRO_DLPF_9HZ,
  INVENSENSE2_GYRO_DLPF_377HZ
};

/* The gyro range in degrees per second(dps) */
enum invensense2_gyro_range_t {
  INVENSENSE2_GYRO_RANGE_250DPS,    ///< Not possible for ICM20649
  INVENSENSE2_GYRO_RANGE_500DPS,
  INVENSENSE2_GYRO_RANGE_1000DPS,
  INVENSENSE2_GYRO_RANGE_2000DPS,
  INVENSENSE2_GYRO_RANGE_4000DPS    ///< Only possible for ICM20649
};

/* The accelerometer digital low pass filter bandwidth configuration */
enum invensense2_accel_dlpf_t {
  INVENSENSE2_ACCEL_DLPF_OFF,
  INVENSENSE2_ACCEL_DLPF_265HZ,
  INVENSENSE2_ACCEL_DLPF_136HZ,
  INVENSENSE2_ACCEL_DLPF_69HZ,
  INVENSENSE2_ACCEL_DLPF_34HZ,
  INVENSENSE2_ACCEL_DLPF_17HZ,
  INVENSENSE2_ACCEL_DLPF_8HZ,
  INVENSENSE2_ACCEL_DLPF_499HZ
};

/* The accelerometer range in G */
enum invensense2_accel_range_t {
  INVENSENSE2_ACCEL_RANGE_2G,   ///< Not possible for ICM20649
  INVENSENSE2_ACCEL_RANGE_4G,
  INVENSENSE2_ACCEL_RANGE_8G,
  INVENSENSE2_ACCEL_RANGE_16G,
  INVENSENSE2_ACCEL_RANGE_30G   ///< Only possible for ICM20649
};

/* Main invensense V2 device structure */
struct invensense2_t {
  uint8_t abi_id;                     ///< The ABI id used to broadcast the device measurements
  enum invensense2_status_t status;   ///< Status of the invensense V2 device
  enum invensense2_device_t device;   ///< The device type detected

  enum invensense2_bus_t bus;         ///< The communication bus used to connect the device SPI/I2C
  union {
    struct invensense2_spi_t spi;     ///< SPI specific configuration
    struct invensense2_i2c_t i2c;     ///< I2C specific configuration
  };

  uint8_t register_bank;              ///< The current register bank communicating with
  uint8_t config_idx;                 ///< The current configuration index
  uint32_t timer;                     ///< Used to time operations during configuration (samples left during measuring)

  enum invensense2_gyro_dlpf_t gyro_dlpf;       ///< Gyro DLPF configuration
  enum invensense2_gyro_range_t gyro_range;     ///< Gyro range configuration
  enum invensense2_accel_dlpf_t accel_dlpf;     ///< Accelerometer DLPF configuration
  enum invensense2_accel_range_t accel_range;   ///< Accelerometer range configuration

  // float temp;                         ///< temperature in degrees Celcius
};

/* External functions */
void invensense2_init(struct invensense2_t *inv);
void invensense2_periodic(struct invensense2_t *inv);
void invensense2_event(struct invensense2_t *inv);

#endif // INVENSENSE2_H
