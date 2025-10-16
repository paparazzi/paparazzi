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
 * @file peripherals/invensense3_456.h
 *
 * Driver for the Invensense 456XY IMUs:
 * - ICM45686
 *
 * This driver is largely based on the original Invense v3 driver by Freek van Tienen,
 * the Arupilot AP_InertialSensor_Invensensev3 with some inspiration from the px4 one
 * as well. Split from the "main" invensensev3 driver as the WHO_AM_I register is different,
 * and we operate this driver in high-resolution (20-bit) mode which is not used by any
 * of the existing drivers. TL;DR: the drivers are quite different (even sensor ODR's),
 * so we would be using if statements everywhere anyway. This seemed like the cleaner
 * solution.
 */

#ifndef INVENSENSE3_456_H
#define INVENSENSE3_456_H

#include "std.h"
#include "mcu_periph/spi.h"

// Hold 22 measurements + 3 for the register address and length
#define INVENSENSE3_456_FIFO_BUFFER_LEN 22
#define INVENSENSE3_456_BUFFER_SIZE INVENSENSE3_456_FIFO_BUFFER_LEN * sizeof(struct FIFODataHighRes) + 3

// REF: https://github.com/ArduPilot/ardupilot/blob/85a8a55611a95d4c59052f79f56744a93a9c5a63/libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp#L160C1-L167C3
struct __attribute__((packed)) FIFODataHighRes {
    uint8_t header;
    uint8_t accel[6];
    uint8_t gyro[6];
    int16_t temperature;
    uint16_t timestamp;
    uint8_t gx : 4, ax : 4, gy : 4, ay : 4, gz : 4, az : 4;
};

/* Invensense v3 SPI peripheral */
/* Modified to have a 4-bit tx buffer to deal with the different way of accessing register banks */
struct invensense3_456_spi_t {
  struct spi_periph *p;                             ///< Peripheral device for communication
  uint8_t slave_idx;                                ///< Slave index used for Slave Select
  struct spi_transaction trans;                     ///< Transaction used during configuration and measurements

  uint8_t tx_buf[4];                       ///< Transmit buffer
  uint8_t rx_buf[INVENSENSE3_456_BUFFER_SIZE]; ///< Receive buffer
};

/* Different states the invensense v3 driver can be in */
enum invensense3_456_status_t {
  INVENSENSE3_456_IDLE,
  INVENSENSE3_456_CONFIG,
  INVENSENSE3_456_RUNNING
};
/* Different device types compatible with the invensense v3 - 456xy driver */
enum invensense3_456_device_t {
  INVENSENSE3_456_UNKOWN,
  INVENSENSE3_456_ICM45686
};

/* Output data rate configuration - for this initial driver version,
   we operate the accelerometer/gyro at the same ODR */
enum invensense3_456_odr_t {
  INVENSENSE3_456_ODR_6_4KHZ = 0b0011,  // LN only
  INVENSENSE3_456_ODR_3_2KHZ = 0b0100,  // LN only
  INVENSENSE3_456_ODR_1_6KHZ = 0b0101, // LN only
  INVENSENSE3_456_ODR_800HZ = 0b0110, // LN only
  INVENSENSE3_456_ODR_400HZ = 0b0111, // LP / LN
  INVENSENSE3_456_ODR_200HZ = 0b1000, // LP / LN
  INVENSENSE3_456_ODR_100HZ = 0b1001, // LP / LN
  INVENSENSE3_456_ODR_50HZ = 0b1010, // LP / LN
  INVENSENSE3_456_ODR_25HZ = 0b1011, // LP / LN
  // INVENSENSE3_456_ODR_12_5HZ = 0b1100, // LP / LN - NOT USED
  // INVENSENSE3_456_ODR_6_25HZ = 0b1101, // LP only - NOT USED
  // INVENSENSE3_456_ODR_3_125HZ = 0b1110, // LP only - NOT USED
  // INVENSENSE3_456_ODR_1_5625HZ = 0b1111,  // LP only - NOT USED
};

/* Main invensense 456 device structure */
struct invensense3_456_t {
  uint8_t abi_id;                     ///< The ABI id used to broadcast the device measurements
  enum invensense3_456_status_t status;   ///< Status of the invensense device
  enum invensense3_456_device_t device;   ///< The device type detected

  struct invensense3_456_spi_t spi;     ///< SPI specific configuration (support SPI only for now)
  uint8_t* rx_buffer;
  uint8_t* tx_buffer;
  uint16_t* rx_length;

  uint8_t config_idx;                 ///< The current configuration index (used for setup/initial configuration)
  uint32_t timer;                     ///< Used to time operations during configuration (samples left during measuring)

  enum invensense3_456_odr_t imu_odr;         ///< Accelerometer / gyro Output Data Rate configuration
  
  int sample_numbers;                           ///< expected FIFO packet number, assuming reading at PERIODIC_FREQUENCY
  float imu_samplerate;                        ///< Sample rate in Hz from the imu_odr
};

/* External functions */
void invensense3_456_init(struct invensense3_456_t *inv);
void invensense3_456_periodic(struct invensense3_456_t *inv);
void invensense3_456_event(struct invensense3_456_t *inv);

#endif // INVENSENSE3_456_H
