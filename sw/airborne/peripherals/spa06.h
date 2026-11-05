/*
 * Copyright (C) 2026 OpenUAS
 * Thanks to Florian Sansou florian.sansou@enac.fr for initial implementation
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

/**
 * @file peripherals/spa06.h
 * @brief Driver for the Goertek SPA06-003 / SPL06-001 barometer (I2C or SPI)
 */

#ifndef SPA06_H
#define SPA06_H

/* Header includes */
#include "peripherals/spa06_regs.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"

/** @brief I2C bus configuration and transaction state */
struct spa06_i2c_t {
  struct i2c_periph *p;         ///< Peripheral device for communication
  struct i2c_transaction trans; ///< Transaction used during configuration and measurements
  uint8_t slave_addr;           ///< The I2C slave address on the bus
};

/** @brief SPI bus configuration and transaction state */
struct spa06_spi_t {
  struct spi_periph *p;         ///< Peripheral device for communication
  uint8_t slave_idx;            ///< Slave index used for Slave Select
  struct spi_transaction trans; ///< Transaction used during configuration and measurements

  uint8_t tx_buf[50];           ///< Transmit buffer (command byte + payload)
  uint8_t rx_buf[50];           ///< Receive buffer (dummy byte + response)
};

/** @brief Communication busses the spa06 driver can use */
enum spa06_bus_t {
  SPA06_SPI,
  SPA06_I2C
};

/** @brief Device type, detected from the chip ID during initialization */
enum spa06_device_t {
  SPA06_UNKNOWN, ///< not (yet) detected
  SPA06,         ///< SPA06-003, has the extra c31/c40 calibration coefficients
  SPL06          ///< SPL06-001
};

/**
 * @brief States of the non-blocking driver state machine
 *
 * Traversed once in order during initialization, then the driver alternates
 * between READ_STATUS_REG and READ_DATA_REGS. Any failure path leads back to
 * SPA06_STATUS_UNINIT.
 */
enum spa06_status_t {
  SPA06_STATUS_UNINIT,          ///< restart point: soft reset, then wait 40ms for the sensor to restart
  SPA06_STATUS_IDLE,            ///< read the chip ID to detect the device type
  SPA06_STATUS_INIT_OK,         ///< wait for SENSOR_RDY and COEFFS_RDY
  SPA06_STATUS_GET_COEF_SRCE,   ///< read which temperature sensor the factory calibration used
  SPA06_STATUS_GET_CALIB,       ///< read the calibration coefficients (in chunks)
  SPA06_STATUS_CONFIGURE,       ///< write the measurement configuration (register by register)
  SPA06_STATUS_READ_STATUS_REG, ///< operational: poll until pressure and temperature are ready
  SPA06_STATUS_READ_DATA_REGS   ///< operational: read the 6 result registers
};

/**
 * @brief Factory calibration coefficients (register trim values)
 *
 * Mixed-width two's complement values unpacked from the COEF registers.
 * c31 and c40 only exist on the SPA06 and are zero on the SPL06.
 */
struct spa06_reg_calib_data {
  int16_t c0;     ///< 12 bit, temperature offset
  int16_t c1;     ///< 12 bit, temperature gain
  int16_t c01;    ///< 16 bit
  int16_t c11;    ///< 16 bit
  int16_t c20;    ///< 16 bit
  int16_t c21;    ///< 16 bit
  int16_t c30;    ///< 16 bit
  int16_t c31;    ///< 12 bit, SPA06 only
  int16_t c40;    ///< 12 bit, SPA06 only
  int32_t c00;    ///< 20 bit, pressure offset
  int32_t c10;    ///< 20 bit
};
/**
 * @brief Driver instance state
 *
 * Fill in bus, and the i2c/spi member for that bus, before calling
 * spa06_init(). Consumers read pressure/temperature when data_available
 * is true and clear the flag themselves.
 */
struct spa06_t {
  enum spa06_status_t status;        ///< state machine status
  enum spa06_device_t device;        ///< The device type detected
  bool initialized;                  ///< config done flag
  bool is_broken;                    ///< sensor absent or persistently failing, retried after a backoff period
  bool reset;                        ///< reset command sent, waiting for the sensor to restart
  uint32_t timer;                    ///< times the 40ms post-reset wait and the broken-sensor retry backoff [us]
  uint8_t init_error_cnt;            ///< Number of consecutive transaction failures
  volatile bool data_available;      ///< data ready flag
  struct spa06_reg_calib_data calib; ///< calibration data
  uint8_t tmp_coef_srce;             ///< TMP_COEF_SRCE bit read from the sensor, mirrored into TMP_CFG bit 7
  int32_t raw_pressure;              ///< uncompensated pressure
  int32_t raw_temperature;           ///< uncompensated temperature
  float pressure;                    ///< pressure in Pascal
  float temperature;                 ///< temperature in deg Celcius
  enum spa06_bus_t bus;              ///< The communication bus used to connect the device SPI/I2C
  union {
    struct spa06_spi_t spi;          ///< SPI specific configuration
    struct spa06_i2c_t i2c;          ///< I2C specific configuration
  };
  volatile uint8_t *rx_buffer;       ///< points into the active bus receive buffer (SPI dummy byte already skipped)
  volatile uint8_t *tx_buffer;       ///< points into the active bus transmit buffer

  uint8_t config_idx;                ///< progress through the configuration writes (spa06_config)
  uint8_t calib_idx;                 ///< progress through the calibration chunks (spa06_get_calib/parse_calib_data)
};

/** @brief Initialize the driver instance, expects bus and i2c/spi members to be pre-filled */
extern void spa06_init(struct spa06_t *spa);
/** @brief Run the state machine, submits at most one bus transaction; call periodically */
extern void spa06_periodic(struct spa06_t *spa);
/** @brief Handle finished transactions and publish measurements; call from the event loop */
extern void spa06_event(struct spa06_t *spa);

#endif /* SPA06_H */