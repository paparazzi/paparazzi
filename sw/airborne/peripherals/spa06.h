/*
 * Florian Sansou florian.sansou@enac.fr
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
 */

/**
 * @file peripherals/spa06.h
 * @brief Sensor driver for SPA06/SPL06 sensor
 *
 * 
 *
 */

#ifndef SPA06_H
#define SPA06_H

/* Header includes */
#include "peripherals/spa06_regs.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"

struct spa06_i2c_t {
  struct i2c_periph *p;         ///< Peripheral device for communication
  struct i2c_transaction trans; ///< Transaction used during configuration and measurements
  uint8_t slave_addr;               ///< The I2C slave address on the bus
};

struct spa06_spi_t {
  struct spi_periph *p;            ///< Peripheral device for communication
  uint8_t slave_idx;               ///< Slave index used for Slave Select
  struct spi_transaction trans;    ///< Transaction used during configuration and measurements

  uint8_t tx_buf[50];               ///< Transmit buffer
  uint8_t rx_buf[50];               //< Receive buffer 
};

/* Possible communication busses for the spa06 driver */
enum spa06_bus_t {
  SPA06_SPI,
  SPA06_I2C
};

enum spa06_device_t{
  UNKOWN,
  SPA06,
  SPL06
};


/**
 * @brief Different states the spa06 driver can be in
 */
enum spa06_status_t {
  SPA06_STATUS_UNINIT,
  SPA06_STATUS_IDLE,
  SPA06_STATUS_INIT_OK,
  SPA06_STATUS_GET_CALIB,
  SPA06_STATUS_CONFIGURE,
  SPA06_STATUS_READ_STATUS_REG,
  SPA06_STATUS_READ_DATA_REGS
};

/**
 * @brief Register Trim Variables
 */ 
struct spa06_reg_calbi_data {
  int16_t c0;
  int16_t c1;
  int16_t c01;
  int16_t c11;
  int16_t c20;
  int16_t c21;
  int16_t c30;
  int16_t c31;
  int16_t c40;
  int32_t c00;
  int32_t c10;
};

struct spa06_t {
  enum spa06_status_t status;           ///< state machine status
  enum spa06_device_t device;       ///< The device type detected
  bool initialized;                 ///< config done flag
  bool reset;                       //
  uint32_t timer;                     ///< Used to time operations during configuration (samples left during measuring)
  volatile bool data_available;     ///< data ready flag
  struct spa06_reg_calbi_data calib; ///< calibration data
  int32_t raw_pressure;            ///< uncompensated pressure
  int32_t raw_temperature;         ///< uncompensated temperature
  float pressure;                   ///< pressure in Pascal
  float temperature;                ///< temperature in deg Celcius
  enum spa06_bus_t bus;       ///< The communication bus used to connect the device SPI/I2C
   union {
    struct spa06_spi_t spi;     ///< SPI specific configuration
    struct spa06_i2c_t i2c;     ///< I2C specific configuration
  };
  uint8_t* rx_buffer;
  uint8_t* tx_buffer;
  uint16_t* rx_length;

  uint8_t config_idx;                 ///< The current configuration index
  uint8_t calib_idx;                 ///< The current calibration index
};


extern void spa06_read_eeprom_calib(struct spa06_t *spa);
extern void spa06_init(struct spa06_t *spa);
extern void spa06_periodic(struct spa06_t *spa);
extern void spa06_event(struct spa06_t *spa);

#endif /* SPA06_H */