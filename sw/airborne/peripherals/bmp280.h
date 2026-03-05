/*
 * Chris Efstathiou hendrixgr@gmail.com
 * Florian Sansou florian.sansou@enac.fr
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
 * @file peripherals/bmp280.h
 * @brief Sensor driver for BMP280 sensor 
 *
 *
 */

#ifndef BMP280_H
#define BMP280_H

/* Header includes */
#include "peripherals/bmp280_regs.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"

struct bmp280_i2c_t {
  struct i2c_periph *p;         ///< Peripheral device for communication
  struct i2c_transaction trans; ///< Transaction used during configuration and measurements
  uint8_t slave_addr;               ///< The I2C slave address on the bus
};

struct bmp280_spi_t {
  struct spi_periph *p;            ///< Peripheral device for communication
  uint8_t slave_idx;               ///< Slave index used for Slave Select
  struct spi_transaction trans;    ///< Transaction used during configuration and measurements

  uint8_t tx_buf[50];               ///< Transmit buffer
  uint8_t rx_buf[50];               //< Receive buffer 
};

/* Possible communication busses for the bmp280 driver */
enum bmp280_bus_t {
  BMP280_SPI,
  BMP280_I2C
};

enum bmp_device_t{
  BMP_UNKOWN,
  BMP_280
};

/* Different states the bmp driver can be in */
enum bmp280_status_t {
  BMP280_STATUS_UNINIT,
  BMP280_STATUS_IDLE,
  BMP280_STATUS_GET_CALIB,
  BMP280_STATUS_CONFIGURE,
  BMP280_STATUS_READ_STATUS_REG,
  BMP280_STATUS_READ_DATA_REGS
};

// brief Register Trim Variables
struct bmp280_reg_calib_data_t {
  uint16_t dig_t1;
  int16_t dig_t2;
  int16_t dig_t3;
  uint16_t dig_p1;
  int16_t dig_p2;
  int16_t dig_p3;
  int16_t dig_p4;
  int16_t dig_p5;
  int16_t dig_p6;
  int16_t dig_p7;
  int16_t dig_p8;
  int16_t dig_p9;
  int64_t t_fine;
};


struct bmp280_t {
  enum bmp280_status_t status;           ///< state machine status
  enum bmp_device_t device;   ///< The device type detected
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  struct bmp280_reg_calib_data_t calib; ///< calibration data
  uint32_t raw_pressure;            ///< uncompensated pressure
  uint32_t raw_temperature;         ///< uncompensated temperature
  float pressure;                   ///< pressure in Pascal
  float temperature;                ///< temperature in deg Celcius
  enum bmp280_bus_t bus;       ///< The communication bus used to connect the device SPI/I2C
   union {
    struct bmp280_spi_t spi;     ///< SPI specific configuration
    struct bmp280_i2c_t i2c;     ///< I2C specific configuration
  };
  uint8_t* rx_buffer;
  uint8_t* tx_buffer;
  uint16_t* rx_length;

  uint8_t config_idx;                 ///< The current configuration index
};


extern void bmp280_read_eeprom_calib(struct bmp280_t *bmp);
extern void bmp280_init(struct bmp280_t *bmp);
extern void bmp280_periodic(struct bmp280_t *bmp);
extern void bmp280_event(struct bmp280_t *bmp);

#endif /* BMP280_H */
