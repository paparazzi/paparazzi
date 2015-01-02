/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file peripherals/mpu9250_spi.h
 *
 * Driver for the MPU-9250 using SPI.
 */

#ifndef MPU9250_SPI_H
#define MPU9250_SPI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Include common MPU9250 options and definitions */
#include "peripherals/mpu9250.h"


#define MPU9250_BUFFER_LEN 32
#define MPU9250_BUFFER_EXT_LEN 16

enum Mpu9250SpiSlaveInitStatus {
  MPU9250_SPI_CONF_UNINIT,
  MPU9250_SPI_CONF_I2C_MST_CLK,
  MPU9250_SPI_CONF_I2C_MST_DELAY,
  MPU9250_SPI_CONF_I2C_MST_EN,
  MPU9250_SPI_CONF_SLAVES_CONFIGURE,
  MPU9250_SPI_CONF_DONE
};

struct Mpu9250_Spi {
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  volatile uint8_t tx_buf[2];
  volatile uint8_t rx_buf[MPU9250_BUFFER_LEN];
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< accel data vector in accel coordinate system
    int16_t value[3];                 ///< accel data values accessible by channel index
  } data_accel;
  union {
    struct Int16Rates rates;          ///< rates data as angular rates in gyro coordinate system
    int16_t value[3];                 ///< rates data values accessible by channel index
  } data_rates;
  uint8_t data_ext[MPU9250_BUFFER_EXT_LEN];
  struct Mpu9250Config config;
  enum Mpu9250SpiSlaveInitStatus slave_init_status;
};

// Functions
extern void mpu9250_spi_init(struct Mpu9250_Spi *mpu, struct spi_periph *spi_p, uint8_t addr);
extern void mpu9250_spi_start_configure(struct Mpu9250_Spi *mpu);
extern void mpu9250_spi_read(struct Mpu9250_Spi *mpu);
extern void mpu9250_spi_event(struct Mpu9250_Spi *mpu);

/// convenience function: read or start configuration if not already initialized
static inline void mpu9250_spi_periodic(struct Mpu9250_Spi *mpu)
{
  if (mpu->config.initialized) {
    mpu9250_spi_read(mpu);
  } else {
    mpu9250_spi_start_configure(mpu);
  }
}

#endif // MPU9250_SPI_H
