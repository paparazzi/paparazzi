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
 * @file peripherals/mpu60x0_spi.h
 *
 * Driver for the MPU-60X0 using SPI.
 */

#ifndef MPU60X0_SPI_H
#define MPU60X0_SPI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Include common MPU60X0 options and definitions */
#include "peripherals/mpu60x0.h"


#define MPU60X0_BUFFER_LEN 32
#define MPU60X0_BUFFER_EXT_LEN 16

enum Mpu60x0SpiSlaveInitStatus {
  MPU60X0_SPI_CONF_UNINIT,
  MPU60X0_SPI_CONF_I2C_MST_CLK,
  MPU60X0_SPI_CONF_I2C_MST_DELAY,
  MPU60X0_SPI_CONF_I2C_MST_EN,
  MPU60X0_SPI_CONF_SLAVES_CONFIGURE,
  MPU60X0_SPI_CONF_DONE
};

struct Mpu60x0_Spi {
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  volatile uint8_t tx_buf[2];
  volatile uint8_t rx_buf[MPU60X0_BUFFER_LEN];
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< accel data vector in accel coordinate system
    int16_t value[3];                 ///< accel data values accessible by channel index
  } data_accel;
  union {
    struct Int16Rates rates;          ///< rates data as angular rates in gyro coordinate system
    int16_t value[3];                 ///< rates data values accessible by channel index
  } data_rates;
  float temp;                         ///< temperature in degrees Celcius
  uint8_t data_ext[MPU60X0_BUFFER_EXT_LEN];
  struct Mpu60x0Config config;
  enum Mpu60x0SpiSlaveInitStatus slave_init_status;
};

// Functions
extern void mpu60x0_spi_init(struct Mpu60x0_Spi *mpu, struct spi_periph *spi_p, uint8_t addr);
extern void mpu60x0_spi_start_configure(struct Mpu60x0_Spi *mpu);
extern void mpu60x0_spi_read(struct Mpu60x0_Spi *mpu);
extern void mpu60x0_spi_event(struct Mpu60x0_Spi *mpu);

/// convenience function: read or start configuration if not already initialized
static inline void mpu60x0_spi_periodic(struct Mpu60x0_Spi *mpu)
{
  if (mpu->config.initialized) {
    mpu60x0_spi_read(mpu);
  } else {
    mpu60x0_spi_start_configure(mpu);
  }
}

#endif // MPU60X0_SPI_H
