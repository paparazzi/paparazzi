/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file peripherals/lis302dl_spi.h
 *
 * Driver for LIS302DL 3-axis accelerometer from ST using SPI.
 */

#ifndef LIS302DL_SPI_H
#define LIS302DL_SPI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Include common LIS302DL options and definitions */
#include "peripherals/lis302dl.h"


struct Lis302dl_Spi {
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  volatile uint8_t tx_buf[2];
  volatile uint8_t rx_buf[8];
  enum Lis302dlConfStatus init_status; ///< init status
  bool initialized;                  ///< config done flag
  volatile bool data_available;      ///< data ready flag
  union {
    struct Int8Vect3 vect;           ///< data vector in accel coordinate system
    int8_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Lis302dlConfig config;
};

// Functions
extern void lis302dl_spi_init(struct Lis302dl_Spi *lis, struct spi_periph *spi_p, uint8_t addr);
extern void lis302dl_spi_start_configure(struct Lis302dl_Spi *lis);
extern void lis302dl_spi_read(struct Lis302dl_Spi *lis);
extern void lis302dl_spi_event(struct Lis302dl_Spi *lis);

/// convenience function: read or start configuration if not already initialized
static inline void lis302dl_spi_periodic(struct Lis302dl_Spi *lis)
{
  if (lis->initialized) {
    lis302dl_spi_read(lis);
  } else {
    lis302dl_spi_start_configure(lis);
  }
}

#endif // LIS302DL_SPI_H
