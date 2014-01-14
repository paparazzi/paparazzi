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
 * @file peripherals/l3gd20_spi.h
 *
 * Driver for L3GD20 3-axis accelerometer from ST using SPI.
 */

#ifndef L3GD20_SPI_H
#define L3GD20_SPI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Include common L3GD20 options and definitions */
#include "peripherals/l3gd20.h"

struct L3gd20_Spi {
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  volatile uint8_t tx_buf[2];
  volatile uint8_t rx_buf[8];
  enum L3gd20ConfStatus init_status; ///< init status
  bool_t initialized;                  ///< config done flag
  volatile bool_t data_available;      ///< data ready flag
  union {
    struct Int16Rates rates;           ///< data vector in accel coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data_rates;
  struct L3gd20Config config;
};

// Functions
extern void l3gd20_spi_init(struct L3gd20_Spi *l3g, struct spi_periph *spi_p, uint8_t addr);
extern void l3gd20_spi_start_configure(struct L3gd20_Spi *l3g);
extern void l3gd20_spi_read(struct L3gd20_Spi *l3g);
extern void l3gd20_spi_event(struct L3gd20_Spi *l3g);

/// convenience function: read or start configuration if not already initialized
static inline void l3gd20_spi_periodic(struct L3gd20_Spi *l3g) {
  if (l3g->initialized)
    l3gd20_spi_read(l3g);
  else
    l3gd20_spi_start_configure(l3g);
}

#endif // L3GD20_SPI_H
