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
 * @file peripherals/ms5611_spi.h
 *
 * Measurement Specialties (Intersema) MS5611-01BA and MS5607-02BA03 pressure/temperature sensor interface for SPI.
 */

#ifndef MS5611_SPI_H
#define MS5611_SPI_H

#include "mcu_periph/spi.h"

/* Include common MS5611 definitions */
#include "peripherals/ms5611.h"

struct Ms5611_Spi {
  struct spi_periph *spi_p;
  struct spi_transaction spi_trans;
  volatile uint8_t tx_buf[1];
  volatile uint8_t rx_buf[4];
  enum Ms5611Status status;
  bool_t is_ms5607;                   ///< TRUE if MS5607, FALSE if MS5611
  bool_t initialized;                 ///< config done flag
  volatile bool_t data_available;     ///< data ready flag
  struct Ms5611Data data;
  int32_t prom_cnt;                   ///< number of bytes read from PROM
};

// Functions
extern void ms5611_spi_init(struct Ms5611_Spi *ms, struct spi_periph *spi_p, uint8_t addr,
                            bool_t is_ms5607);
extern void ms5611_spi_start_configure(struct Ms5611_Spi *ms);
extern void ms5611_spi_start_conversion(struct Ms5611_Spi *ms);
extern void ms5611_spi_periodic_check(struct Ms5611_Spi *ms);
extern void ms5611_spi_event(struct Ms5611_Spi *ms);
#if USE_CHIBIOS_RTOS
extern void ms5611_spi_synchronous_periodic_check(struct Ms5611_Spi* ms);
#endif /* USE_CHIBIOS_RTOS */

/** convenience function to trigger new measurement.
 * (or start configuration if not already initialized)
 * Still need to regularly run ms5611_spi_periodic_check to complete the measurement.
 */
static inline void ms5611_spi_read(struct Ms5611_Spi *ms)
{
  if (ms->initialized) {
    ms5611_spi_start_conversion(ms);
  } else {
    ms5611_spi_start_configure(ms);
  }
}

/// convenience function
static inline void ms5611_spi_periodic(struct Ms5611_Spi *ms)
{
  ms5611_spi_read(ms);
  ms5611_spi_periodic_check(ms);
}


#endif /* MS5611_SPI_H */
