/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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
 * @file peripherals/ms2100.h
 * Driver for the ms2100 magnetic sensor from PNI.
 */

#ifndef MS2100_H
#define MS2100_H

#include "std.h"
#include "mcu_periph/spi.h"
#include "math/pprz_algebra_int.h"


enum Ms2100Status {
  MS2100_IDLE,
  MS2100_SENDING_REQ,
  MS2100_GOT_EOC,
  MS2100_READING_RES,
  MS2100_DATA_AVAILABLE
};

struct Ms2100 {
  struct spi_periph *spi_p;
  struct spi_transaction req_trans;
  struct spi_transaction read_trans;
  volatile uint8_t req_buf[1];        ///< SPI buffer for the command byte
  volatile uint8_t read_buf[2];       ///< SPI buffer for reading a single axis
  volatile enum Ms2100Status status;
  volatile uint8_t cur_axe;
  union {
    struct Int16Vect3 vect;           ///< data vector in mag coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
};

// keep gobal var for now...
extern struct Ms2100 ms2100;

extern void ms2100_init(struct Ms2100 *ms, struct spi_periph *spi_p, uint8_t slave_idx);
extern void ms2100_read(struct Ms2100 *ms);
extern void ms2100_event(struct Ms2100 *ms);

static inline void ms2100_periodic(struct Ms2100 *ms)
{
  if (ms->status == MS2100_IDLE) {
    ms2100_read(ms);
  }
}

/* underlying architecture */
#include "peripherals/ms2100_arch.h"
/* must be implemented by underlying architecture */
extern void ms2100_arch_init(void);

#endif /* MS2100_H */
