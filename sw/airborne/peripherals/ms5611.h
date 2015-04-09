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
 * @file peripherals/ms5611.h
 *
 * MS5611 barometer driver common interface (I2C and SPI).
 */

#ifndef MS5611_H
#define MS5611_H

#include "std.h"

/* Include address and register definition */
#include "peripherals/ms5611_regs.h"


enum Ms5611Status {
  MS5611_STATUS_UNINIT,
  MS5611_STATUS_RESET,
  MS5611_STATUS_RESET_OK,
  MS5611_STATUS_PROM,
  MS5611_STATUS_IDLE,
  MS5611_STATUS_CONV_D1,
  MS5611_STATUS_CONV_D1_OK,
  MS5611_STATUS_ADC_D1,
  MS5611_STATUS_CONV_D2,
  MS5611_STATUS_CONV_D2_OK,
  MS5611_STATUS_ADC_D2
};

struct Ms5611Data {
  uint32_t pressure;    ///< pressure in Pascal (0.01mbar)
  int32_t temperature;  ///< temperature with 0.01 degrees Celsius resolution
  uint16_t c[PROM_NB];
  uint32_t d1;
  uint32_t d2;
};

extern bool_t ms5611_prom_crc_ok(uint16_t *prom);
extern bool_t ms5611_calc(struct Ms5611Data *ms);
extern bool_t ms5607_calc(struct Ms5611Data *ms);

#endif /* MS5611_H */
