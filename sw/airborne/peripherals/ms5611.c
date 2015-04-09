/*
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
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
 * @file peripherals/ms5611.c
 *
 * MS5611 and MS5607 barometer driver common functions (I2C and SPI).
 */

#include "peripherals/ms5611.h"
#include "std.h"

/**
 * Check if CRC of PROM data is OK.
 * @return TRUE if OK, FALSE otherwise
 */
bool_t ms5611_prom_crc_ok(uint16_t *prom)
{
  int32_t i, j;
  uint32_t res = 0;
  uint8_t crc = prom[7] & 0xF;
  prom[7] &= 0xFF00;
  for (i = 0; i < 16; i++) {
    if (i & 1) {
      res ^= ((prom[i >> 1]) & 0x00FF);
    } else {
      res ^= (prom[i >> 1] >> 8);
    }
    for (j = 8; j > 0; j--) {
      if (res & 0x8000) {
        res ^= 0x1800;
      }
      res <<= 1;
    }
  }
  prom[7] |= crc;
  if (crc == ((res >> 12) & 0xF)) {
    return TRUE;
  } else {
    return FALSE;
  }
}

/**
 * Calculate temperature and compensated pressure for MS5611.
 * @return TRUE if measurement was valid, FALSE otherwise
 */
bool_t ms5611_calc(struct Ms5611Data *ms)
{
  int64_t dt, tempms, off, sens, t2, off2, sens2;

  /* difference between actual and ref temperature */
  dt = ms->d2 - (int64_t)ms->c[5] * (1 << 8);
  /* actual temperature */
  tempms = 2000 + ((int64_t)dt * ms->c[6]) / (1 << 23);
  /* offset at actual temperature */
  off = ((int64_t)ms->c[2] * (1 << 16)) + ((int64_t)ms->c[4] * dt) / (1 << 7);
  /* sensitivity at actual temperature */
  sens = ((int64_t)ms->c[1] * (1 << 15)) + ((int64_t)ms->c[3] * dt) / (1 << 8);
  /* second order temperature compensation */
  if (tempms < 2000) {
    t2 = (dt * dt) / (1 << 31);
    off2 = 5 * ((int64_t)(tempms - 2000) * (tempms - 2000)) / (1 << 1);
    sens2 = 5 * ((int64_t)(tempms - 2000) * (tempms - 2000)) / (1 << 2);
    if (tempms < -1500) {
      off2 = off2 + 7 * (int64_t)(tempms + 1500) * (tempms + 1500);
      sens2 = sens2 + 11 * ((int64_t)(tempms + 1500) * (tempms + 1500)) / (1 << 1);
    }
    tempms = tempms - t2;
    off = off - off2;
    sens = sens - sens2;
  }

  /* temperature compensated pressure in Pascal (0.01mbar) */
  uint32_t p = (((int64_t)ms->d1 * sens) / (1 << 21) - off) / (1 << 15);
  /* if temp and pressare are in valid bounds, copy and return TRUE (valid) */
  if ((tempms > -4000) && (tempms < 8500) && (p > 1000) && (p < 120000)) {
    /* temperature in deg Celsius with 0.01 degC resolultion */
    ms->temperature = (int32_t)tempms;
    ms->pressure = p;
    return TRUE;
  }
  return FALSE;
}

/**
 * Calculate temperature and compensated pressure for MS5607.
 * MS5607 basically has half the resolution of the MS5611.
 * @return TRUE if measurement was valid, FALSE otherwise
 */
bool_t ms5607_calc(struct Ms5611Data *ms)
{
  int64_t dt, tempms, off, sens, t2, off2, sens2;

  /* difference between actual and ref temperature */
  dt = ms->d2 - (int64_t)ms->c[5] * (1 << 8);
  /* actual temperature */
  tempms = 2000 + ((int64_t)dt * ms->c[6]) / (1 << 23);
  /* offset at actual temperature */
  off = ((int64_t)ms->c[2] * (1 << 17)) + ((int64_t)ms->c[4] * dt) / (1 << 6);
  /* sensitivity at actual temperature */
  sens = ((int64_t)ms->c[1] * (1 << 16)) + ((int64_t)ms->c[3] * dt) / (1 << 7);
  /* second order temperature compensation */
  if (tempms < 2000) {
    t2 = (dt * dt) / (1 << 31);
    off2 = 61 * ((int64_t)(tempms - 2000) * (tempms - 2000)) / (1 << 4);
    sens2 = 2 * ((int64_t)(tempms - 2000) * (tempms - 2000));
    if (tempms < -1500) {
      off2 = off2 + 15 * (int64_t)(tempms + 1500) * (tempms + 1500);
      sens2 = sens2 + 8 * ((int64_t)(tempms + 1500) * (tempms + 1500));
    }
    tempms = tempms - t2;
    off = off - off2;
    sens = sens - sens2;
  }

  /* temperature compensated pressure in Pascal (0.01mbar) */
  uint32_t p = (((int64_t)ms->d1 * sens) / (1 << 21) - off) / (1 << 15);
  /* if temp and pressare are in valid bounds, copy and return TRUE (valid) */
  if ((tempms > -4000) && (tempms < 8500) && (p > 1000) && (p < 120000)) {
    /* temperature in deg Celsius with 0.01 degC resolultion */
    ms->temperature = (int32_t)tempms;
    ms->pressure = p;
    return TRUE;
  }
  return FALSE;
}
