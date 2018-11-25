/*
 * Copyright (C) 2018 Michal Podhradsky <mpodhradsky@galois.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file rng_arch.c
 *  \brief arch specific Random Number Generator API
 *
 */
#include "hal.h"
#include <string.h>

struct CRYDriver cryp;
const CRYConfig *config;

uint8_t crypto_buf[16] = {};

void rng_init(void) {
  cryObjectInit(&cryp);
  cryStart(&cryp, config);
}

void rng_deinit(void) {
  cryStop(&cryp);
}

bool rng_get(uint32_t *rand_nr) {
  if (CRY_NOERROR == cryTRNG(&cryp,crypto_buf)) {
    memcpy(rand_nr, crypto_buf, sizeof(*rand_nr));
    return true;
  } else {
    return false;
  }
}

uint32_t rng_wait_and_get(void) {
  uint32_t rnd = 0;
  cryTRNG(&cryp,crypto_buf);
  memcpy(&rnd, crypto_buf, sizeof(rnd));
  return rnd;
}
