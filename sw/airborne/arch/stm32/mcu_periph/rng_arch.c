/*
 * Copyright (C) 2017 Michal Podhradsky <mpodhradsky@galois.com>
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
#include "mcu_periph/rng.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rng.h>

uint32_t last;

void rng_init(void) {
  rcc_periph_clock_enable(RCC_RNG);
  rng_enable();
  // dont forget to throw away the first generated number
  last = rng_wait_and_get();
}

void rng_deinit(void) {
  rng_disable();
  rcc_periph_clock_disable(RCC_RNG);
}

// Return true only if we got a new number
// that is different from the previous one
bool rng_get(uint32_t *rand_nr) {
  uint32_t tmp = 0;
  if (rng_get_random(&tmp) && (tmp != last)) {
    last = tmp;
    *rand_nr = tmp;
    return true;
  } else {
    return false;
  }
}

// Wait until we get a new number that is different
// from the previous one. We can wait forever here if
// the clocks are not setup properly.
uint32_t rng_wait_and_get(void) {
  uint32_t tmp = last;
  while (tmp == last) {
    tmp = rng_get_random_blocking();
  }
  last = tmp;
  return tmp;
}
