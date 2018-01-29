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

#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

void rng_init(void) {}

void rng_deinit(void) {}

// Return true only if we got a new number
// that is different from the previous one
bool rng_get(uint32_t *rand_nr)
{
  int fd = open("/dev/urandom", O_RDONLY);
  if (fd == -1) {
    printf("Cannot open /dev/urandom\n");
    return false;
  }

  uint8_t len = sizeof(uint32_t);
  uint8_t res = read(fd, rand_nr, len);
  if (res != len) {
    printf("Error on reading, expected %u bytes, got %u bytes\n",
        len, res);
    return false;
  }
  close(fd);
  return true;
}

// Wait until we get a new number that is different
// from the previous one. We can wait forever here if
// the clocks are not setup properly.
uint32_t rng_wait_and_get(void) {
  uint32_t tmp = 0;
  while (!rng_get(&tmp)) {};
  return tmp;
}
