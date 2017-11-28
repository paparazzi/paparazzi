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

/** \file rng.h
 *  \brief arch independent Random Number Generator API
 *
 */

#ifndef MCU_PERIPH_RNG_H
#define MCU_PERIPH_RNG_H

#include <inttypes.h>
#include "std.h"

void rng_init(void);
void rng_deinit(void);
bool rng_get(uint32_t *rand_nr);
uint32_t rng_wait_and_get(void);

#endif /* MCU_PERIPH_RNG_H */
