/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** \file rng_arch.c
 *  \brief arch specific Random Number Generator API
 *
 */

#include "mcu_periph/rng.h"
#include <hal.h>

/***********
 * RNG API *
 ***********/

// FIXME remove and use API from ChibiOS after updating to newer version

static void TRNGStart(void)
{
  rccEnableAHB2(RCC_AHB2ENR_RNGEN, 0);
  RNG->CR |= RNG_CR_RNGEN;
}

static void TRNGStop(void)
{
  RNG->CR &= ~RNG_CR_RNGEN;
  rccDisableAHB2(RCC_AHB2ENR_RNGEN);
}

static uint32_t TRNGGet(void)
{
  // waiting for data ready
  while ((RNG->SR & RNG_SR_DRDY) == 0) {};
  return RNG->DR;
}

static bool TRNGGetErrors(void)
{
  return (RNG->SR & RNG_SR_CECS) || (RNG->SR & RNG_SR_SECS);
}

static void	TRNGClearErrors(void)
{
  RNG->SR &= ~(RNG_SR_SEIS | RNG_SR_CEIS);
}

static bool TRNGGenerate(size_t size, uint32_t *out)
{
  while (size) {
    out[--size] = TRNGGet();
  }
  return TRNGGetErrors();
}


/************
 * PPRZ API *
 ************/

void rng_init(void)
{
  TRNGStart();
}

void rng_deinit(void)
{
  TRNGStop();
}

// Return true only if we got a new number
// that is different from the previous one
bool rng_get(uint32_t *rand_nr)
{
  bool err = TRNGGenerate(1, rand_nr);
  if (err) {
    TRNGClearErrors();
    return false;
  }
  else {
    return true;
  }
}

// Wait until we get a new number that is different
// from the previous one. We can wait forever here if
// the clocks are not setup properly.
uint32_t rng_wait_and_get(void)
{
  uint32_t tmp = 0;
  bool err = true;
  do {
    err = TRNGGenerate(1, &tmp);
  }  while (err);
  return tmp;
}

