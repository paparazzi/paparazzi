/*
 * Copyright (C) 2025 The Paparazzi Team
 * 
 * This file is part of paparazzi.
 *
 * Paparazzi is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * See LICENSE file for the full license version, or see http://www.gnu.org/licenses/
 */
#include "modules/core/threads.h"
#include "modules/core/threads_arch.h"
#include "stdbool.h"


int pprz_mtx_init(pprz_mutex_t* mtx) {
  (void)mtx;
  return 0;
}

int pprz_mtx_lock(pprz_mutex_t* mtx) {
  (void)mtx;
  return 0;
}

int pprz_mtx_trylock(pprz_mutex_t* mtx) {
  (void)mtx;
  (void)mtx;
  return 0;
}

int pprz_mtx_unlock(pprz_mutex_t* mtx) {
  (void)mtx;
  return 0;
}



void pprz_bsem_init(pprz_bsem_t* bsem, bool taken) {
  bsem->value = taken ? 0: 1;
}

void pprz_bsem_wait(pprz_bsem_t* bsem) {
  while (bsem->value == 0)
  {
    // active wait
    asm("NOP");
  }
  bsem->value = 0;
}

void pprz_bsem_signal(pprz_bsem_t* bsem) {
  bsem->value = 1;
}
