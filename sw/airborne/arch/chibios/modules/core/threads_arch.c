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
  chMtxObjectInit(&mtx->mtx);
  return 0;
}

int pprz_mtx_lock(pprz_mutex_t* mtx) {
  chMtxLock(&mtx->mtx);
  return 0;
}

int pprz_mtx_trylock(pprz_mutex_t* mtx) {
 return chMtxTryLock(&mtx->mtx) ? 0 : -1;
}

int pprz_mtx_unlock(pprz_mutex_t* mtx) {
  chMtxUnlock(&mtx->mtx);
  return 0;
}



void pprz_bsem_init(pprz_bsem_t* bsem, bool taken) {
  chBSemObjectInit(&bsem->bsem, taken);
}

void pprz_bsem_wait(pprz_bsem_t* bsem) {
  chBSemWait(&bsem->bsem);
}

void pprz_bsem_signal(pprz_bsem_t* bsem) {
  chBSemSignal(&bsem->bsem);
}


int pprz_thread_create(pprz_thread_t* thread, size_t size, const char *name, uint8_t prio, void (*pf)(void *), void* arg) {
  *thread = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(size), name, prio, pf, arg);
  if(*thread == NULL) {
    return -1;
  }
  return 0;
}

void pprz_thread_exit(void *retval) {
  chThdExit((msg_t)retval);
}


int pprz_thread_join(pprz_thread_t* thread, void** retval) {
  *retval = (void*)chThdWait(*thread);
  return 0;
}


int pprz_thread_tryjoin(pprz_thread_t* thread, void** retval) {
  if(chThdTerminatedX(*thread)) {
    return pprz_thread_join(thread, retval);
  }
  return -1;
}
