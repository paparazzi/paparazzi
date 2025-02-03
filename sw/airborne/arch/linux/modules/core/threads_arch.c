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
#include <semaphore.h>
#include <stdlib.h>


int pprz_mtx_init(pprz_mutex_t* mtx) {
  return pthread_mutex_init(&mtx->mtx, NULL);
}

int pprz_mtx_lock(pprz_mutex_t* mtx) {
  return pthread_mutex_lock(&mtx->mtx);
}

int pprz_mtx_trylock(pprz_mutex_t* mtx) {
 return pthread_mutex_trylock(&mtx->mtx);
}

int pprz_mtx_unlock(pprz_mutex_t* mtx) {
  return pthread_mutex_unlock(&mtx->mtx);
}



void pprz_bsem_init(pprz_bsem_t* bsem, bool taken) {
  pthread_mutex_init(&bsem->mtx, NULL);
  sem_init(&bsem->sem, 0, taken ? 0: 1);
}

void pprz_bsem_wait(pprz_bsem_t* bsem) {
  sem_wait(&bsem->sem);
}

void pprz_bsem_signal(pprz_bsem_t* bsem) {
  int val;
  pthread_mutex_lock(&bsem->mtx);
  if(!sem_getvalue(&bsem->sem, &val)) {
    if(val < 1) {
      sem_post(&bsem->sem);
    }
  }
  pthread_mutex_unlock(&bsem->mtx);
}



// stub function argument: the thread function to run with its argument.
struct stub_arg
{
  void (*func)(void*);
  void* arg;
};

// stub function, ran in a new thread
static void* stub(void* arg) {
  struct stub_arg* sa = (struct stub_arg*) arg;
  sa->func(sa->arg);
  free(sa);
  return NULL;
}

int pprz_thread_create(pprz_thread_t* thread, size_t size, const char *name, uint8_t prio, void (*func)(void*), void* arg) {
  (void)size;
  (void)prio;
  struct stub_arg *sa = malloc(sizeof(struct stub_arg));   // allocate a stub arg. Must be allocated on the heap
  sa->func = func;
  sa->arg = arg;
  int ret = pthread_create(thread, NULL, stub, (void*)sa);  // launch the stub in a new thread
  if(ret) {return ret;}
  return pthread_setname_np(*thread, name);
}

void pprz_thread_exit(void *retval) {
  pthread_exit(retval);
}

int pprz_thread_join(pprz_thread_t* thread, void** retval) {
  return pthread_join(*thread, retval);
}

int pprz_thread_tryjoin(pprz_thread_t* thread, void** retval) {
  return pthread_tryjoin_np(*thread, retval);
}
