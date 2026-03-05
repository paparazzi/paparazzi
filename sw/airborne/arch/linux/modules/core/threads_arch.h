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

#pragma once

#include <pthread.h>
#include <semaphore.h>

#define PPRZ_NORMAL_PRIO 128

#define THREADS_ATTRIBUTES

struct pprzMutex {
  pthread_mutex_t mtx;
};

struct pprzBSem {
  pthread_mutex_t mtx;
  sem_t sem;
};


typedef pthread_t pprz_thread_t;
