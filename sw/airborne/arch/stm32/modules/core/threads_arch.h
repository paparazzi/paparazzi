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

#define PPRZ_NORMAL_PRIO NORMALPRIO

#define THREADS_ATTRIBUTES __attribute__((error("Threads cannot be used in STM32 bare metal ARCH.")))

struct pprzMutex {
  int dummy;  // avoid warning: empty declaration
};

struct pprzBSem {
  volatile int value;
};


typedef int pprz_thread_t;
