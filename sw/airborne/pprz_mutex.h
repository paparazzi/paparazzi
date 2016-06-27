/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/**
 * @file pprz_mutex.h
 *
 * Utility functions and macros to abstract some RTOS functionalities
 * such as mutexes.
 */

#ifndef PPRZ_MUTEX_H
#define PPRZ_MUTEX_H

#if USE_CHIBIOS_RTOS

#include <ch.h>

#define PPRZ_MUTEX(_mtx) mutex_t _mtx
#define PPRZ_MUTEX_DECL(_mtx) extern mutex_t _mtx
#define PPRZ_MUTEX_INIT(_mtx) chMtxObjectInit(&(_mtx))
#define PPRZ_MUTEX_LOCK(_mtx) chMtxLock(&(_mtx))
#define PPRZ_MUTEX_UNLOCK(_mtx) chMtxUnlock(&(_mtx))

#else // no RTOS

#define PPRZ_MUTEX(_mtx)
#define PPRZ_MUTEX_DECL(_mtx)
#define PPRZ_MUTEX_INIT(_mtx) {}
#define PPRZ_MUTEX_LOCK(_mtx) {}
#define PPRZ_MUTEX_UNLOCK(_mtx) {}

#endif

#endif

