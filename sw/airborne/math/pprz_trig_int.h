/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file pprz_trig_int.h
 * @brief Paparazzi fixed point trig functions.
 *
 */

#ifndef PPRZ_TRIG_INT_H
#define PPRZ_TRIG_INT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"

/** Allow makefile to define PPRZ_TRIG_CONST in case we want
 to make the trig tables const and store them in flash.
 Otherwise use the empty string and keep the table in RAM. */
#ifndef PPRZ_TRIG_CONST
#define PPRZ_TRIG_CONST
#endif

extern PPRZ_TRIG_CONST int16_t pprz_trig_int[];

extern int32_t pprz_itrig_sin(int32_t angle);
extern int32_t pprz_itrig_cos(int32_t angle);
extern int32_t int32_atan2(int32_t y, int32_t x);
extern int32_t int32_atan2_2(int32_t y, int32_t x);

/* for backwards compatibility */
#define PPRZ_ITRIG_SIN(_s, _a) { _s = pprz_itrig_sin(_a); }
#define PPRZ_ITRIG_COS(_c, _a) { _c = pprz_itrig_cos(_a); }
#define INT32_ATAN2(_a, _y, _x) { _a = int32_atan2(_y, _x); }
#define INT32_ATAN2_2(_a, _y, _x) { _a = int32_atan2_2(_y, _x); }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_TRIG_INT_H */
