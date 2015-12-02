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

#if defined(PPRZ_TRIG_INT_TEST)
#define PPRZ_TRIG_INT_COMPR_FLASH
#define PPRZ_TRIG_INT_COMPR_HIGHEST
#define PPRZ_TRIG_INT_COMPR_HIGH
#define PPRZ_TRIG_INT_COMPR_LOW
#define PPRZ_TRIG_INT_COMPR_NONE
#endif

#if defined(PPRZ_TRIG_INT_COMPR_FLASH) && !defined(PPRZ_TRIG_INT_COMPR_HIGHEST) && !defined(PPRZ_TRIG_INT_COMPR_HIGH) && !defined(PPRZ_TRIG_INT_COMPR_LOW)
#define PPRZ_TRIG_INT_COMPR_NONE
#endif

#define TRIG_INT_SIZE           6434
#define TRIG_INT_VAL_MAX  14
#define TREE_SIZE_4             (TRIG_INT_VAL_MAX - 4)
#define TREE_SIZE_8             (TRIG_INT_VAL_MAX - 8)
#define TREE_SIZE_12            (TRIG_INT_VAL_MAX - 12)

/* resulting compressed size */
#define TRIG_INT_RLE_LEN        3379
#define TRIG_INT_COMPR_LEN      ((TRIG_INT_RLE_LEN + 1) / 2)

/* minimum size to use multi */
#define ONE_MIN   ((3 * 3) + 1)

#define TREE_BUF_12_1 1035
#define TREE_BUF_12_2 2145
#define TREE_BUF_12_3 3474

#if !defined(PPRZ_TRIG_INT_COMPR_FLASH) || defined(PPRZ_TRIG_INT_TEST)
extern PPRZ_TRIG_CONST int16_t pprz_trig_int[];
#endif

extern int32_t pprz_itrig_sin(int32_t angle);
extern int32_t pprz_itrig_cos(int32_t angle);
extern int32_t int32_atan2(int32_t y, int32_t x);
extern int32_t int32_atan2_2(int32_t y, int32_t x);

#if defined(PPRZ_TRIG_INT_COMPR_FLASH)
uint8_t get_nibble(uint16_t pos);
int pprz_trig_int_init(void);

#if defined(PPRZ_TRIG_INT_COMPR_HIGHEST)
void table_encode_4(int16_t val, int16_t val_prev, int16_t cnt, int16_t *tab);
int16_t pprz_trig_int_4(int16_t val);
#endif

#if defined(PPRZ_TRIG_INT_COMPR_HIGH)
void table_encode_8(int16_t val, int16_t val_prev, int16_t cnt, int16_t *tab);
int16_t pprz_trig_int_8(int16_t val);
#endif

#if defined(PPRZ_TRIG_INT_COMPR_LOW)
void table_encode_12(int16_t val, int16_t val_prev, int16_t cnt, int16_t *tab);
int16_t pprz_trig_int_12(int16_t val);
#endif

#if defined(PPRZ_TRIG_INT_COMPR_NONE)
int16_t pprz_trig_int_16(int32_t val);
void table_encode_16(int16_t val, int16_t cnt);
#endif

int16_t pprz_trig_int_f(int32_t angle);
#endif // PPRZ_TRIG_INT_COMPR_FLASH

/* for backwards compatibility */
#define PPRZ_ITRIG_SIN(_s, _a) { _s = pprz_itrig_sin(_a); }
#define PPRZ_ITRIG_COS(_c, _a) { _c = pprz_itrig_cos(_a); }
#define INT32_ATAN2(_a, _y, _x) { _a = int32_atan2(_y, _x); }
#define INT32_ATAN2_2(_a, _y, _x) { _a = int32_atan2_2(_y, _x); }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_TRIG_INT_H */
