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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef PPRZ_TRIG_INT_H
#define PPRZ_TRIG_INT_H

#include "std.h"
#include "math/pprz_algebra_int.h"

/** Allow makefile to define PPRZ_TRIG_CONST in case we want
 to make the trig tables const and store them in flash.
 Otherwise use the empty string and keep the table in RAM. */
#ifndef PPRZ_TRIG_CONST
#define PPRZ_TRIG_CONST
#endif


extern PPRZ_TRIG_CONST int16_t pprz_trig_int[];



#define PPRZ_ITRIG_SIN(_s, _a) {					\
    int32_t _an = _a;							\
    INT32_ANGLE_NORMALIZE(_an);						\
    if (_an > INT32_ANGLE_PI_2) _an = INT32_ANGLE_PI - _an;		\
    else if (_an < -INT32_ANGLE_PI_2) _an = -INT32_ANGLE_PI - _an;		\
    if (_an >= 0) _s = pprz_trig_int[_an];				\
    else _s = -pprz_trig_int[-_an];					\
  }


#define PPRZ_ITRIG_COS(_c, _a) {					\
    PPRZ_ITRIG_SIN( _c, _a + INT32_ANGLE_PI_2);				\
  }


/* http://jet.ro/files/The_neglected_art_of_Fixed_Point_arithmetic_20060913.pdf */
/* http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm */

#define R_FRAC 14

#define INT32_ATAN2(_a, _y, _x) {                   \
    const int32_t c1 = INT32_ANGLE_PI_4;            \
    const int32_t c2 = 3 * INT32_ANGLE_PI_4;        \
    const int32_t abs_y = abs(_y) + 1;              \
    int32_t r;                                      \
    if ( (_x) >= 0) {                               \
      r = (((_x)-abs_y)<<R_FRAC) / ((_x)+abs_y);    \
      (_a) = c1 - ((c1 * r)>>R_FRAC);               \
    }                                               \
    else {                                          \
      r = (((_x)+abs_y)<<R_FRAC) / (abs_y-(_x));    \
      (_a) = c2 - ((c1 * r)>>R_FRAC);               \
    }                                               \
    if ((_y)<0)                                     \
      (_a) = -(_a);                                 \
  }


#define INT32_ATAN2_2(_a, _y, _x) {                                     \
    const int32_t c1 = INT32_ANGLE_PI_4;                                \
    const int32_t c2 = 3 * INT32_ANGLE_PI_4;                            \
    const int32_t abs_y = abs(_y) + 1;                                  \
    int32_t r;                                                          \
    if ( (_x) >= 0) {                                                   \
      r = (((_x)-abs_y)<<R_FRAC) / ((_x)+abs_y);                        \
      int32_t r2 = (r * r)>>R_FRAC;                                     \
      int32_t tmp1 = ((r2 * (int32_t)ANGLE_BFP_OF_REAL(0.1963))>>INT32_ANGLE_FRAC) - ANGLE_BFP_OF_REAL(0.9817); \
      (_a) = ((tmp1 * r)>>R_FRAC) + c1;                                 \
    }                                                                   \
    else {                                                              \
      r = (((_x)+abs_y)<<R_FRAC) / (abs_y-(_x));                        \
      (_a) = c2 - ((c1 * r)>>R_FRAC);                                   \
    }                                                                   \
    if ((_y)<0)                                                         \
      (_a) = -(_a);                                                     \
  }

#endif /* PPRZ_TRIG_INT_H */
