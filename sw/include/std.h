/*
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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
 *
 *
 * a couple of fundamentals used in the avr code
 *
 */

#ifndef STD_H
#define STD_H

#include <inttypes.h>
#include <stdbool.h>
#include <math.h>

/* some convenience macros to print debug/config messages at compile time */
#include "message_pragmas.h"

/* stringify a define, e.g. one that was not quoted */
#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

#define PTR(_f) &_f

#ifndef FALSE
#define FALSE false
#endif
#ifndef TRUE
#define TRUE true
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

/* Unit (void) values */
typedef uint8_t unit_t;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_4
#define M_PI_4 (M_PI/4)
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI/2)
#endif


#ifndef bit_is_set
#define bit_is_set(x, b) ((x >> b) & 0x1)
#endif

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#define SetBit(a, n) a |= (1 << n)
#define ClearBit(a, n) a &= ~(1 << n)

#define NormRadAngle(x) { \
    while (x > M_PI) x -= 2 * M_PI; \
    while (x < -M_PI) x += 2 * M_PI; \
  }
#define DegOfRad(x) ((x) * (180. / M_PI))
#define DeciDegOfRad(x) ((x) * (1800./ M_PI))
#define RadOfDeg(x) ((x) * (M_PI/180.))
#define RadOfDeciDeg(x) ((x) * (M_PI/1800.))

#define MOfCm(_x) (((float)(_x))/100.)
#define MOfMm(_x) (((float)(_x))/1000.)

#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)

#ifndef ABS
#define ABS(val) ((val) < 0 ? -(val) : (val))
#endif

#define BoundUpper(_x, _max) { if (_x > (_max)) _x = (_max);}


#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }
#define BoundInverted(_x, _min, _max) {           \
    if ((_x < (_min)) && (_x > (_max))) {         \
      if (abs(_x - (_min)) < abs(_x - (_max)))    \
        _x = (_min);                              \
      else                                        \
        _x = (_max);                              \
    }                                             \
  }
#define BoundWrapped(_x, _min, _max) {            \
    if ((_max) > (_min))                          \
      Bound(_x, _min, _max)                       \
      else                                        \
        BoundInverted(_x, _min, _max)             \
      }
#define BoundAbs(_x, _max) Bound(_x, -(_max), (_max))
#define Clip(_x, _min, _max) ( (_x) < (_min) ? (_min) : (_x) > (_max) ? (_max) : (_x) )
#define ClipAbs(x, max) Clip(x, -(max), (max))
// Align makes the value of x a multiple of a1
#define Align(_x, _a1) (_x%_a1 ? _x + (_a1 - (_x%_a1)) : _x )

#define DeadBand(_x, _v) {            \
    if (_x > (_v))                    \
      _x = _x -(_v);                  \
    else if  (_x < -(_v))             \
      _x = _x +(_v);                  \
    else                              \
      _x = 0;                         \
  }

#define Blend(a, b, rho) (((rho)*(a))+(1-(rho))*(b))

#define RunOnceEvery(_prescaler, _code) {   \
    static uint16_t prescaler = 0;          \
    prescaler++;                            \
    if (prescaler >= _prescaler) {          \
      prescaler = 0;                        \
      _code;                                \
    }                                       \
  }

#define RunXTimesEvery(_jumpstart, _prescaler, _interval, _xtimes, _code) {   \
    static uint16_t prescaler = _jumpstart;     \
    static uint16_t xtimes = 0;                 \
    prescaler++;                                \
    if (prescaler >= _prescaler + _interval*xtimes && xtimes < _xtimes) {     \
      _code;                                    \
      xtimes++;                                 \
    }                                           \
    if (xtimes >= _xtimes) {                    \
      xtimes = 0;                               \
      prescaler = 0;                            \
    }                                           \
  }


#define PeriodicPrescaleBy5( _code_0, _code_1, _code_2, _code_3, _code_4) { \
    static uint8_t _50hz = 0;           \
    _50hz++;                            \
    if (_50hz >= 5) _50hz = 0;          \
    switch (_50hz) {                    \
      case 0:                           \
        _code_0;                        \
        break;                          \
      case 1:                           \
        _code_1;                        \
        break;                          \
      case 2:                           \
        _code_2;                        \
        break;                          \
      case 3:                           \
        _code_3;                        \
        break;                          \
      case 4:                           \
        _code_4;                        \
        break;                          \
    }                                   \
  }

#define PeriodicPrescaleBy10( _code_0, _code_1, _code_2, _code_3, _code_4, _code_5, _code_6, _code_7, _code_8, _code_9) { \
    static uint8_t _cnt = 0;            \
    _cnt++;                             \
    if (_cnt >= 10) _cnt = 0;           \
    switch (_cnt) {                     \
      case 0:                           \
        _code_0;                        \
        break;                          \
      case 1:                           \
        _code_1;                        \
        break;                          \
      case 2:                           \
        _code_2;                        \
        break;                          \
      case 3:                           \
        _code_3;                        \
        break;                          \
      case 4:                           \
        _code_4;                        \
        break;                          \
      case 5:                           \
        _code_5;                        \
        break;                          \
      case 6:                           \
        _code_6;                        \
        break;                          \
      case 7:                           \
        _code_7;                        \
        break;                          \
      case 8:                           \
        _code_8;                        \
        break;                          \
      case 9:                           \
      default:                          \
        _code_9;                        \
        break;                          \
    }                                   \
  }

static inline bool str_equal(const char *a, const char *b)
{
  int i = 0;
  while (!(a[i] == 0 && b[i] == 0)) {
    if (a[i] != b[i]) { return FALSE; }
    i++;
  }
  return TRUE;
}

#ifdef __GNUC__
#  define UNUSED __attribute__((__unused__))
#  define WEAK __attribute__((weak))
#else
#  define UNUSED
#  define WEAK
#endif

#if __GNUC__ >= 7
#  define INTENTIONAL_FALLTHRU __attribute__ ((fallthrough));
#else
#  define INTENTIONAL_FALLTHRU
#endif

#endif /* STD_H */
