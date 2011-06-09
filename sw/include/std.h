/*
 * $Id$
 *
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
//#include <stdbool.h>
#include <math.h>

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

/* Boolean values */
typedef uint8_t bool_t;

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

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
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

#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)

#define Bound(_x, _min, _max) { if (_x > _max) _x = _max; else if (_x < _min) _x = _min; }
#define BoundAbs(_x, _max) Bound(_x, -(_max), (_max))
#define Chop(_x, _min, _max) ( (_x) < (_min) ? (_min) : (_x) > (_max) ? (_max) : (_x) )
#define ChopAbs(x, max) Chop(x, -max, max)

#define DeadBand(_x, _v) {						\
    if (_x > _v)							\
      _x = _x -_v;							\
    else if  (_x < -_v)							\
      _x = _x +_v;							\
    else								\
      _x = 0;								\
  }

#define Blend(a, b, rho) (((rho)*(a))+(1-(rho))*(b))

#define ScalarProduct(x1,y1,x2,y2) ((x1)*(x2)+(y1)*(y2))


#define RunOnceEvery(_prescaler, _code) {		\
    static uint16_t prescaler = 0;			\
    prescaler++;					\
    if (prescaler >= _prescaler) {			\
      prescaler = 0;					\
      _code;						\
    }							\
  }


#define PeriodicPrescaleBy5( _code_0, _code_1, _code_2, _code_3, _code_4) { \
    static uint8_t _50hz = 0;						\
    _50hz++;								\
    if (_50hz >= 5) _50hz = 0;						\
    switch (_50hz) {							\
    case 0:								\
      _code_0;								\
      break;								\
    case 1:								\
      _code_1;								\
      break;								\
    case 2:								\
      _code_2;								\
      break;								\
    case 3:								\
      _code_3;								\
      break;								\
    case 4:								\
      _code_4;								\
      break;								\
    }									\
  }

#define PeriodicPrescaleBy10( _code_0, _code_1, _code_2, _code_3, _code_4, _code_5, _code_6, _code_7, _code_8, _code_9) { \
    static uint8_t _cnt = 0;						\
    _cnt++;								\
    if (_cnt >= 10) _cnt = 0;						\
    switch (_cnt) {							\
    case 0:								\
      _code_0;								\
      break;								\
    case 1:								\
      _code_1;								\
      break;								\
    case 2:								\
      _code_2;								\
      break;								\
    case 3:								\
      _code_3;								\
      break;								\
    case 4:								\
      _code_4;								\
      break;								\
    case 5:								\
      _code_5;								\
      break;								\
    case 6:								\
      _code_6;								\
      break;								\
    case 7:								\
      _code_7;								\
      break;								\
    case 8:								\
      _code_8;								\
      break;								\
    case 9:								\
    default:								\
      _code_9;								\
      break;								\
    }									\
  }




#endif /* STD_H */
