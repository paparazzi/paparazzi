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
#include <math.h>

#define FALSE 0
#define TRUE (!FALSE)

/* Boolean values */
typedef uint8_t bool_t;

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



#define NormRadAngle(x) { \
    while (x > M_PI) x -= 2 * M_PI; \
    while (x < -M_PI) x += 2 * M_PI; \
  }
#define DegOfRad(x) ((x) / M_PI * 180.)
#define DeciDegOfRad(x) ((x) / M_PI * 1800.)
#define RadOfDeg(x) ((x)/180. * M_PI)

#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)

#define Bound(_x, _min, _max) { if (_x > _max) _x = _max; if (_x < _min) _x = _min; }
#define BoundAbs(_x, _max) Bound(_x, -_max, _max)


#endif /* STD_H */
