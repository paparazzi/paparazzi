/*
 * $Id$
 *  
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

/* Allow makefile to define BOOZ_TRIG_CONST in case we want
 to make the trig tables const and store them in flash.
 Otherwise use the empty string and keep the table in RAM. */
#ifndef PPRZ_TRIG_CONST
#define PPRZ_TRIG_CONST
#endif


extern PPRZ_TRIG_CONST int16_t pprz_trig_int[];



#define PPRZ_ITRIG_SIN(_s, _a) {					\
    int32_t an = _a;							\
    INT32_ANGLE_NORMALIZE(an);						\
    if (an > INT32_ANGLE_PI_2) an = INT32_ANGLE_PI - an;		\
    else if (an < -INT32_ANGLE_PI_2) an = -INT32_ANGLE_PI - an;		\
    if (an >= 0) _s = pprz_trig_int[an];				\
    else _s = -pprz_trig_int[-an];					\
  }


#define PPRZ_ITRIG_COS(_c, _a) {					\
    PPRZ_ITRIG_SIN( _c, _a + INT32_ANGLE_PI_2);				\
  }



#endif /* PPRZ_TRIG_INT_H */
