/*
 * This file is part of mathlib.
 *
 * Copyright (C) 2010-2011 Greg Horn <ghorn@stanford.edu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * misc_math.h
 */

#ifndef __MISC_MATH_H__
#define __MISC_MATH_H__

#include <math.h>

#define BOUND(x, min, max) {                    \
    if ((x) > (max)) (x) = (max);               \
    if ((x) < (min)) (x) = (min);               \
  }

#define SQR(x) ((x)*(x))

#define RADOFDEG(deg) ((deg) * M_PI / 180.0)

#ifdef __cplusplus
extern "C"{
#endif

  double bound(const double in, const double min, const double max);

#ifdef __cplusplus
}
#endif

#endif // __MISC_MATH_H__
