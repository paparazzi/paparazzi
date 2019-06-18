/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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
#include <stdlib.h>

/**
 * Replace new and delete operators of C++
 */

void * operator new(size_t size)
{
  if (size < 1) {
    size = 1;
  }
  return(calloc(size, 1));
}

void operator delete(void *p)
{
  if (p) free(p);
}

void * operator new[](size_t size)
{
  if (size < 1) {
    size = 1;
  }
  return(calloc(size, 1));
}

void operator delete[](void * ptr)
{
  if (ptr) free(ptr);
}
