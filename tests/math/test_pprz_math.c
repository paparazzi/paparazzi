/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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
 * @file test_pprz_math.c
 * @brief Tests for Paparazzi math libary.
 *
 * Using libtap to create a TAP (TestAnythingProtocol) producer:
 * https://github.com/zorgnax/libtap
 *
 */

#include "tap.h"
#include "math/pprz_algebra_int.h"

int main()
{
  note("running algebra math tests");
  plan(1);

  /* test int32_vect2_normalize */
  struct Int32Vect2 v = {2300, -4200};
  int32_vect2_normalize(&v, 10);

  ok((v.x == 491 && v.y == -898),
     "int32_vect2_normalize([2300, -4200], 10) returned [%d, %d]", v.x, v.y);


  done_testing();
}
