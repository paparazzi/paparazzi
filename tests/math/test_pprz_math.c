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
#include "math/pprz_algebra_float.h"

int main()
{
  note("running algebra math tests");
   plan(6); // number of tests in this file to run

  /* test int32_vect2_normalize */
  struct Int32Vect2 v = {2300, -4200};
  int32_vect2_normalize(&v, 10);

  ok((v.x == 491 && v.y == -898),
     "int32_vect2_normalize([2300, -4200], 10) returned [%d, %d]", v.x, v.y);

  /*test float_eulers_of_quat_zxy*/
  struct FloatQuat quat = {0.9266,   -0.2317,    0.1165,    0.2722};
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, &quat);

  ok((fabs(eulers_zxy.psi - 0.6436) < 0.01 && fabs(eulers_zxy.phi - -0.3746) < 0.01 && fabs(eulers_zxy.theta - 0.3763) < 0.01),
     "float_eulers_of_quat_zxy(0.9266,   -0.2317,    0.1165,    0.2722) returned [%f, %f, %f]", eulers_zxy.phi, eulers_zxy.theta, eulers_zxy.psi);

  /*test float_quat_of_eulers_zxy*/
  struct FloatQuat quat_zxy;
  float_quat_of_eulers_zxy(&quat_zxy, &eulers_zxy);
  ok((fabs(quat_zxy.qi - 0.9266) < 0.01 && fabs(quat_zxy.qx - -0.2317) < 0.01 && fabs(quat_zxy.qy - 0.1165) < 0.01) && fabs(quat_zxy.qz - 0.2722),
     "float_quat_of_eulers_zxy(float_eulers_of_quat_zxy(0.9266,   -0.2317,    0.1165,    0.2722)) returned [%f, %f, %f, %f]", quat_zxy.qi, quat_zxy.qx, quat_zxy.qy, quat_zxy.qz);

  /* test int32_quat_normalize */
  struct Int32Quat q = {32768, 0, 0, 0}; // 1.0 in Q15 format
  q.qx = 16384; // 0.5 in Q15
  q.qy = 0;
  q.qz = 0;
  int32_quat_normalize(&q);
  // After normalization, the quaternion should be unit length (|q| = 1.0 in Q15)
  // For [1, 0.5, 0, 0], norm = sqrt(1^2 + 0.5^2) = sqrt(1.25) ≈ 1.118
  // Normalized: [1/1.118, 0.5/1.118, 0, 0] ≈ [0.894, 0.447, 0, 0]
  // In Q15: 0.894*32768 ≈ 29300, 0.447*32768 ≈ 14650
  ok((q.qi >= 29295 && q.qi <= 29315) && (q.qx >= 14645 && q.qx <= 14655), // allowing a small margin of error due to integer rounding
     "int32_quat_normalize([32768,16384,0,0]) returned [%d, %d, %d, %d] (expected approx [29300,14650,0,0] +/- 15)", q.qi, q.qx, q.qy, q.qz);

  uint32_t sqrt_100 = int32_sqrt(100);
  ok(sqrt_100 == 10, "int32_sqrt(100) == %d (expected 10)", sqrt_100);

  uint32_t sqrt_12345 = int32_sqrt(12345);
  ok(sqrt_12345 >= 111 && sqrt_12345 <= 112, "int32_sqrt(12345) == %d (expected approx 111)", sqrt_12345);

  done_testing();
}
