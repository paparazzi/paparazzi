/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 */

/* Linear Least Square regression */

#include "lls.h"
#include "infrared.h"

float lls_a;
float lls_b;
float lls_x;
float lls_y;

void lls_init() {
  float lls_a_init = ir_gain;
  float lls_b_init = (float)(-ir_roll_neutral) * lls_a_init;
  lls_x = ir_roll_neutral + LLS_IR_HALF_INTERVAL;
  lls_y = lls_a_init * lls_x + lls_b_init;
  lls_update();
  lls_x = ir_roll_neutral - LLS_IR_HALF_INTERVAL;
  lls_y = lls_a_init * lls_x + lls_b_init;
  lls_update();
}

void lls_update() {
  static float sum_x = 0.;
  static float sum_y = 0.;
  static float sum_xy = 0.;
  static float sum_x2 = 0.;
  static uint16_t n = 0;
  float fn;
  float mean_x, mean_y, c_xy, s2_x;

  n++;
  sum_x += lls_x;
  sum_y += lls_y;
  sum_xy += lls_x * lls_y;
  sum_x2 += lls_x * lls_x;
  fn = (float)n;

  mean_x = sum_x / fn;
  mean_y = sum_y / fn;
  c_xy = mean_x * mean_y + (sum_xy - mean_x * sum_y - mean_y * sum_x ) / fn;
  s2_x = mean_x * mean_x  + (sum_x2 - 2* mean_x * sum_x) / fn;
  lls_a = c_xy / s2_x;
  lls_b = mean_y - lls_a * mean_x;
}
