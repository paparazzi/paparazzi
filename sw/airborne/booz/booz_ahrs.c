/*
 * $Id$
 *
 * Copyright (C) 2009 Felix Ruess <felix.ruess@gmail.com>
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

#include "booz_ahrs.h"
#include "booz_imu.h"

struct BoozAhrs booz_ahrs;
struct BoozAhrsFloat booz_ahrs_float;

#define RB_MAXN 64

struct Int32Vect3 accel_buf[RB_MAXN];
struct Int32Vect3 booz_ahrs_accel_mean;

uint8_t rb_r; /* pos to read from, oldest measurement */
uint8_t rb_w; /* pos to write to */
uint8_t rb_n; /* number of elements in rb */

void booz_ahrs_init_accel_rb(void) {
  rb_r = 0;
  rb_w = 0;
  rb_n = 0;
}

void booz_ahrs_store_accel(void) {
  VECT3_COPY(accel_buf[rb_w], booz_imu.accel);
  rb_w = (rb_w + 1) < RB_MAXN ? (rb_w + 1) : 0;

  /* once the buffer is full it always has the last RB_MAXN accel measurements */
  if (rb_n < RB_MAXN) {
	rb_n++;
  } else {
	rb_r = (rb_r + 1) < RB_MAXN ? (rb_r + 1) : 0;
  }
}

/* compute the mean of the last n accel measurements */
void booz_ahrs_compute_accel_mean(uint8_t n) {
  struct Int32Vect3 sum;
  int i, j;

  INT_VECT3_ZERO(sum);

  if (n > rb_n) {
	n = rb_n;
  }
  for (i = 1; i <= n; i++) {
	j = (rb_w - i) > 0 ? rb_w - i : rb_w - i + RB_MAXN;
	VECT3_ADD(sum, accel_buf[j]);
  }
  if (n > 1) {
	VECT3_SDIV(booz_ahrs_accel_mean, sum, n);
  } else {
	VECT3_COPY(booz_ahrs_accel_mean, sum);
  }
}

