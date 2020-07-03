/*
 * Copyright (C) Joost Meulenbeld
 *
 * This file is part of paparazzi
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
 * @file "math/pprz_random.c"
 * @author Joost Meulenbeld
 * Random number functions
 */

#include "pprz_random.h"

#ifdef BOARD_CONFIG
#include "mcu_periph/sys_time.h"
#else
#include <time.h>
#endif


void init_random(void)
{
#ifdef BOARD_CONFIG
  srand(get_sys_time_msec());
#else
  srand(time(NULL));
#endif

}

double rand_uniform(void)
{
  return (double) rand() / RAND_MAX;
}

/*
 * http://www.taygeta.com/random/gaussian.html
 */
double rand_gaussian(void)
{
  static int nb_call = 0;
  static double x2;
  static double w;
  double x1;

  nb_call++;
  if (nb_call % 2) {
    do {
      x1 = 2.0 * rand_uniform() - 1.0;
      x2 = 2.0 * rand_uniform() - 1.0;
      w = x1 * x1 + x2 * x2;
    } while (w >= 1.0 || w == 0.0);

    w = sqrt((-2.0 * log(w)) / w);
    return x1 * w;
  } else {
    return x2 * w;
  }
}
