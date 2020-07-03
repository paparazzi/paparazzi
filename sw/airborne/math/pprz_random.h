/*
 * Copyright (C) 2017-2018 Joost Meulenbeld
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
 *
 * Functions for getting random numbers. The rand_gaussian() internally uses the rand_uniform().
 * rand_uniform() uses rand() internally which is initialized with the current time on rand_init().
 * This means that the board doesn't need an internal rng but comes at the cost of more computations.
 *
 * Usage:
 * rand_init(); // initialize once
 *
 * float random_number = rand_uniform();
 */

#ifndef RANDOM_H
#define RANDOM_H

#include <std.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

// Initialize the random number generator (call this before using the other functions)
void init_random(void);

// Random number from uniform[0,1] distribution
double rand_uniform(void);

// Random number from gaussian(0, 1) distribution
double rand_gaussian(void);

#endif // RANDOM_H
