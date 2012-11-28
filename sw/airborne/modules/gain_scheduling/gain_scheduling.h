
/*
 * Copyright (C) 2012 Pranay Sinha <psinha@transition-robotics.com>
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

/**
 * @file gain_scheduling.h
 *
 * Module that interpolates between gain sets, depending on the scheduling variable.
 */

#ifndef GAIN_SCHEDULING_H
#define GAINS_SCHEDULING_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "math/pprz_algebra_int.h"
#include "generated/airframe.h"
#include "std.h"

#ifndef NUMBER_OF_GAINSETS
#error You must define the number of gainsets to use this module!
#endif

extern struct Int32AttitudeGains gainlibrary[NUMBER_OF_GAINSETS];

/**
 * Initialises periodic loop;
 */
extern void gain_scheduling_init(void);

/**
 * Periodic function that interpolates between gain sets depending on the scheduling variable.
 * If the variable has not changed, keep the same gain set.
 */
extern void gain_scheduling_periodic(void);

void set_gainset(int gainset);

#endif  /* GAIN_SCHEDULING_H */

