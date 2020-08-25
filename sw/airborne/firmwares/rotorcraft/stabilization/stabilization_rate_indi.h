/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2020 Rohan Chotalal 
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

/** @file stabilization_rate_indi.h
 *  Rate stabilization for rotorcrafts based on INDI.
 */

#ifndef STABILIZATION_RATE_INDI
#define STABILIZATION_RATE_INDI

/* access declarations of basic functions from "stabilization_rate.h" file */
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"

#include "math/pprz_algebra_int.h"

extern struct FloatRates stabilization_rate_sp;

extern void stabilization_rate_indi_cmd(bool in_flight, struct Int32Rates rate_sp);
extern void stabilization_rate_indi_run(bool in_flight, struct Int32Rates rates_sp);
extern void stabilization_rate_indi_enter(void);

#endif /* STABILIZATION_RATE_INDI */
