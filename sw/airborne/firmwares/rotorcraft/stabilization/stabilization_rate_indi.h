/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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

#include "math/pprz_algebra_int.h"

extern void stabilization_rate_init(void);
extern void stabilization_rate_read_rc(void);
extern void stabilization_rate_read_rc_switched_sticks(void);
extern void stabilization_indi_rate_set_setpoint_i(struct Int32Rates *pqr);
extern void stabilization_indi_rate_run(bool in_flight, struct Int32Rates rates_sp);
extern void stabilization_indi_rate_enter(void);

extern struct FloatRates stabilization_rate_sp;

#endif /* STABILIZATION_RATE_INDI */
