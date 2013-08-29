/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file stabilization_rate.h
 *  Rate stabilization for rotorcrafts.
 *
 *  Control loops for angular velocity.
 */

#ifndef STABILIZATION_RATE
#define STABILIZATION_RATE

#include "math/pprz_algebra_int.h"

extern void stabilization_rate_init(void);
extern void stabilization_rate_read_rc(void);
extern void stabilization_rate_read_rc_switched_sticks(void);
extern void stabilization_rate_run(bool_t in_flight);
extern void stabilization_rate_enter(void);

extern struct Int32Rates stabilization_rate_sp;
extern struct Int32Rates stabilization_rate_gain;
extern struct Int32Rates stabilization_rate_igain;
extern struct Int32Rates stabilization_rate_ddgain;
extern struct Int32Rates stabilization_rate_ref;
extern struct Int32Rates stabilization_rate_refdot;
extern struct Int32Rates stabilization_rate_sum_err;

extern struct Int32Rates stabilization_rate_fb_cmd;
extern struct Int32Rates stabilization_rate_ff_cmd;

#endif /* STABILIZATION_RATE */
