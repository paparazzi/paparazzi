/*
 * Copyright (C) 2011-2012 The Paparazzi Team
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

/** @file stabilization_none.h
 *  Dummy stabilization for rotorcrafts.
 *
 *  Doesn't actually do any stabilization,
 *  just directly passes the RC commands along.
 */

#ifndef STABILIZATION_ONELOOP
#define STABILIZATION_ONELOOP

#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

extern struct FloatEulers stab_att_sp_euler;
extern struct Int32Quat   stab_att_sp_quat;
extern struct FloatRates  stab_att_ff_rates;


extern struct Int32Rates stabilization_oneloop_rc_cmd;


#endif /* STABILIZATION_NONE */
