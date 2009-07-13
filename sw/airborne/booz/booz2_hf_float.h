/*
 * $Id$
 *  
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

#ifndef BOOZ2_HF_FLOAT_H
#define BOOZ2_HF_FLOAT_H

#include "pprz_algebra_float.h"
#include "pprz_algebra_int.h"

extern struct Int32Vect3 b2ins_accel_bias;
#define B2INS_ACCEL_BIAS_FRAC 19
extern struct Int32Vect3 b2ins_accel_ltp;
#define B2INS_ACCEL_LTP_FRAC 10
extern struct Int32Vect3 b2ins_speed_ltp;
#define B2INS_SPEED_LTP_FRAC 19
extern struct Int64Vect3 b2ins_pos_ltp;
#define B2INS_POS_LTP_FRAC   28

extern struct Int32Vect3  b2ins_meas_gps_pos_ned;
extern struct Int32Vect3  b2ins_meas_gps_speed_ned;

extern void b2ins_init(void);
extern void b2ins_propagate(void);
extern void b2ins_update_gps(void);

#endif /* BOOZ2_HF_FLOAT_H */
