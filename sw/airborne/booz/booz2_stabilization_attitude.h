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

#ifndef BOOZ2_STABILIZATION_ATTITUDE_H
#define BOOZ2_STABILIZATION_ATTITUDE_H

#include "math/pprz_algebra_int.h"

#include "airframe.h"

extern void booz2_stabilization_attitude_init(void);
extern void booz2_stabilization_attitude_read_rc(bool_t in_flight);
extern void booz2_stabilization_attitude_enter(void);
extern void booz2_stabilization_attitude_run(bool_t  in_flight);


//extern struct booz_ieuler booz_stabilization_att_sp;
//extern struct booz_ieuler booz_stabilization_att_ref;
//extern struct booz_ivect  booz_stabilization_rate_ref;
//extern struct booz_ivect  booz_stabilization_accel_ref;
#include "booz2_stabilization_attitude_ref_traj_euler.h"


extern struct Int32Vect3  booz_stabilization_igain;
extern struct Int32Vect3  booz_stabilization_pgain;
extern struct Int32Vect3  booz_stabilization_dgain;
extern struct Int32Vect3  booz_stabilization_ddgain;
extern struct Int32Eulers booz_stabilization_att_sum_err;

extern int32_t booz2_stabilization_att_err_cmd[COMMANDS_NB];

#endif /* BOOZ2_STABILIZATION_ATTITUDE_H */
