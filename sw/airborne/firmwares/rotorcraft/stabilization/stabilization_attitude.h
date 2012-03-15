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

#ifndef STABILIZATION_ATTITUDE_H
#define STABILIZATION_ATTITUDE_H


#include STABILISATION_ATTITUDE_H
extern void stabilization_attitude_init(void);
extern void stabilization_attitude_read_rc(bool_t in_flight);
extern void stabilization_attitude_enter(void);
extern void stabilization_attitude_run(bool_t  in_flight);

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref.h"
#include STABILISATION_ATTITUDE_REF_H
extern void stabilization_attitude_ref_init(void);
extern void stabilization_attitude_ref_update(void);

#define stabilization_attitude_SetKiPhi(_val) {	\
    stabilization_gains.i.x = _val;             \
    stabilization_att_sum_err.phi = 0;          \
  }

#endif /* STABILIZATION_ATTITUDE_H */
