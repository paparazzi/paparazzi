/*
 * $Id$
 *
 * Copyright (C) 2008-2010 The Paparazzi Team
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
#ifndef BOOZ_STABILISATION_ATTITUDE_REF_INT_H
#define BOOZ_STABILISATION_ATTITUDE_REF_INT_H

extern struct Int32Eulers stab_att_sp_vi_euler; /* vehicle interface */
extern struct Int32Eulers stab_att_sp_rc_euler; /* radio control     */
extern struct Int32Eulers stab_att_sp_euler;    /* sum of the above  */
extern struct Int32Quat   stab_att_sp_quat;
extern struct Int32Eulers stab_att_ref_euler;
extern struct Int32Quat   stab_att_ref_quat;
extern struct Int32Rates  stab_att_ref_rate;
extern struct Int32Rates  stab_att_ref_accel;

struct Int32RefModel {
  struct Int32Rates omega;
  struct Int32Rates zeta;
};

extern struct Int32RefModel stab_att_ref_model;

#endif /* BOOZ_STABILISATION_ATTITUDE_REF_INT_H */
