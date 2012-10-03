/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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

#ifndef VF_EXTENDED_FLOAT_H
#define VF_EXTENDED_FLOAT_H

#define VFF_STATE_SIZE 4

extern float vff_z;
extern float vff_zdot;
extern float vff_bias;
extern float vff_offset;
extern float vff_P[VFF_STATE_SIZE][VFF_STATE_SIZE];
extern float vff_zdotdot;

extern float vff_z_meas;
extern float vff_z_meas_baro;

extern void vff_init(float z, float zdot, float accel_bias, float baro_offset);
extern void vff_propagate(float accel);
extern void vff_update_baro(float z_meas);
extern void vff_update_alt(float z_meas);
extern void vff_update_offset(float offset);
extern void vff_update_baro_conf(float z_meas, float conf);
extern void vff_update_alt_conf(float z_meas, float conf);
//extern void vff_update_vz_conf(float vz_meas, float conf);
extern void vff_realign(float z_meas);

#endif /* VF_EXTENDED_FLOAT_H */
