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

/**
 * @file subsystems/ins/vf_float.h
 *
 * Vertical filter (in float) estimating altitude, velocity and accel bias.
 *
 */

#ifndef VF_FLOAT_H
#define VF_FLOAT_H

#define VFF_STATE_SIZE 3

struct Vff {
  float z;       ///< z-position estimate in m (NED, z-down)
  float zdot;    ///< z-velocity estimate in m/s (NED, z-down)
  float bias;    ///< accel bias estimate in m/s^2
  float zdotdot; ///< z-acceleration in m/s^2 (NED, z-down)
  float z_meas;  ///< last measurement
  float P[VFF_STATE_SIZE][VFF_STATE_SIZE];  ///< covariance matrix
};

extern struct Vff vff;

extern void vff_init_zero(void);
extern void vff_init(float z, float zdot, float bias);
extern void vff_propagate(float accel, float dt);
extern void vff_update(float z_meas);
extern void vff_update_z_conf(float z_meas, float conf);
extern void vff_update_vz_conf(float vz_meas, float conf);
extern void vff_realign(float z_meas);

#endif /* VF_FLOAT_H */
