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

/**
 * @file modules/ins/vf_extended_float.h
 *
 * Interface for extended vertical filter (in float).
 *
 */

#ifndef VF_EXTENDED_FLOAT_H
#define VF_EXTENDED_FLOAT_H

#define VFF_STATE_SIZE 5

struct VffExtended {
  /* state vector */
  float z;           ///< z-position estimate in m (NED, z-down)
  float zdot;        ///< z-velocity estimate in m/s (NED, z-down)
  float bias;        ///< accel bias estimate in m/s^2
  float offset;      ///< baro offset estimate
  float obs_height;  ///< estimate of height of obstacles under the vehicle

  float zdotdot;     ///< z-acceleration in m/s^2 (NED, z-down)
  float z_meas;      ///< last z measurement in m
  float z_meas_baro; ///< last z measurement from baro in m

  float P[VFF_STATE_SIZE][VFF_STATE_SIZE];  ///< covariance matrix

  float accel_noise;
  float r_baro;
  float r_alt;
  float r_obs_height;
};

extern struct VffExtended vff;

extern void vff_init_zero(void);
extern void vff_init(float z, float zdot, float accel_bias, float baro_offset, float obstacle_hieght);
extern void vff_propagate(float accel, float dt);
extern void vff_update_baro(float z_meas);
extern void vff_update_z(float z_meas);
extern void vff_update_baro_conf(float z_meas, float conf);
extern void vff_update_z_conf(float z_meas, float conf);
extern void vff_update_vz_conf(float vz_meas, float conf);
extern void vff_realign(float z_meas);
extern void vff_update_agl(float z_meas, float conf);

#endif /* VF_EXTENDED_FLOAT_H */
