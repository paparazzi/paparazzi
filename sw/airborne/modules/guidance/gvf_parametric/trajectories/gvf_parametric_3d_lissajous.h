/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_3d_lissajous.h
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 *
 * 3D lissajous figures
 */

#ifndef GVF_PARAMETRIC_3D_LISSAJOUS_H
#define GVF_PARAMETRIC_3D_LISSAJOUS_H

#ifdef __cplusplus
extern "C" {
#endif

/** @typedef gvf_3d_lis_par
* @brief Parameters for the GVF parametric 3D lissajous
* @param kx Gain defining how agressive is the vector field in x coordinate
* @param ky Gain defining how agressive is the vector field in y coordinate
* @param kz Gain defining how agressive is the vector field in z coordinate
* @param cx Amplitude of the trajectory in the X plane
* @param cy Amplitude of the trajectory in the Y plane
* @param cz Amplitude of the trajectory in the Z plane
* @param wx Frequency of the trajectory in the X plane
* @param wy Frequency of the trajectory in the Y plane
* @param wz Frequency of the trajectory in the Z plane
* @param dx Offphase of the trajectory in the X plane
* @param dy Offphase of the trajectory in the Y plane
* @param dz Offphase of the trajectory in the Z plane
* @param alpha Orientation/rotation of the trajectory in the XY plane
*/
typedef struct {
  float kx;
  float ky;
  float kz;
  float cx;
  float cy;
  float cz;
  float wx;
  float wy;
  float wz;
  float dx;
  float dy;
  float dz;
  float alpha;
} gvf_par_3d_lis_par;

extern gvf_par_3d_lis_par gvf_parametric_3d_lissajous_par;

extern void gvf_parametric_3d_lissajous_info(float *f1, float *f2, float *f3, float *f1d, float *f2d, float *f3d,
    float *f1dd, float *f2dd, float *f3dd);

#ifdef __cplusplus
}
#endif

#endif // GVF_PARAMETRIC_3D_LISSAJOUS_H
