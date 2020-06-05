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
 * @file modules/guidance/gvf_advanced/gvf_advanced.cpp
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 */

#include <iostream>
#include <Eigen/Dense>

#include "gvf_advanced.h"
#include "./trajectories/gvf_advanced_3d_ellipse.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "subsystems/navigation/common_nav.h"
#include "subsystems/datalink/telemetry.h"
#include "autopilot.h"

#ifdef __cplusplus
}
#endif

uint32_t t0 = 0; // We need it for calculting the time lapse delta_T
gvf_advanced_tra gvf_advanced_trajectory;

void gvf_advanced_init(void)
{

}

void gvf_advanced_control_2D(float, Eigen::Vector3f *, Eigen::Matrix3f *);
void gvf_advanced_control_2D(float ktheta, Eigen::Vector3f *Chi2d, Eigen::Matrix3f *J2d)
{

}

void gvf_advanced_control_3D(float, Eigen::Vector4f *, Eigen::Matrix4f *);
void gvf_advanced_control_3D(float ktheta, Eigen::Vector4f *Chi3d, Eigen::Matrix4f *J3d)
{
    float u_theta;
    Eigen::Matrix2f E;
    Eigen::Matrix<float, 2, 4> F;
    Eigen::Matrix<float, 2, 4> Fp;
    Eigen::Matrix4f G;
    Eigen::Matrix4f Gp;

    F << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    E << 0.0, -1.0,
         1.0,  0.0;

    G = F.transpose() * F;
    Fp = E * F;
    Gp = F.transpose() * E * F;

    u_theta = - 1/((*Chi3d).transpose() * G * *Chi3d);
}

// 3D ELLIPSE

void gvf_advanced_3d_ellipse_info(Eigen::Vector4f *, Eigen::Matrix4f *);
void gvf_advanced_3d_ellipse_info(Eigen::Vector4f *Chi3d, Eigen::Matrix4f *J3d)
{

}

bool gvf_advanced_3D_ellipse(float x, float y, float r, float zl, float zh, float alpha)
{
  Eigen::Vector4f Chi3d;
  Eigen::Matrix4f J3d;

  gvf_advanced_trajectory.type = ELLIPSE_3D;
  gvf_advanced_trajectory.p_advanced[0] = x;
  gvf_advanced_trajectory.p_advanced[1] = y;
  gvf_advanced_trajectory.p_advanced[2] = r;
  gvf_advanced_trajectory.p_advanced[3] = zl;
  gvf_advanced_trajectory.p_advanced[4] = zh;
  gvf_advanced_trajectory.p_advanced[5] = alpha;

  // SAFE MODE
  if (zl > zh)
    zl = zh;
  if (zl < 1 || zh < 1){
    zl = 10;
    zh = 10;
  }
  if (r < 1)
    r = 60;

  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - t0;
  t0 = now;

  if(delta_T > 300) // We need at least two iterations for Delta_T
    return true;

  gvf_advanced_3d_ellipse_info(&Chi3d, &J3d);
  gvf_advanced_control_3D(gvf_advanced_3d_ellipse_par.ktheta, &Chi3d, &J3d);

  return true;
}
