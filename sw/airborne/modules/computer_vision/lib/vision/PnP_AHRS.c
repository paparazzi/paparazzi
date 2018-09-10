/*
 * Copyright (C) Guido de Croon, 2018
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/lib/vision/PnP_AHRS.c
 * @brief Functions for solving a perspective-n-point problem, using the AHRS to get the relevant angles.
 *
 * Solves a perspective-n-point problem while using the attitude estimates of the AHRS. Using the AHRS makes the solution more robust
 * than a purely maximal likelihood estimate from a standard PnP solver.
 *
 * The module needs to get information on:
 * 1. The real-world coordinates of the points.
 * 2. The image coordinates of the points.
 * 3. The camera parameters for undistortion purposes.
 * 4. The rotation of the camera to the drone body.
 *
 */

// Own Header
#include "PnP_AHRS.h"
#include "state.h"

// TODO: in snake_gate_detection.c: add a function that sorts the corners, so that they will always correspond to, e.g., top-left CW
/**
 * Get the world position of the camera, given image coordinates and corresponding world coordinates.
 *
 * @param[in] x_corners Image x-coordinates of the corners.
 * @param[in] y_corners Image y-coordinates of the corners.
 * @param[in] world_corners World coordinates of the corners. The order has to be the same as for the image coordinates.
 * @param[in] n_corners Number of corners.
 * @param[in] cam_intrinsics The camera intrinsics necessary for doing undistortion.
 * @param[in] cam_body The rotation from camera to body.
 * @param[out] A single struct FloatVect3 representing the camera world position.
 */
struct FloatVect3 get_world_position_from_image_points(int* x_corners, int* y_corners, struct FloatVect3* world_corners, int n_corners,
                                          struct camera_intrinsics_t cam_intrinsics, struct FloatEulers cam_body) {

  // legend:
  // E = Earth
  // B = body
  // C = camera

  struct FloatVect3 pos_drone_E_vec;

  // Make an identity matrix:
  static struct FloatRMat I_mat;
  MAT33_ELMT(I_mat, 0, 0) = 1;//(row,column)
  MAT33_ELMT(I_mat, 0, 1) = 0;
  MAT33_ELMT(I_mat, 0, 2) = 0;
  MAT33_ELMT(I_mat, 1, 0) = 0;//(row,column)
  MAT33_ELMT(I_mat, 1, 1) = 1;
  MAT33_ELMT(I_mat, 1, 2) = 0;
  MAT33_ELMT(I_mat, 2, 0) = 0;//(row,column)
  MAT33_ELMT(I_mat, 2, 1) = 0;
  MAT33_ELMT(I_mat, 2, 2) = 1;

  static struct FloatRMat Q_mat;
  MAT33_ELMT(Q_mat, 0, 0) = 0;//(row,column)
  MAT33_ELMT(Q_mat, 0, 1) = 0;
  MAT33_ELMT(Q_mat, 0, 2) = 0;
  MAT33_ELMT(Q_mat, 1, 0) = 0;//(row,column)
  MAT33_ELMT(Q_mat, 1, 1) = 0;
  MAT33_ELMT(Q_mat, 1, 2) = 0;
  MAT33_ELMT(Q_mat, 2, 0) = 0;//(row,column)
  MAT33_ELMT(Q_mat, 2, 1) = 0;
  MAT33_ELMT(Q_mat, 2, 2) = 0;

  //camera to body rotation
  struct FloatRMat R_E_B, R_B_E, R_C_B;
  float_rmat_of_eulers_321(&R_C_B,&cam_body);

  // Use the AHRS roll and pitch from the filter to get the attitude in a rotation matrix R_E_B:
  struct FloatEulers attitude;
  attitude.phi =    stateGetNedToBodyEulers_f()->phi;//positive ccw
  attitude.theta = stateGetNedToBodyEulers_f()->theta;//negative downward
  // local_psi typically assumed 0.0f (meaning you are straight in front of the object):
  float local_psi = 0.0f;
  attitude.psi = local_psi;
  float_rmat_of_eulers_321(&R_E_B,&attitude);
  MAT33_TRANS(R_B_E, R_E_B);

  // Camera calibration matrix:
  float K[9] = {cam_intrinsics.focal_x, 0.0f, cam_intrinsics.center_x,
                  0.0f, cam_intrinsics.focal_y, cam_intrinsics.center_y,
                  0.0f, 0.0f, 1.0f
                 };

  // vectors in world coordinates, that will be "attached" to the world coordinates for the PnP:
  struct FloatVect3 gate_vectors[4], vec_B, vec_E, p_vec, temp_vec;
  p_vec.x = 0;
  p_vec.y = 0;
  p_vec.z = 0;

  // Equivalent MATLAB code:
  //
  // for i = 1:4
  //    Q = Q + (eye(3,3)-n(:,i)*n(:,i)');
  //    p = p + (eye(3,3)-n(:,i)*n(:,i)')*a(:,i);
  //
  // where n is the normalized vector, and a is the world coordinate

  struct FloatRMat temp_mat, temp_mat_2;
  MAT33_ELMT(temp_mat, 0, 0) = 0;//(row,column)
  MAT33_ELMT(temp_mat, 0, 1) = 0;
  MAT33_ELMT(temp_mat, 0, 2) = 0;
  MAT33_ELMT(temp_mat, 1, 0) = 0;//(row,column)
  MAT33_ELMT(temp_mat, 1, 1) = 0;
  MAT33_ELMT(temp_mat, 1, 2) = 0;
  MAT33_ELMT(temp_mat, 2, 0) = 0;//(row,column)
  MAT33_ELMT(temp_mat, 2, 1) = 0;
  MAT33_ELMT(temp_mat, 2, 2) = 0;

  // Construct the matrix for linear least squares:
  for(int i = 0; i < n_corners; i++) {

    // undistort the image coordinate and put it in a world vector:
    float x_n, y_n;
    // TODO: use the boolean that is returned to take action if undistortion is not possible.
    distorted_pixels_to_normalized_coords((float) x_corners[i], (float) y_corners[i], &x_n, &y_n, cam_intrinsics.Dhane_k, K);
    gate_vectors[i].x = 1.0;
    gate_vectors[i].y = x_n;
    gate_vectors[i].z = y_n;

    // transform the vector to the gate corner to earth coordinates:
    MAT33_VECT3_MUL(vec_B, R_C_B, gate_vectors[i]);
    MAT33_VECT3_MUL(vec_E, R_B_E, vec_B);
    double vec_norm = sqrt(VECT3_NORM2(vec_E));
    VECT3_SDIV(vec_E, vec_E, vec_norm);

    // TODO: check this - should we not do this in a nicer way?
    // to ned
    vec_E.z = -vec_E.z;

    VECT3_VECT3_TRANS_MUL(temp_mat, vec_E, vec_E); // n(:,i)*n(:,i)'
    MAT33_MAT33_DIFF(temp_mat, I_mat, temp_mat); // (eye(3,3)-n(:,i)*n(:,i)')
    MAT33_COPY(temp_mat_2, Q_mat);
    MAT33_MAT33_SUM(Q_mat, temp_mat_2, temp_mat); // Q + (eye(3,3)-n(:,i)*n(:,i)')
    MAT33_VECT3_MUL(temp_vec, temp_mat, world_corners[i]); // (eye(3,3)-n(:,i)*n(:,i)')*a(:,i)
    VECT3_SUM(p_vec, p_vec, temp_vec); // p + (eye(3,3)-n(:,i)*n(:,i)')*a(:,i);
  }

  // Solve the linear system:
  MAT33_INV(temp_mat, Q_mat);
  MAT33_VECT3_MUL(pos_drone_E_vec, temp_mat,p_vec); // position = inv(Q) * p

  //bound y to remove outliers
  float y_threshold = 4;
  if(pos_drone_E_vec.y > y_threshold) pos_drone_E_vec.y = y_threshold;
  else if(pos_drone_E_vec.y < -y_threshold) pos_drone_E_vec.y = -y_threshold;

  return pos_drone_E_vec;
}
