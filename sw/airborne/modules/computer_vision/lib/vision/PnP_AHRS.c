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
#include <math.h>
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"
#include "peripherals/video_device.h"
#include "state.h"
#include "modules/computer_vision/lib/vision/undistortion.h"

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
  float_rmat_of_eulers_321(&R_C_B,&cam_body);

  // Use the AHRS roll and pitch from the filter to get the attitude in a rotation matrix R_E_B:
  struct FloatEulers attitude;
  attitude.phi =    stateGetNedToBodyEulers_f()->phi;//positive ccw
  attitude.theta = stateGetNedToBodyEulers_f()->theta;//negative downward
  // local_psi typically assumed 0.0f (meaning you are straight in front of the object):
  float local_psi = 0.0f;
  attitude.psi = local_psi;
  struct FloatRMat R_E_B, R_B_E;
  float_rmat_of_eulers_321(&R_E_B,&attitude);
  MAT33_TRANS(R_B_E, R_E_B);

  // Camera calibration matrix:
  float K[9] = {cam_intrinsics.focal_x, 0.0f, cam_intrinsics.center_x,
                  0.0f, cam_intrinsics.focal_y, cam_intrinsics.center_y,
                  0.0f, 0.0f, 1.0f
                 };

  // vectors in world coordinates, that will be "attached" to the world coordinates for the PnP:
  struct FloatVect3 gate_vectors[4], vec_B;

  // Equivalent MATLAB code:
  //
  // for i = 1:4
  //    R = R + (eye(3,3)-n(:,i)*n(:,i)');
  //    q = q + (eye(3,3)-n(:,i)*n(:,i)')*a(:,i);
  //
  // where n is the normalized vector, and a is the world coordinate


  // Construct the matrix for linear least squares:
  for(int i = 0; i < n_corners; i++) {

    // undistort the image coordinate and put it in a world vector:
    float x_n, y_n;
    bool success = distorted_pixels_to_normalized_coords((float) x_corners[i], (float) y_corners[i], &x_n, &y_n, cam_intrinsics.k, K);
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
    vec_e.z = -vec_e.z;

    struct FloatRMat temp_mat;
    VECT3_VECT3_TRANS_MUL(temp_mat, vec_E, vec_E); // n(:,i)*n(:,i)'
    RMAT_DIFF(temp_mat, I_mat, temp_mat); // MAT33_MAT33_DIFF(temp_mat, I_mat, temp_mat);
    MAT33_COPY(temp_mat_2, Q_mat);
    MAT33_MAT33_SUM(Q_mat, temp_mat_2, temp_mat);
    MAT33_VECT3_MUL(temp_vec, temp_mat, world_corners[i]);
    VECT3_SUM(p_vec, p_vec, temp_vec);



  }

}


struct FloatVect3 vec_point_1, vec_point_2, vec_point_3, vec_point_4, vec_temp1, vec_temp2, vec_temp3, vec_temp4,
vec_ver_1, vec_ver_2, vec_ver_3, vec_ver_4;
struct FloatVect3 gate_vectors[4];
struct FloatVect3 world_corners[4];
struct FloatVect3 p3p_pos_solution;

gate_center_height = -3.5;//

    // Construct the real gate coordinates:
          VECT3_ASSIGN(world_corners[0], gate_dist_x,-(gate_size_m/2), gate_center_height-(gate_size_m/2));
    VECT3_ASSIGN(world_corners[1], gate_dist_x, (gate_size_m/2), gate_center_height-(gate_size_m/2));
    VECT3_ASSIGN(world_corners[2], gate_dist_x, (gate_size_m/2), gate_center_height+(gate_size_m/2));
    VECT3_ASSIGN(world_corners[3], gate_dist_x,-(gate_size_m/2), gate_center_height+(gate_size_m/2));


  //Undistort fisheye points
  float k_fisheye = 1.085;
  float x_gate_corners[4];
  float y_gate_corners[4];

  struct FloatRMat R,R_C_B,R_trans,Q_mat,I_mat, temp_mat, temp_mat_2;
  struct FloatEulers attitude, cam_body;
  struct FloatVect3 vec_20, p_vec,pos_vec,temp_vec,vec_e;

  p_vec.x = 0;
  p_vec.y = 0;
  p_vec.z = 0;


  // Empty Q(?) matrix:


  p_vec.x = 0;
  p_vec.y = 0;
  p_vec.z = 0;

  // get a rotation matrix for the attitude:
  float_rmat_of_eulers_321(&R,&attitude);

  //for(int i = 0;i<4;i++)
  for(int i = 1;i<4;i++) // Only three points? In principle enough for P3P... but does it not help to use 4 points?
  {
    float undist_x, undist_y;
    //debug_1 = (float)best_gate.x_corners[i];// best_gate.y_corners[i]
    //debug_2 = (float)best_gate.y_corners[i];//
    //undist_y = 5;//(float)best_gate.y_corners[i];
    float princ_x = 157.0;
    float princ_y = 32.0;
    undistort_fisheye_point(best_gate.x_corners[i] ,best_gate.y_corners[i],&undist_x,&undist_y,f_fisheye,k_fisheye,princ_x,princ_y);

    x_gate_corners[i] = undist_x+157.0;
    y_gate_corners[i] = undist_y+32.0;

    if(gate_graphics)draw_cross(img,((int)x_gate_corners[i]),((int)y_gate_corners[i]),green_color);
    //if(gate_graphics)draw_cross(img,((int)x_trail[i])+157,((int)y_trail[i])+32,green_color);
    vec_from_point_ned(undist_x, undist_y, f_fisheye,&gate_vectors[i]);

    //Least squares stuff here:
    vec_from_point_2(undist_x,undist_y,168,&temp_vec);
    //vec_from_point_2(x_trail[i],y_trail[i],168,&temp_vec);

    //camera to body rotation
    cam_body.phi = 0;
    cam_body.theta = -25*(3.14/180);
    cam_body.psi = 0;
    float_rmat_of_eulers_321(&R_C_B,&cam_body);
    MAT33_VECT3_MUL(vec_20, R_C_B,temp_vec);


    MAT33_TRANS(R_trans,R);
    MAT33_VECT3_MUL(vec_e, R_trans, vec_20);
    //print_vector(vec_e);

    //to ned
    vec_e.z = -vec_e.z;

    double vec_norm = sqrt(VECT3_NORM2(vec_e));
    VECT3_SDIV(vec_e, vec_e, vec_norm);
    VECT3_VECT3_TRANS_MUL(temp_mat, vec_e,vec_e);
    MAT33_MAT33_DIFF(temp_mat,I_mat,temp_mat);
    MAT33_COPY(temp_mat_2,Q_mat);
    MAT33_MAT33_SUM(Q_mat,temp_mat_2,temp_mat);
    MAT33_VECT3_MUL(temp_vec, temp_mat, world_corners[i]);
    VECT3_SUM(p_vec,p_vec,temp_vec);

//    for i = 1:4
//    R = R + (eye(3,3)-n(:,i)*n(:,i)');
//    q = q + (eye(3,3)-n(:,i)*n(:,i)')*a(:,i);
//
//    p = R\q;
//    or q = Rp
//        hence p = R_inv*q


  }


  MAT33_INV(temp_mat,Q_mat);

  MAT33_VECT3_MUL(pos_vec, temp_mat,p_vec);

  ls_pos_x = pos_vec.x;

  //bound y to remove outliers
  float y_threshold = 4;//2.5;
  if(pos_vec.y > y_threshold)pos_vec.y = y_threshold;
  if(pos_vec.y < -y_threshold)pos_vec.y = -y_threshold;


  ls_pos_y = pos_vec.y;//// minus for simulating open gate ! ---------------------------------------------------------------------
  //if(stateGetNedToBodyEulers_f()->psi > 1.6 || stateGetNedToBodyEulers_f()->psi < -1.6)ls_pos_y-=0.25;

  ls_pos_z = pos_vec.z;
