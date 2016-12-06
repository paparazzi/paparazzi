/*
 * Copyright (C) 2016 Michal Podhradsky <michal.pohradsky@aggiemail.usu.edu>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file modules/ins/ins_lpe.h
 *
 * Local Position Estimator
 */
#ifndef INS_LPE_H
#define INS_LPE_H

#include "subsystems/ins.h"
#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, n_x};
enum {U_ax = 0, U_ay, U_az, n_u};




struct InsLidar {
  bool data_available;
  uint32_t timestamp;
  float agl;
  float min_distance;
  float max_distance;
  float offset;
};

struct InsSonar {
  bool data_available;
  uint32_t timestamp;
  float agl;
  float min_distance;
  float max_distance;
  float offset;
};

struct InsBaro {
  bool data_available;
  uint32_t timestamp;
  float baro_z;  ///< z-position calculated from baro in meters (z-down)
  float qfe;
  bool baro_initialized;
};

struct InsAccel {
  bool data_available;
  uint32_t timestamp;
  struct Int32Vect3 accel_meas_body;
  struct Int32Vect3 accel_meas_ltp;
};

struct InsGps {
  bool data_available;
  uint32_t timestamp;
  // TODO
};

struct InsOpticflow {
  bool data_available;
  uint32_t timestamp;
  struct FloatVect3 vel_body;
};


struct InsLpe {
  bool initialized;

  struct InsLidar lidar;
  struct InsSonar sonar;
  struct InsBaro baro;
  struct InsAccel accel;
  struct InsGps gps;
  struct InsOpticflow opticflow;

  // process matrix
  struct FloatMatrixMN A; // matrix struct
  float A_data[n_x][n_x]; // matrix data
  float *A_data_ptr[n_x]; // matrix pointer

  // input matrix
  struct FloatMatrixMN B; // matrix struct
  float B_data[n_x][n_u]; // matrix data
  float *B_data_ptr[n_x]; // matrix pointer

  // measurement noise covariance matrix
  struct FloatMatrixMN R; // matrix struct
  float R_data[n_u][n_u]; // matrix data
  float *R_data_ptr[n_u]; // matrix pointer

  // process noise covariance
  struct FloatMatrixMN Q; // matrix struct
  float Q_data[n_x][n_x]; // matrix data
  float *Q_data_ptr[n_x]; // matrix pointer

  // estimate error covariance
  struct FloatMatrixMN P; // matrix struct
  float P_data[n_x][n_x]; // matrix data
  float *P_data_ptr[n_x]; // matrix pointer

  float x[n_x]; // state vector
  float u[n_u]; // input vector

  // Params
  float accel_xy_stddev;
  float accel_z_stddev;
  float pn_p_noise_density;
  float pn_v_noise_density;
  float pn_b_noise_density;
};

extern struct InsLpe ins_lpe;

extern void ins_lpe_init(void);
extern void ins_lpe_periodic(void);

void ins_lpe_reset_states(void);
void ins_lpe_init_states(void);
void ins_lpe_update_states(void);
void ins_lpe_update_params(void);

#endif /* INS_LPE_H */
