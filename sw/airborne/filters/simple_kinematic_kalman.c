/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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

/** @file "filters/simple_kinematic_kalman.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * @author Antoine Leclerc, Nathan Puch, Pauline Molitor
 *
 * Basic kinematic kalman filter for tag tracking and constant speed
 */

#include "filters/simple_kinematic_kalman.h"
#include <math.h>

void simple_kinematic_kalman_init(struct SimpleKinematicKalman *kalman, float P0_pos, float P0_speed, float Q_sigma2,
                                  float r, float dt)
{
  int i, j;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt / 2.f;
  const float dt4 = dt2 * dt2 / 4.f;
  for (i = 0; i < SIMPLE_KINEMATIC_KALMAN_DIM; i++) {
    kalman->state[i] = 0.f; // don't forget to call set_state before running the filter for better results
    for (j = 0; j < SIMPLE_KINEMATIC_KALMAN_DIM; j++) {
      kalman->P[i][j] = 0.f;
      kalman->Q[i][j] = 0.f;
    }
  }
  for (i = 0; i < SIMPLE_KINEMATIC_KALMAN_DIM; i += 2) {
    kalman->P[i][i] = P0_pos;
    kalman->P[i + 1][i + 1] = P0_speed;
    kalman->Q[i][i] = Q_sigma2 * dt4;
    kalman->Q[i + 1][i] = Q_sigma2 * dt3;
    kalman->Q[i][i + 1] = Q_sigma2 * dt3;
    kalman->Q[i + 1][i + 1] = Q_sigma2 * dt2;
  }
  kalman->r = r;
  kalman->dt = dt;

  // kalman->F = {{1, dt, 0, 0 , 0, 0},
  //              {0, 1 , 0, 0,  0, 0},
  //              {0, 0,  1, dt 0, 0},
  //              {0, 0,  0, 1  0, 0},
  //              {0, 0,  0, 0,  1 ,dt}
  //              {0, 0,  0, 0,  0, 1 }};

  for (int i = 0; i < SIMPLE_KINEMATIC_KALMAN_DIM; i++) {
    for (int j = 0; j < SIMPLE_KINEMATIC_KALMAN_DIM; j++) {
      if (i == j) {
        kalman->F[i][i] = 1;
      } else if (i % 2 == 0 && j == i + 1) {
        kalman->F[i][j] = dt;
      } else {
        kalman->F[i][j] = 0;
      }
    }
  }

  // kalman->Hp = {{1,0,0,0,0,0},
  //               {0,0,1,0,0,0},
  //               {0,0,0,0,1,0}};

  for (int i = 0; i < SIMPLE_KINEMATIC_KALMAN_DIM / 2; i++) {
    for (int j = 0; j < SIMPLE_KINEMATIC_KALMAN_DIM; j++) {
      if (2 * i == j) {
        kalman->Hp[i][j] = 1.f;
      } else {
        kalman->Hp[i][j] = 0.f;
      }
      // horizontal and vertical speed can be updated separately, init matrix to zero
      kalman->Hs[i][j] = 0.f;
    }
  }
}

void simple_kinematic_kalman_set_state(struct SimpleKinematicKalman *kalman, struct FloatVect3 pos,
                                       struct FloatVect3 speed)
{
  kalman->state[0] = pos.x;
  kalman->state[1] = speed.x;
  kalman->state[2] = pos.y;
  kalman->state[3] = speed.y;
  kalman->state[4] = pos.z;
  kalman->state[5] = speed.z;
}

void simple_kinematic_kalman_get_state(struct SimpleKinematicKalman *kalman, struct FloatVect3 *pos,
                                       struct FloatVect3 *speed)
{
  pos->x = kalman->state[0];
  pos->y = kalman->state[2];
  pos->z = kalman->state[4];
  speed->x = kalman->state[1];
  speed->y = kalman->state[3];
  speed->z = kalman->state[5];
}

struct FloatVect3 simple_kinematic_kalman_get_pos(struct SimpleKinematicKalman *kalman)
{
  struct FloatVect3 pos;
  pos.x = kalman->state[0];
  pos.y = kalman->state[2];
  pos.z = kalman->state[4];
  return pos;
}

struct FloatVect3 simple_kinematic_kalman_get_speed(struct SimpleKinematicKalman *kalman)
{
  struct FloatVect3 speed;
  speed.x = kalman->state[1];
  speed.y = kalman->state[3];
  speed.z = kalman->state[5];
  return speed;
}

void simple_kinematic_kalman_update_noise(struct SimpleKinematicKalman *kalman, float Q_sigma2, float r)
{
  int i;
  const float dt = kalman->dt;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt / 2.f;
  const float dt4 = dt2 * dt2 / 4.f;
  for (i = 0; i < SIMPLE_KINEMATIC_KALMAN_DIM; i += 2) {
    kalman->Q[i][i] = Q_sigma2 * dt4;
    kalman->Q[i + 1][i] = Q_sigma2 * dt3;
    kalman->Q[i][i + 1] = Q_sigma2 * dt3;
    kalman->Q[i + 1][i + 1] = Q_sigma2 * dt2;
  }
  kalman->r = r;
}

/** propagate dynamic model
 *
 * F = [ 1 dt 0 0  0 0
 *       0 1  0 0  0 0
 *       0 0  1 dt 0 0
 *       0 0  0 1  0 0
 *       0 0  0 0  1 dt
 *       0 0  0 0  0 1  ]
 */
void simple_kinematic_kalman_predict(struct SimpleKinematicKalman *kalman)
{
  int i;
  for (i = 0; i < SIMPLE_KINEMATIC_KALMAN_DIM; i += 2) {
    // kinematic equation of the dynamic model X = F*X
    kalman->state[i] += kalman->state[i + 1] * kalman->dt;

    // propagate covariance P = F*P*Ft + Q
    // since F is diagonal by block, P can be updated by block here as well
    // let's unroll the matrix operations as it is simple

    const float d_dt = kalman->P[i + 1][i + 1] * kalman->dt;
    kalman->P[i][i] += kalman->P[i + 1][i] * kalman->dt + kalman->dt * (kalman->P[i][i + 1] + d_dt) + kalman->Q[i][i];
    kalman->P[i][i + 1] += d_dt + kalman->Q[i][i + 1];
    kalman->P[i + 1][i] += d_dt + kalman->Q[i + 1][i];
    kalman->P[i + 1][i + 1] += kalman->Q[i + 1][i + 1];
  }
}

/** generic correction step
 *
 * K = PHt(HPHt+R)^-1 = PHtS^-1
 * X = X + K(Z-HX)
 * P = (I-KH)P
 *
 * @param kalman pointer to kalman structure
 * @param[in] H pointer to observation matrix
 * @param[in] Z pointer to measurement vector
 */
static void simple_kinematic_kalman_update(struct SimpleKinematicKalman *kalman, float **_H, float *Z)
{
  float S[3][3] = {
    {kalman->P[0][0] + kalman->r, kalman->P[0][2],             kalman->P[0][4]},
    {kalman->P[2][0],             kalman->P[2][2] + kalman->r, kalman->P[2][4]},
    {kalman->P[4][0],             kalman->P[4][2],             kalman->P[4][4] + kalman->r}
  };

  if (fabsf(S[0][0]) + fabsf(S[1][1]) + fabsf(S[2][2]) < 1e-5) {
    return; // don't inverse S if it is too small
  }

  // prepare variables and pointers
  MAKE_MATRIX_PTR(_S, S, 3);
  float invS[3][3];
  MAKE_MATRIX_PTR(_invS, invS, 3);
  float HinvS_tmp[6][3];
  MAKE_MATRIX_PTR(_HinvS_tmp, HinvS_tmp, 6);
  float Ht[6][3];
  MAKE_MATRIX_PTR(_Ht, Ht, 6);
  float K[6][3];
  MAKE_MATRIX_PTR(_K, K, 6);
  float HX_tmp[3];
  float Z_HX[3];
  float K_ZHX_tmp[6];
  float KH_tmp[6][6];
  MAKE_MATRIX_PTR(_KH_tmp, KH_tmp, 6);
  MAKE_MATRIX_PTR(_P, kalman->P, 6);
  float P_tmp[6][6];

  // finally compute gain and correct state
  float_mat_invert(_invS, _S, 3);                       // S^-1
  float_mat_transpose(_Ht, _H, 3, 6);                   // Ht
  float_mat_mul(_HinvS_tmp, _Ht, _invS, 6, 3, 3);       // Ht * S^-1
  float_mat_mul(_K, _P, _HinvS_tmp, 6, 6, 3);           // P * Ht * S^-1
  float_mat_vect_mul(HX_tmp, _H, kalman->state, 3, 6);  // H * X
  float_vect_diff(Z_HX, Z, HX_tmp, 3);                  // Z - H * X
  float_mat_vect_mul(K_ZHX_tmp, _K, Z_HX, 6, 3);        // K * (Z - H * X)
  float_vect_add(kalman->state, K_ZHX_tmp, 6);          // X + K * (Z - H * X)

  // precompute K*H and store current P
  float_mat_mul(_KH_tmp, _K, _H, 6, 3, 6);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P_tmp[i][j] = kalman->P[i][j];
    }
  }
  // correct covariance P = (I-K*H)*P = P - K*H*P
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      for (int k = 0; k < 6; k++) {
        kalman->P[i][j] -= KH_tmp[i][k] * P_tmp[k][j];
      }
    }
  }
}

/** correction step for position
 *
 * K = P.Hpt(Hp.P.Hpt+R)^-1 = P.Hpt.S^-1
 * X = X + K(Z-Hp.X)
 * P = (I-K.Hp)P
 */
void simple_kinematic_kalman_update_pos(struct SimpleKinematicKalman *kalman, struct FloatVect3 pos)
{
  // measurement vector
  float Z[3] = { pos.x, pos.y, pos.z };
  // position observation matrix
  MAKE_MATRIX_PTR(_H, kalman->Hp, 3);

  // call update
  simple_kinematic_kalman_update(kalman, _H, Z);
}

/** correction step for speed
 *
 * K = P.Hst(Hs.P.Hst+R)^-1 = P.Hst.S^-1
 * X = X + K(Z-Hs.X)
 * P = (I-K.Hs)P
 */
void simple_kinematic_kalman_update_speed(struct SimpleKinematicKalman *kalman, struct FloatVect3 speed, uint8_t type)
{
  // measurement vector
  float Z[3] = { speed.x, speed.y, speed.z };
  if (type == SIMPLE_KINEMATIC_KALMAN_SPEED_HORIZONTAL) {
    // update horizontal speed
    kalman->Hs[0][1] = 1.f;
    kalman->Hs[1][3] = 1.f;
    kalman->Hs[2][5] = 0.f;
  } else if (type == SIMPLE_KINEMATIC_KALMAN_SPEED_VERTICAL) {
    // update vertical speed
    kalman->Hs[0][1] = 0.f;
    kalman->Hs[1][3] = 0.f;
    kalman->Hs[2][5] = 1.f;
  } else {
    // update 3D speed
    kalman->Hs[0][1] = 1.f;
    kalman->Hs[1][3] = 1.f;
    kalman->Hs[2][5] = 1.f;
  }
  // speed observation matrix
  MAKE_MATRIX_PTR(_H, kalman->Hs, 3);

  // call update
  simple_kinematic_kalman_update(kalman, _H, Z);
}

