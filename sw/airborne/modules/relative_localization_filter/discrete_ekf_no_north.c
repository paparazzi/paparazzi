/*
 * Copyright (C) Mario Coppola
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
/**
 * @file "modules/relativelocalizationfilter/discrete_ekf_no_north.c"
 * @author Steven van der Helm, Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization
 */

#include "discrete_ekf.h"
#include "math/pprz_algebra_float.h"
#include <math.h>
#include <stdio.h> // needed for the printf statements

void extractPhiGamma(int n_row, int n_colA, int n_colB, float *inmat, float *phi, float *gamma){
  int totalsize = n_row+n_colB;
  for(int row = 0; row<totalsize;row++){
    for (int col = 0; col<totalsize;col++){
      if(row<n_row && col<n_colA){
        phi[col+row*n_colA] = *inmat;
      }
      else if(row<n_row && col>=n_colA){
        gamma[(col-n_colA)+row*n_colB] = *inmat;
      }
      inmat++;
    }
  }
}

void combineMatrices(int n_row, int n_colA, int n_colB, float *A, float *B, float *combmat){
  int totalsize = n_row+n_colB;
  for (int row =0; row<totalsize; row++){
    for (int col = 0; col<totalsize;col++){
      if ((row<n_row) && col<n_colA){
        combmat[col+row*totalsize]=*A;
        A++;
      }
      else if ((row<n_row) && (col>=n_colA) ){
        combmat[col+row*totalsize]= *B;
        B++;
      }
      else{
        combmat[col+row*totalsize]=0.0;
      }
    }
  }
}

void fmat_expm(int n, float *a, float *expa){
  float a_norm, c, t;
  const int q = 6;
  float d[n*n];
  float x[n*n];
  float a2[n*n];
  int ee, k, s;
  bool p;

  fmat_copy ( n, n, a, a2 );
  a_norm = fmat_norm_li ( n, n, a2 );
  ee = ( int ) ( fmat_log_2 ( a_norm ) ) + 1;
  s = fmat_max_i( 0, ee + 1 );
  t = 1.0 / pow ( 2.0, s );
  fmat_scal_mult( n, n, a2, t, a2 );
  fmat_copy ( n, n, a2, x );
  c = 0.5;
  fmat_make_identity ( expa, n);
  fmat_add_scal_mult ( n, n, expa, expa, c, a2);
  fmat_make_identity ( d, n);
  fmat_add_scal_mult ( n, n, d, d, -c, a2);
  
  p = true;
  for ( k = 2; k <= q; k++ ) {
    c = c * ( float ) ( q - k + 1 ) / ( float ) ( k * ( 2 * q - k + 1 ) );
    fmat_mult_cop(n, n, n, x, x, a2);
    fmat_add_scal_mult(n,n,expa,expa,c,x);

    if (p) {
      fmat_add_scal_mult(n,n,d,d,c,x);
    }
    else {
      fmat_add_scal_mult(n,n,d,d,-c,x);
    }
    p = !p;
  }

  /*
    E -> inverse(D) * E
  */
  fmat_invmult(n,n,d,expa,expa);

  /*
    E -> E^(2*S)
  */
  for ( k = 1; k <= s; k++ ) {
    fmat_mult_cop(n,n,n,expa,expa,expa);
  }

}

enum ekf_statein{x12,y12,z1,z2,u1,v1,u2,v2,gam};
enum ekf_input{u1dm,v1dm,u2dm,v2dm,r1m,r2m};

void c2d(int n_row, int n_colA, int n_colB, float *A, float *B, float dt, float *phi, float *gamma){
  int totalsize = n_row + n_colB;
  float combmat[totalsize][totalsize];
  float expm[totalsize][totalsize];

  float_mat_scale();
  float_mat_scale();

  // fmat_scal_mult(n_row,n_colA,A,dt,A);
  // fmat_scal_mult(n_row,n_colB,B,dt,B);
  combineMatrices(n_row, n_colA, n_colB, A, B, combmat);
  fmat_expm(totalsize, combmat, expm);
  extractPhiGamma(n_row, n_colA, n_colB, expm, phi, gamma);
}

/*
 * Continuous time state transition equation
 * state is: {x_rel,y_rel,h1,h2,u1,v1,u2,v2,gamma}
 */
void discrete_ekf_no_north_fsym(float *statein, float *input, float *output){
  output[0] =  input[r1m]*statein[y12] - statein[u1] + statein[u2] * cos(statein[gam]) - statein[v2] * sin(statein[gam]);
  output[1] = -input[r1m]*statein[x12] - statein[v1] + statein[u2] * sin(statein[gam]) + statein[v2] * cos(statein[gam]);
  output[2] = 0;
  output[3] = 0;
  output[4] = input[u1dm] + input[r1m] * statein[v1];
  output[5] = input[v1dm] - input[r1m] * statein[u1];
  output[6] = input[u2dm] + input[r2m] * statein[v2];
  output[7] = input[v2dm] - input[r2m] * statein[u2];
  output[8] = input[r2m] - input[r1m];
}

/*
 * Measurement equation, measures range, height1, height2, u1, v1, u2, v2
 */
void discrete_ekf_no_north_hsym(float *statein,float *output){
  output[0] = pow(pow((statein[z1]-statein[z2]),2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5);
  output[1] = statein[z1];
  output[2] = statein[z2];
  output[3] = statein[u1];
  output[4] = statein[v1];
  output[5] = statein[u2];
  output[6] = statein[v2];
}

void discrete_ekf_no_north_Fx(float *statein,float *input,float **output){
  fmat_make_zeros(output,EKF_N,EKF_N);
  output[0][1] = input[r1m];
  output[0][4] = -1;
  output[0][6] = cos(statein[gam]);
  output[0][7] = -sin(statein[gam]);
  output[0][8] = -statein[v2] * cos(statein[gam]) - statein[u2] * sin(statein[gam]);
  output[1][0] = -input[r1m];
  output[1][5] = -1;
  output[1][6] = sin(statein[gam]);
  output[1][7] = cos(statein[gam]);
  output[1][8] = statein[u2]*cos(statein[gam]) - statein[v2] * sin(statein[gam]);
  output[4][5] = input[r1m];
  output[5][4] = -input[r1m];
  output[6][7] = input[r2m];
  output[7][6] = -input[r2m];
}

void discrete_ekf_no_north_G(float *statein, float **output){
  fmat_make_zeros(output,EKF_N,EKF_L);
  output[0][4] = statein[y12];
  output[1][4] = -statein[x12];
  output[4][0] = 1;
  output[4][4] = statein[v1];
  output[5][1] = 1;
  output[5][4] = -statein[u1];
  output[6][2] = 1;
  output[6][4] = statein[v2];
  output[7][3] = 1;
  output[7][4] = -statein[u2]
  output[8][4] = -1;
  output[8][5] = 1;
}

void discrete_ekf_no_north_Hx(float *statein, float **output){
  fmat_make_zeros(output,EKF_M,EKF_N);
  output[0][0] = output,statein[x12]/(pow(pow(statein[z1]-statein[z2],2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5));
  output[0][1] = output,statein[y12]/(pow(pow(statein[z1]-statein[z2],2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5));
  output[0][2] = output,(2*statein[z1]-2*statein[z2])/(2*pow(pow(statein[z1]-statein[z2],2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5));
  output[0][3] = -(2*statein[z1]-2*statein[z2])/(2*pow(pow(statein[z1]-statein[z2],2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5));
  output[1][2] = 1;
  output[2][3] = 1;
  output[3][4] = 1;
  output[4][5] = 1;
  output[5][6] = 1;
  output[6][7] = 1;
}

void discrete_ekf_no_north_new(struct discrete_ekf *filter)
{
  MAKE_MATRIX_PTR(_P, filter->P, EKF_N);
  MAKE_MATRIX_PTR(_Q, filter->Q, EKF_N);
  MAKE_MATRIX_PTR(_R, filter->R, EKF_M);
  
  float_mat_diagonal_scal(_P, 16, EKF_N); // P Matrix
  float_mat_diagonal_scal(_Q, pow(2,2), EKF_N); // Q Matrix [inputs: a1x, a1y, a2x, a2y, r1, r2]
  filter->Q[4][4] = pow(0.2,2);
  filter->Q[5][5] = pow(0.2,2);
   
  float_mat_diagonal_scal(_R, 0.7, EKF_M); // R Matrix [range, h1, h2, u1, v1, u2, v2]
  filter->R[0][0] = pow(0.5,2);
  filter->R[1][1] = pow(0.2,2);
  filter->R[2][2] = pow(0.2,2);

  float_vect_zero(filter->X, EKF_N);       // Initial state
  filter->X[0]=EKF_XZERO;
  filter->X[1]=EKF_YZERO;
  filter->X[2]=-1;
  filter->X[3]=-1;
  filter->dt = 0.1; // Unitary time difference
}

/* Perform the prediction step

    Predict state
      x_p = f(x);
      A = Jacobian of f(x)

    Predict P
      P = A * P * A' + Q;

    Predict measure
      z_p = h(x_p)
      H = Jacobian of h(x)

*/
void discrete_ekf_no_north_predict(struct discrete_ekf *filter, float *U, float *measurements, float dt)
{
  float dX[EKF_N];
  
  MAKE_MATRIX_PTR(_tmp1, filter->tmp1, EKF_N);
  MAKE_MATRIX_PTR(_tmp2, filter->tmp2, EKF_N);
  MAKE_MATRIX_PTR(_tmp3, filter->tmp3, EKF_N);
  MAKE_MATRIX_PTR(_tmp4, filter->tmp4, EKF_N);
  MAKE_MATRIX_PTR(_H,    filter->H,    EKF_N);
  MAKE_MATRIX_PTR(_P,    filter->P,    EKF_N);
  MAKE_MATRIX_PTR(_Q,    filter->Q,    EKF_N);
  MAKE_MATRIX_PTR(_Gamma,filter->Gamma,EKF_N);
  MAKE_MATRIX_PTR(_Phi,  filter->Phi,  EKF_N);

  discrete_ekf_no_north_fsym(filter->X,U,dX); 
  discrete_ekf_no_north_Fx(filter->X,U,filter->Fx); // state transition matrix
  discrete_ekf_no_north_G(filter->X,U,filter->G); // input transition matrix
  
  float_vect_scale(dX,filter->dt,EKF_N)
  float_vect_sum(filter->Xp,filter->X,dX,EKF_N);

  c2d(EKF_N,EKF_N,EKF_L,filter->Fx,filter->G,filter->dt,filter->Phi,filter->Gamma);

  float_mat_mul(_tmp1, _Phi, _P, EKF_N, EKF_N, EKF_N); // tmp1 <- Phi*P
  float_mat_transpose_square(_Phi, EKF_N); // tmp2 <- Phi'
  float_mat_mul(_tmp3, _tmp1, _Phi); // tmp3 <- Phi*P*Phi'

  float_mat_mul(_tmp1, _Gamma, _Q, EKF_N, EKF_L, EKF_L); // tmp1 <- Gamma*Q
  float_mat_transpose(_tmp2, _Gamma, EKF_N, EKF_L); // tmp2 <- Gamma'
  float_mat_mul(_tmp4, _tmp1, _tmp2, EKF_N, EKF_L, EKF_N); // tmp4 <- Gamma*Q*Gamma  
  
  float_mat_sum(_P, _tmp3, _tmp4, EKF_N, EKF_N); // P <- Phi*P*Phi' + Gamma*Q*Gamma'

  discrete_ekf_no_north_hsym(filter->Xp,filter->Zp);
  discrete_ekf_no_north_Hx(filter->Xp,filter->H);

}