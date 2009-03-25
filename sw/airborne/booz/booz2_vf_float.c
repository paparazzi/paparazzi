#include "booz2_vf_float.h"

/*

X = [ z zdot bias ]

temps : 
  propagate 86us
  update    46us      

*/
/* initial covariance diagonal */
#define INIT_PXX 1.
/* process noise */
#define ACCEL_NOISE 0.5
#define Qzz       ACCEL_NOISE/512./512./2.
#define Qzdotzdot ACCEL_NOISE/512.
#define Qbiasbias 1e-7
#define R 1.

FLOAT_T b2_vff_z;
FLOAT_T b2_vff_bias;
FLOAT_T b2_vff_zdot;
FLOAT_T b2_vff_zdotdot;

FLOAT_T b2_vff_P[B2_VFF_STATE_SIZE][B2_VFF_STATE_SIZE];

FLOAT_T b2_vff_z_meas;

void b2_vff_init(FLOAT_T init_z, FLOAT_T init_zdot, FLOAT_T init_bias) {
  b2_vff_z    = init_z;
  b2_vff_zdot = init_zdot;
  b2_vff_bias = init_bias;
  int i, j;
  for (i=0; i<B2_VFF_STATE_SIZE; i++) {
    for (j=0; j<B2_VFF_STATE_SIZE; j++) 
      b2_vff_P[i][j] = 0.;
    b2_vff_P[i][i] = INIT_PXX;
  }

}


/*

 F = [ 1 dt -dt^2/2
       0  1 -dt
       0  0   1     ];
   
 B = [ dt^2/2 dt 0]';
 
 Q = [ 0.01  0     0
       0     0.01  0
       0     0     0.001 ];
 
 Xk1 = F * Xk0 + B * accel;
 
 Pk1 = F * Pk0 * F' + Q;

*/
void b2_vff_propagate(FLOAT_T accel) {
  /* update state */
  b2_vff_zdotdot = accel + 9.81 - b2_vff_bias;
  b2_vff_z = b2_vff_z + DT_VFILTER * b2_vff_zdot;
  b2_vff_zdot = b2_vff_zdot + DT_VFILTER * b2_vff_zdotdot; 
  /* update covariance */
  const FLOAT_T FPF00 = b2_vff_P[0][0] + DT_VFILTER * ( b2_vff_P[1][0] + b2_vff_P[0][1] + DT_VFILTER * b2_vff_P[1][1] );  
  const FLOAT_T FPF01 = b2_vff_P[0][1] + DT_VFILTER * ( b2_vff_P[1][1] - b2_vff_P[0][2] - DT_VFILTER * b2_vff_P[1][2] );
  const FLOAT_T FPF02 = b2_vff_P[0][2] + DT_VFILTER * ( b2_vff_P[1][2] );
  const FLOAT_T FPF10 = b2_vff_P[1][0] + DT_VFILTER * (-b2_vff_P[2][0] + b2_vff_P[1][1] - DT_VFILTER * b2_vff_P[2][1] );  
  const FLOAT_T FPF11 = b2_vff_P[1][1] + DT_VFILTER * (-b2_vff_P[2][1] - b2_vff_P[1][2] + DT_VFILTER * b2_vff_P[2][2] ); 
  const FLOAT_T FPF12 = b2_vff_P[1][2] + DT_VFILTER * (-b2_vff_P[2][2] );
  const FLOAT_T FPF20 = b2_vff_P[2][0] + DT_VFILTER * ( b2_vff_P[2][1] );
  const FLOAT_T FPF21 = b2_vff_P[2][1] + DT_VFILTER * (-b2_vff_P[2][2] );
  const FLOAT_T FPF22 = b2_vff_P[2][2];
 
  b2_vff_P[0][0] = FPF00 + Qzz;
  b2_vff_P[0][1] = FPF01;
  b2_vff_P[0][2] = FPF02;
  b2_vff_P[1][0] = FPF10;
  b2_vff_P[1][1] = FPF11 + Qzdotzdot;
  b2_vff_P[1][2] = FPF12;
  b2_vff_P[2][0] = FPF20;
  b2_vff_P[2][1] = FPF21;
  b2_vff_P[2][2] = FPF22 + Qbiasbias;

}
/*
  H = [1 0 0];
  R = 0.1;
  // state residual
  y = rangemeter - H * Xm;
  // covariance residual
  S = H*Pm*H' + R;
  // kalman gain
  K = Pm*H'*inv(S);
  // update state
  Xp = Xm + K*y;
  // update covariance
  Pp = Pm - K*H*Pm;
*/
void b2_vff_update(FLOAT_T z_meas) {
  b2_vff_z_meas = z_meas;

  const FLOAT_T y = z_meas - b2_vff_z;
  const FLOAT_T S = b2_vff_P[0][0] + R;
  const FLOAT_T K1 = b2_vff_P[0][0] * 1/S; 
  const FLOAT_T K2 = b2_vff_P[1][0] * 1/S; 
  const FLOAT_T K3 = b2_vff_P[2][0] * 1/S; 
  
  b2_vff_z    = b2_vff_z    + K1 * y; 
  b2_vff_zdot = b2_vff_zdot + K2 * y; 
  b2_vff_bias = b2_vff_bias + K3 * y; 

  const FLOAT_T P11 = (1. - K1) * b2_vff_P[0][0];
  const FLOAT_T P12 = (1. - K1) * b2_vff_P[0][1];
  const FLOAT_T P13 = (1. - K1) * b2_vff_P[0][2];
  const FLOAT_T P21 = -K2 * b2_vff_P[0][0] + b2_vff_P[1][0];
  const FLOAT_T P22 = -K2 * b2_vff_P[0][1] + b2_vff_P[1][1];
  const FLOAT_T P23 = -K2 * b2_vff_P[0][2] + b2_vff_P[1][2];
  const FLOAT_T P31 = -K3 * b2_vff_P[0][0] + b2_vff_P[2][0];
  const FLOAT_T P32 = -K3 * b2_vff_P[0][1] + b2_vff_P[2][1];
  const FLOAT_T P33 = -K3 * b2_vff_P[0][2] + b2_vff_P[2][2];

  b2_vff_P[0][0] = P11;
  b2_vff_P[0][1] = P12;
  b2_vff_P[0][2] = P13;
  b2_vff_P[1][0] = P21;
  b2_vff_P[1][1] = P22;
  b2_vff_P[1][2] = P23;
  b2_vff_P[2][0] = P31;
  b2_vff_P[2][1] = P32;
  b2_vff_P[2][2] = P33;

}

void b2_vff_realign(FLOAT_T z_meas) {
  b2_vff_z = z_meas;
}


