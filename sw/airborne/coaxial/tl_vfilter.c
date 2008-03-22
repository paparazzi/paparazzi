#include "tl_vfilter.h"

/*

X = [ z zdot bias ]

temps : 
  predict 86us
  update  46us      

*/
/* initial covariance diagonal */
#define INIT_PXX 10.
/* process noise */
#define Qzz       0.001
#define Qzdotzdot 0.001
#define Qbiasbias 0.00001
#define R 2.

FLOAT_T tl_vf_z;
FLOAT_T tl_vf_zdot;
FLOAT_T tl_vf_bias;

FLOAT_T tl_vf_P[STATE_SIZE][STATE_SIZE];

FLOAT_T tl_vf_z_meas;

void tl_vf_init(FLOAT_T init_z, FLOAT_T init_zdot, FLOAT_T init_bias) {
  tl_vf_z = init_z;
  tl_vf_zdot = init_zdot;
  tl_vf_bias = init_bias;
  int i, j;
  for (i=0; i<STATE_SIZE; i++) {
    for (j=0; j<STATE_SIZE; j++) 
      tl_vf_P[i][j] = 0.;
    tl_vf_P[i][i] = INIT_PXX;
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
void tl_vf_predict(FLOAT_T accel) {
  /* update state */
  FLOAT_T u = accel + 9.81;
  tl_vf_z = tl_vf_z + DT_VFILTER * tl_vf_zdot;
  tl_vf_zdot = tl_vf_zdot + DT_VFILTER * ( u - tl_vf_bias); 
  /* update covariance */
  const FLOAT_T FPF00 = tl_vf_P[0][0] + DT_VFILTER * ( tl_vf_P[1][0] + tl_vf_P[0][1] + DT_VFILTER * tl_vf_P[1][1] );  
  const FLOAT_T FPF01 = tl_vf_P[0][1] + DT_VFILTER * ( tl_vf_P[1][1] - tl_vf_P[0][2] - DT_VFILTER * tl_vf_P[1][2] );
  const FLOAT_T FPF02 = tl_vf_P[0][2] + DT_VFILTER * ( tl_vf_P[1][2] );
  const FLOAT_T FPF10 = tl_vf_P[1][0] + DT_VFILTER * (-tl_vf_P[2][0] + tl_vf_P[1][1] - DT_VFILTER * tl_vf_P[2][1] );  
  const FLOAT_T FPF11 = tl_vf_P[1][1] + DT_VFILTER * (-tl_vf_P[2][1] - tl_vf_P[1][2] + DT_VFILTER * tl_vf_P[2][2] ); 
  const FLOAT_T FPF12 = tl_vf_P[1][2] + DT_VFILTER * (-tl_vf_P[2][2] );
  const FLOAT_T FPF20 = tl_vf_P[2][0] + DT_VFILTER * ( tl_vf_P[2][1] );
  const FLOAT_T FPF21 = tl_vf_P[2][1] + DT_VFILTER * (-tl_vf_P[2][2] );
  const FLOAT_T FPF22 = tl_vf_P[2][2];
 
  tl_vf_P[0][0] = FPF00 + Qzz;
  tl_vf_P[0][1] = FPF01;
  tl_vf_P[0][2] = FPF02;
  tl_vf_P[1][0] = FPF10;
  tl_vf_P[1][1] = FPF11 + Qzdotzdot;
  tl_vf_P[1][2] = FPF12;
  tl_vf_P[2][0] = FPF20;
  tl_vf_P[2][1] = FPF21;
  tl_vf_P[2][2] = FPF22 + Qbiasbias;

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
void tl_vf_update(FLOAT_T z_meas) {
  tl_vf_z_meas = z_meas;

  const FLOAT_T y = z_meas - tl_vf_z;
  const FLOAT_T S = tl_vf_P[0][0] + R;
  const FLOAT_T K1 = tl_vf_P[0][0] * 1/S; 
  const FLOAT_T K2 = tl_vf_P[1][0] * 1/S; 
  const FLOAT_T K3 = tl_vf_P[2][0] * 1/S; 
  
  tl_vf_z    = tl_vf_z    + K1 * y; 
  tl_vf_zdot = tl_vf_zdot + K2 * y; 
  tl_vf_bias = tl_vf_bias + K3 * y; 

  const FLOAT_T P11 = (1. - K1) * tl_vf_P[0][0];
  const FLOAT_T P12 = (1. - K1) * tl_vf_P[0][1];
  const FLOAT_T P13 = (1. - K1) * tl_vf_P[0][2];
  const FLOAT_T P21 = -K2 * tl_vf_P[0][0] + tl_vf_P[1][0];
  const FLOAT_T P22 = -K2 * tl_vf_P[0][1] + tl_vf_P[1][1];
  const FLOAT_T P23 = -K2 * tl_vf_P[0][2] + tl_vf_P[1][2];
  const FLOAT_T P31 = -K3 * tl_vf_P[0][0] + tl_vf_P[2][0];
  const FLOAT_T P32 = -K3 * tl_vf_P[0][1] + tl_vf_P[2][1];
  const FLOAT_T P33 = -K3 * tl_vf_P[0][2] + tl_vf_P[2][2];

  tl_vf_P[0][0] = P11;
  tl_vf_P[0][1] = P12;
  tl_vf_P[0][2] = P13;
  tl_vf_P[1][0] = P21;
  tl_vf_P[1][1] = P22;
  tl_vf_P[1][2] = P23;
  tl_vf_P[2][0] = P31;
  tl_vf_P[2][1] = P32;
  tl_vf_P[2][2] = P33;

}
