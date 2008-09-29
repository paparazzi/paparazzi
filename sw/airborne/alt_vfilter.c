#include "alt_vfilter.h"

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

float alt_vf_z;
float alt_vf_zdot;
float alt_vf_bias;

float alt_vf_P[STATE_SIZE][STATE_SIZE];

float alt_vf_z_meas;

void alt_vf_init(float init_z, float init_zdot, float init_bias) {
  alt_vf_z = init_z;
  alt_vf_zdot = init_zdot;
  alt_vf_bias = init_bias;
  int i, j;
  for (i=0; i<STATE_SIZE; i++) {
    for (j=0; j<STATE_SIZE; j++) 
      alt_vf_P[i][j] = 0.;
    alt_vf_P[i][i] = INIT_PXX;
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
void alt_vf_predict(float accel) {
  /* update state */
  float u = accel + 9.81;
  alt_vf_z = alt_vf_z + DT_VFILTER * alt_vf_zdot;
  alt_vf_zdot = alt_vf_zdot + DT_VFILTER * ( u - alt_vf_bias); 
  /* update covariance */
  const float FPF00 = alt_vf_P[0][0] + DT_VFILTER * ( alt_vf_P[1][0] + alt_vf_P[0][1] + DT_VFILTER * alt_vf_P[1][1] );  
  const float FPF01 = alt_vf_P[0][1] + DT_VFILTER * ( alt_vf_P[1][1] - alt_vf_P[0][2] - DT_VFILTER * alt_vf_P[1][2] );
  const float FPF02 = alt_vf_P[0][2] + DT_VFILTER * ( alt_vf_P[1][2] );
  const float FPF10 = alt_vf_P[1][0] + DT_VFILTER * (-alt_vf_P[2][0] + alt_vf_P[1][1] - DT_VFILTER * alt_vf_P[2][1] );  
  const float FPF11 = alt_vf_P[1][1] + DT_VFILTER * (-alt_vf_P[2][1] - alt_vf_P[1][2] + DT_VFILTER * alt_vf_P[2][2] ); 
  const float FPF12 = alt_vf_P[1][2] + DT_VFILTER * (-alt_vf_P[2][2] );
  const float FPF20 = alt_vf_P[2][0] + DT_VFILTER * ( alt_vf_P[2][1] );
  const float FPF21 = alt_vf_P[2][1] + DT_VFILTER * (-alt_vf_P[2][2] );
  const float FPF22 = alt_vf_P[2][2];
 
  alt_vf_P[0][0] = FPF00 + Qzz;
  alt_vf_P[0][1] = FPF01;
  alt_vf_P[0][2] = FPF02;
  alt_vf_P[1][0] = FPF10;
  alt_vf_P[1][1] = FPF11 + Qzdotzdot;
  alt_vf_P[1][2] = FPF12;
  alt_vf_P[2][0] = FPF20;
  alt_vf_P[2][1] = FPF21;
  alt_vf_P[2][2] = FPF22 + Qbiasbias;

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
void alt_vf_update_z(float z_meas) {
  alt_vf_z_meas = z_meas;

  const float y = z_meas - alt_vf_z;
  const float S = alt_vf_P[0][0] + R;
  const float K1 = alt_vf_P[0][0] * 1/S; 
  const float K2 = alt_vf_P[1][0] * 1/S; 
  const float K3 = alt_vf_P[2][0] * 1/S; 
  
  alt_vf_z    = alt_vf_z    + K1 * y; 
  alt_vf_zdot = alt_vf_zdot + K2 * y; 
  alt_vf_bias = alt_vf_bias + K3 * y; 

  const float P11 = (1. - K1) * alt_vf_P[0][0];
  const float P12 = (1. - K1) * alt_vf_P[0][1];
  const float P13 = (1. - K1) * alt_vf_P[0][2];
  const float P21 = -K2 * alt_vf_P[0][0] + alt_vf_P[1][0];
  const float P22 = -K2 * alt_vf_P[0][1] + alt_vf_P[1][1];
  const float P23 = -K2 * alt_vf_P[0][2] + alt_vf_P[1][2];
  const float P31 = -K3 * alt_vf_P[0][0] + alt_vf_P[2][0];
  const float P32 = -K3 * alt_vf_P[0][1] + alt_vf_P[2][1];
  const float P33 = -K3 * alt_vf_P[0][2] + alt_vf_P[2][2];

  alt_vf_P[0][0] = P11;
  alt_vf_P[0][1] = P12;
  alt_vf_P[0][2] = P13;
  alt_vf_P[1][0] = P21;
  alt_vf_P[1][1] = P22;
  alt_vf_P[1][2] = P23;
  alt_vf_P[2][0] = P31;
  alt_vf_P[2][1] = P32;
  alt_vf_P[2][2] = P33;

}

/*
  H = [0 1 0];
  R = 0.1;
  // state residual
  yd = vz - H * Xm;
  // covariance residual
  S = H*Pm*H' + R;
  // kalman gain
  K = Pm*H'*inv(S);
  // update state
  Xp = Xm + K*yd;
  // update covariance
  Pp = Pm - K*H*Pm;
*/
void alt_vf_update_vz(float vz) {
  const float yd = vz - alt_vf_zdot;
  const float S = alt_vf_P[1][1] + R;
  const float K1 = alt_vf_P[0][1] * 1/S; 
  const float K2 = alt_vf_P[1][1] * 1/S; 
  const float K3 = alt_vf_P[2][1] * 1/S; 
  
  alt_vf_z    = alt_vf_z    + K1 * yd; 
  alt_vf_zdot = alt_vf_zdot + K2 * yd; 
  alt_vf_bias = alt_vf_bias + K3 * yd; 

  const float P11 = -K1 * alt_vf_P[1][0] + alt_vf_P[0][0];
  const float P12 = -K1 * alt_vf_P[1][1] + alt_vf_P[0][1];
  const float P13 = -K1 * alt_vf_P[1][2] + alt_vf_P[0][2];
  const float P21 = (1. - K2) * alt_vf_P[1][0];
  const float P22 = (1. - K2) * alt_vf_P[1][1];
  const float P23 = (1. - K2) * alt_vf_P[1][2];
  const float P31 = -K3 * alt_vf_P[1][0] + alt_vf_P[2][0];
  const float P32 = -K3 * alt_vf_P[1][1] + alt_vf_P[2][1];
  const float P33 = -K3 * alt_vf_P[1][2] + alt_vf_P[2][2];

  alt_vf_P[0][0] = P11;
  alt_vf_P[0][1] = P12;
  alt_vf_P[0][2] = P13;
  alt_vf_P[1][0] = P21;
  alt_vf_P[1][1] = P22;
  alt_vf_P[1][2] = P23;
  alt_vf_P[2][0] = P31;
  alt_vf_P[2][1] = P32;
  alt_vf_P[2][2] = P33;

}


