#include "agl_vfilter.h"

#include CONFIG
#include "airframe.h"
#include "std.h"

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

float agl_vf_z;
float agl_vf_zdot;
float agl_vf_bias;

struct adc_buf agl_adc_buf;

float agl_vf_P[STATE_SIZE][STATE_SIZE];

float agl_vf_z_meas;

void agl_vf_init(float init_z, float init_zdot, float init_bias) {
  agl_vf_z = init_z;
  agl_vf_zdot = init_zdot;
  agl_vf_bias = init_bias;
  int i, j;
  for (i=0; i<STATE_SIZE; i++) {
    for (j=0; j<STATE_SIZE; j++) 
      agl_vf_P[i][j] = 0.;
    agl_vf_P[i][i] = INIT_PXX;
  }
  adc_buf_channel(ADC_CHANNEL_TELEMETER, &agl_adc_buf, DEFAULT_AV_NB_SAMPLE);

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
void agl_vf_predict(float accel) {
  /* update state */
  float u = accel + 9.81;
  agl_vf_z = agl_vf_z + DT_VFILTER * agl_vf_zdot;
  agl_vf_zdot = agl_vf_zdot + DT_VFILTER * ( u - agl_vf_bias); 
  /* update covariance */
  const float FPF00 = agl_vf_P[0][0] + DT_VFILTER * ( agl_vf_P[1][0] + agl_vf_P[0][1] + DT_VFILTER * agl_vf_P[1][1] );  
  const float FPF01 = agl_vf_P[0][1] + DT_VFILTER * ( agl_vf_P[1][1] - agl_vf_P[0][2] - DT_VFILTER * agl_vf_P[1][2] );
  const float FPF02 = agl_vf_P[0][2] + DT_VFILTER * ( agl_vf_P[1][2] );
  const float FPF10 = agl_vf_P[1][0] + DT_VFILTER * (-agl_vf_P[2][0] + agl_vf_P[1][1] - DT_VFILTER * agl_vf_P[2][1] );  
  const float FPF11 = agl_vf_P[1][1] + DT_VFILTER * (-agl_vf_P[2][1] - agl_vf_P[1][2] + DT_VFILTER * agl_vf_P[2][2] ); 
  const float FPF12 = agl_vf_P[1][2] + DT_VFILTER * (-agl_vf_P[2][2] );
  const float FPF20 = agl_vf_P[2][0] + DT_VFILTER * ( agl_vf_P[2][1] );
  const float FPF21 = agl_vf_P[2][1] + DT_VFILTER * (-agl_vf_P[2][2] );
  const float FPF22 = agl_vf_P[2][2];
 
  agl_vf_P[0][0] = FPF00 + Qzz;
  agl_vf_P[0][1] = FPF01;
  agl_vf_P[0][2] = FPF02;
  agl_vf_P[1][0] = FPF10;
  agl_vf_P[1][1] = FPF11 + Qzdotzdot;
  agl_vf_P[1][2] = FPF12;
  agl_vf_P[2][0] = FPF20;
  agl_vf_P[2][1] = FPF21;
  agl_vf_P[2][2] = FPF22 + Qbiasbias;

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
void agl_vf_update(float z_meas) {
  agl_vf_z_meas = z_meas;

  const float y = z_meas - agl_vf_z;
  const float S = agl_vf_P[0][0] + R;
  const float K1 = agl_vf_P[0][0] * 1/S; 
  const float K2 = agl_vf_P[1][0] * 1/S; 
  const float K3 = agl_vf_P[2][0] * 1/S; 
  
  agl_vf_z    = agl_vf_z    + K1 * y; 
  agl_vf_zdot = agl_vf_zdot + K2 * y; 
  agl_vf_bias = agl_vf_bias + K3 * y; 

  const float P11 = (1. - K1) * agl_vf_P[0][0];
  const float P12 = (1. - K1) * agl_vf_P[0][1];
  const float P13 = (1. - K1) * agl_vf_P[0][2];
  const float P21 = -K2 * agl_vf_P[0][0] + agl_vf_P[1][0];
  const float P22 = -K2 * agl_vf_P[0][1] + agl_vf_P[1][1];
  const float P23 = -K2 * agl_vf_P[0][2] + agl_vf_P[1][2];
  const float P31 = -K3 * agl_vf_P[0][0] + agl_vf_P[2][0];
  const float P32 = -K3 * agl_vf_P[0][1] + agl_vf_P[2][1];
  const float P33 = -K3 * agl_vf_P[0][2] + agl_vf_P[2][2];

  agl_vf_P[0][0] = P11;
  agl_vf_P[0][1] = P12;
  agl_vf_P[0][2] = P13;
  agl_vf_P[1][0] = P21;
  agl_vf_P[1][1] = P22;
  agl_vf_P[1][2] = P23;
  agl_vf_P[2][0] = P31;
  agl_vf_P[2][1] = P32;
  agl_vf_P[2][2] = P33;

}

