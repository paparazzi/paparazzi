#include "booz2_vf_int.h"

#include "booz_geometry_mixed.h"

int64_t b2_vfi_z;
int32_t b2_vfi_zd;
int32_t b2_vfi_abias;
int32_t b2_vfi_zdd;
int32_t b2_vfi_P[B2_VFI_S_SIZE][B2_VFI_S_SIZE];

/* initial covariance */
#define VFI_INIT_PZZ    BOOZ_INT_OF_FLOAT(1., B2_VFI_P_FRAC) 
#define VFI_INIT_PZDZD  BOOZ_INT_OF_FLOAT(1., B2_VFI_P_FRAC)
#define VFI_INIT_PABAB  BOOZ_INT_OF_FLOAT(1., B2_VFI_P_FRAC)

/* system and measurement noise */
#define VFI_ACCEL_NOISE 0.1
#define VFI_DT2_2 (1./(512.*512.)/2.)
#define VFI_DT    (1./512.)
#define VFI_QZZ         BOOZ_INT_OF_FLOAT(VFI_ACCEL_NOISE*VFI_DT2_2, B2_VFI_P_FRAC)
#define VFI_QZDZD       BOOZ_INT_OF_FLOAT(VFI_ACCEL_NOISE*VFI_DT, B2_VFI_P_FRAC)
#define VFI_QABAB       BOOZ_INT_OF_FLOAT(1e-7, B2_VFI_P_FRAC)
#define VFI_R           BOOZ_INT_OF_FLOAT(1., B2_VFI_P_FRAC)


void booz2_vfi_init(int32_t z0, int32_t zd0, int32_t bias0 ) {

  // initialize state vector
  b2_vfi_z     = z0;
  b2_vfi_zd    = zd0;
  b2_vfi_abias = bias0;
  b2_vfi_zdd   = 0;
  // initialize covariance
  int i, j;
  for (i=0; i<B2_VFI_S_SIZE; i++)
    for (j=0; j<B2_VFI_S_SIZE; j++) 
      b2_vfi_P[i][j] = 0;
  b2_vfi_P[B2_VFI_S_Z][B2_VFI_S_Z]   = VFI_INIT_PZZ;
  b2_vfi_P[B2_VFI_S_ZD][B2_VFI_S_ZD] = VFI_INIT_PZDZD;
  b2_vfi_P[B2_VFI_S_AB][B2_VFI_S_AB] = VFI_INIT_PABAB;

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

void booz2_vfi_propagate( int32_t accel_reading ) {
  
  // compute unbiased vertical acceleration
  b2_vfi_zdd = accel_reading + BOOZ_INT_OF_FLOAT(9.81, B2_VFI_ZDD_FRAC) - b2_vfi_abias;
  // propagate state
  const int32_t dz  = b2_vfi_zd  >> ( B2_VFI_F_UPDATE_FRAC + B2_VFI_ZD_FRAC - B2_VFI_Z_FRAC);
  b2_vfi_z += dz;
  const int32_t dzd = b2_vfi_zdd >> ( B2_VFI_F_UPDATE_FRAC + B2_VFI_ZDD_FRAC - B2_VFI_ZD_FRAC);
  b2_vfi_zd += dzd;

  // propagate covariance
  const int32_t tmp1  =  b2_vfi_P[1][0] + b2_vfi_P[0][1] + (b2_vfi_P[1][1]>>B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF00 =  b2_vfi_P[0][0] + (tmp1>>B2_VFI_F_UPDATE_FRAC);
  const int32_t tmp2  =  b2_vfi_P[1][1] - b2_vfi_P[0][2] - (b2_vfi_P[1][2]>>B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF01 =  b2_vfi_P[0][1] + (tmp2>>B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF02 =  b2_vfi_P[0][2] + (b2_vfi_P[1][2] >> B2_VFI_F_UPDATE_FRAC);;
  const int32_t tmp3  = -b2_vfi_P[2][0] + b2_vfi_P[1][1] - (b2_vfi_P[2][1]>>B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF10 =  b2_vfi_P[1][0] + (tmp3>>B2_VFI_F_UPDATE_FRAC);
  const int32_t tmp4  = -b2_vfi_P[2][1] - b2_vfi_P[1][2] + (b2_vfi_P[2][2]>>B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF11 =  b2_vfi_P[1][1] + (tmp4>>B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF12 =  b2_vfi_P[1][2] - (b2_vfi_P[2][2] >> B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF20 =  b2_vfi_P[2][0] + (b2_vfi_P[2][1] >> B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF21 =  b2_vfi_P[2][1] - (b2_vfi_P[2][2] >> B2_VFI_F_UPDATE_FRAC);
  const int32_t FPF22 =  b2_vfi_P[2][2];

  b2_vfi_P[0][0] = FPF00 + VFI_QZZ;
  b2_vfi_P[0][1] = FPF01;
  b2_vfi_P[0][2] = FPF02;
  b2_vfi_P[1][0] = FPF10;
  b2_vfi_P[1][1] = FPF11 + VFI_QZDZD;
  b2_vfi_P[1][2] = FPF12;
  b2_vfi_P[2][0] = FPF20;
  b2_vfi_P[2][1] = FPF21;
  b2_vfi_P[2][2] = FPF22 + VFI_QABAB;

}


void booz2_vfi_update( int32_t z_meas ) {

  const int64_t y = (z_meas<<(B2_VFI_Z_FRAC-B2_VFI_MEAS_Z_FRAC)) - b2_vfi_z;
  const int32_t S = b2_vfi_P[0][0] + VFI_R;
  
  const int32_t K1 = b2_vfi_P[0][0] / S; 
  const int32_t K2 = b2_vfi_P[1][0] / S; 
  const int32_t K3 = b2_vfi_P[2][0] / S; 

  b2_vfi_z     = b2_vfi_z     + ((K1 * y)>>B2_VFI_P_FRAC); 
  b2_vfi_zd    = b2_vfi_zd    + ((K2 * y)>>B2_VFI_P_FRAC); 
  b2_vfi_abias = b2_vfi_abias + ((K3 * y)>>B2_VFI_P_FRAC); 

#if 0

  const int32_t P11 = ((BOOZ_INT_OF_FLOAT(1., B2_VFI_P_RES) - K1) * b2_vfi_P[0][0])>>B2_VFI_P_RES;
  const int32_t P12 = (BOOZ_INT_OF_FLOAT(1., B2_VFI_P_RES) - K1) * b2_vfi_P[0][1];
  const int32_t P13 = (BOOZ_INT_OF_FLOAT(1., B2_VFI_P_RES) - K1) * b2_vfi_P[0][2];
  const int32_t P21 = -K2 * b2_vfi_P[0][0] + b2_vfi_P[1][0];
  const int32_t P22 = -K2 * b2_vfi_P[0][1] + b2_vfi_P[1][1];
  const int32_t P23 = -K2 * b2_vfi_P[0][2] + b2_vfi_P[1][2];
  const int32_t P31 = -K3 * b2_vfi_P[0][0] + b2_vfi_P[2][0];
  const int32_t P32 = -K3 * b2_vfi_P[0][1] + b2_vfi_P[2][1];
  const int32_t P33 = -K3 * b2_vfi_P[0][2] + b2_vfi_P[2][2];

  tl_vf_P[0][0] = P11;
  tl_vf_P[0][1] = P12;
  tl_vf_P[0][2] = P13;
  tl_vf_P[1][0] = P21;
  tl_vf_P[1][1] = P22;
  tl_vf_P[1][2] = P23;
  tl_vf_P[2][0] = P31;
  tl_vf_P[2][1] = P32;
  tl_vf_P[2][2] = P33;
#endif
}
