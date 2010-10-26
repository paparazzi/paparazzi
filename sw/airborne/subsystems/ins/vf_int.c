/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "subsystems/ins/vf_int.h"

#include "booz_geometry_mixed.h"

int64_t vfi_z;
int32_t vfi_zd;
int32_t vfi_abias;
int32_t vfi_zdd;
int32_t vfi_P[VFI_S_SIZE][VFI_S_SIZE];

/* initial covariance */
#define VFI_INIT_PZZ    BOOZ_INT_OF_FLOAT(1., VFI_P_FRAC)
#define VFI_INIT_PZDZD  BOOZ_INT_OF_FLOAT(1., VFI_P_FRAC)
#define VFI_INIT_PABAB  BOOZ_INT_OF_FLOAT(1., VFI_P_FRAC)

/* system and measurement noise */
#define VFI_ACCEL_NOISE 0.1
#define VFI_DT2_2 (1./(512.*512.)/2.)
#define VFI_DT    (1./512.)
#define VFI_QZZ         BOOZ_INT_OF_FLOAT(VFI_ACCEL_NOISE*VFI_DT2_2, VFI_P_FRAC)
#define VFI_QZDZD       BOOZ_INT_OF_FLOAT(VFI_ACCEL_NOISE*VFI_DT, VFI_P_FRAC)
#define VFI_QABAB       BOOZ_INT_OF_FLOAT(1e-7, VFI_P_FRAC)
#define VFI_R           BOOZ_INT_OF_FLOAT(1., VFI_P_FRAC)


void vfi_init(int32_t z0, int32_t zd0, int32_t bias0 ) {

  // initialize state vector
  vfi_z     = z0;
  vfi_zd    = zd0;
  vfi_abias = bias0;
  vfi_zdd   = 0;
  // initialize covariance
  int i, j;
  for (i=0; i<VFI_S_SIZE; i++)
    for (j=0; j<VFI_S_SIZE; j++)
      vfi_P[i][j] = 0;
  vfi_P[VFI_S_Z][VFI_S_Z]   = VFI_INIT_PZZ;
  vfi_P[VFI_S_ZD][VFI_S_ZD] = VFI_INIT_PZDZD;
  vfi_P[VFI_S_AB][VFI_S_AB] = VFI_INIT_PABAB;

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

void vfi_propagate( int32_t accel_reading ) {

  // compute unbiased vertical acceleration
  vfi_zdd = accel_reading + BOOZ_INT_OF_FLOAT(9.81, VFI_ZDD_FRAC) - vfi_abias;
  // propagate state
  const int32_t dz  = vfi_zd  >> ( VFI_F_UPDATE_FRAC + VFI_ZD_FRAC - VFI_Z_FRAC);
  vfi_z += dz;
  const int32_t dzd = vfi_zdd >> ( VFI_F_UPDATE_FRAC + VFI_ZDD_FRAC - VFI_ZD_FRAC);
  vfi_zd += dzd;

  // propagate covariance
  const int32_t tmp1  =  vfi_P[1][0] + vfi_P[0][1] + (vfi_P[1][1]>>VFI_F_UPDATE_FRAC);
  const int32_t FPF00 =  vfi_P[0][0] + (tmp1>>VFI_F_UPDATE_FRAC);
  const int32_t tmp2  =  vfi_P[1][1] - vfi_P[0][2] - (vfi_P[1][2]>>VFI_F_UPDATE_FRAC);
  const int32_t FPF01 =  vfi_P[0][1] + (tmp2>>VFI_F_UPDATE_FRAC);
  const int32_t FPF02 =  vfi_P[0][2] + (vfi_P[1][2] >> VFI_F_UPDATE_FRAC);;
  const int32_t tmp3  = -vfi_P[2][0] + vfi_P[1][1] - (vfi_P[2][1]>>VFI_F_UPDATE_FRAC);
  const int32_t FPF10 =  vfi_P[1][0] + (tmp3>>VFI_F_UPDATE_FRAC);
  const int32_t tmp4  = -vfi_P[2][1] - vfi_P[1][2] + (vfi_P[2][2]>>VFI_F_UPDATE_FRAC);
  const int32_t FPF11 =  vfi_P[1][1] + (tmp4>>VFI_F_UPDATE_FRAC);
  const int32_t FPF12 =  vfi_P[1][2] - (vfi_P[2][2] >> VFI_F_UPDATE_FRAC);
  const int32_t FPF20 =  vfi_P[2][0] + (vfi_P[2][1] >> VFI_F_UPDATE_FRAC);
  const int32_t FPF21 =  vfi_P[2][1] - (vfi_P[2][2] >> VFI_F_UPDATE_FRAC);
  const int32_t FPF22 =  vfi_P[2][2];

  vfi_P[0][0] = FPF00 + VFI_QZZ;
  vfi_P[0][1] = FPF01;
  vfi_P[0][2] = FPF02;
  vfi_P[1][0] = FPF10;
  vfi_P[1][1] = FPF11 + VFI_QZDZD;
  vfi_P[1][2] = FPF12;
  vfi_P[2][0] = FPF20;
  vfi_P[2][1] = FPF21;
  vfi_P[2][2] = FPF22 + VFI_QABAB;

}


void vfi_update( int32_t z_meas ) {

  const int64_t y = (z_meas<<(VFI_Z_FRAC-VFI_MEAS_Z_FRAC)) - vfi_z;
  const int32_t S = vfi_P[0][0] + VFI_R;

  const int32_t K1 = vfi_P[0][0] / S;
  const int32_t K2 = vfi_P[1][0] / S;
  const int32_t K3 = vfi_P[2][0] / S;

  vfi_z     = vfi_z     + ((K1 * y)>>VFI_P_FRAC);
  vfi_zd    = vfi_zd    + ((K2 * y)>>VFI_P_FRAC);
  vfi_abias = vfi_abias + ((K3 * y)>>VFI_P_FRAC);

#if 0

  const int32_t P11 = ((BOOZ_INT_OF_FLOAT(1., VFI_P_RES) - K1) * vfi_P[0][0])>>VFI_P_RES;
  const int32_t P12 = (BOOZ_INT_OF_FLOAT(1., VFI_P_RES) - K1) * vfi_P[0][1];
  const int32_t P13 = (BOOZ_INT_OF_FLOAT(1., VFI_P_RES) - K1) * vfi_P[0][2];
  const int32_t P21 = -K2 * vfi_P[0][0] + vfi_P[1][0];
  const int32_t P22 = -K2 * vfi_P[0][1] + vfi_P[1][1];
  const int32_t P23 = -K2 * vfi_P[0][2] + vfi_P[1][2];
  const int32_t P31 = -K3 * vfi_P[0][0] + vfi_P[2][0];
  const int32_t P32 = -K3 * vfi_P[0][1] + vfi_P[2][1];
  const int32_t P33 = -K3 * vfi_P[0][2] + vfi_P[2][2];

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
