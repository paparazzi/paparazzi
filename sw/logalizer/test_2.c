#include <math.h>

#include "ahrs_utils.h"

float C[4];

float dcm00;
float dcm01;
float dcm02;
float dcm12;
float dcm22;

float phi;
float theta;
float psi;

float q0;
float q1;
float q2;
float q3;


/* d_euler / dq */
/* 
   phi = atan ( 2(q2q3 + q0q1) / (q0^2 - q1^2 - q2^2 + q3^2))

*/
void test_dphi_dq ( void ) {
  float my_dcm22 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  float dcm22_sq = my_dcm22 * my_dcm22;
  float my_dcm12 = 2*(q2*q3 + q0*q1);
  float dcm12_sq = my_dcm12 * my_dcm12;
  C[0] = 2 * q1 * dcm22 / (dcm22_sq + dcm12_sq);
  C[1] = 2 * q0 * dcm22 / (dcm22_sq + dcm12_sq);
  C[2] = 2 * q3 * dcm22 / (dcm22_sq + dcm12_sq);
  C[3] = 2 * q2 * dcm22 / (dcm22_sq + dcm12_sq);
}
/*
  theta = asin(-2(q1q3 - q0q2))

  dasin = 1/sqrt(1-x^2)
*/
void test_dtheta_dq ( void ) {
  float my_dcm02 = 2 * (q1*q3 - q0*q2);
  float dcm02_sq = my_dcm02 * my_dcm02;
  C[0] =  2 * q2 / (sqrt(1 - dcm02_sq));
  C[1] = -2 * q3 / (sqrt(1 - dcm02_sq));
  C[2] =  2 * q0 / (sqrt(1 - dcm02_sq));
  C[3] = -2 * q1 / (sqrt(1 - dcm02_sq));
}

/* 
   psi = atan ( 2(q1q2 + q0q3) / (q0^2 + q1^2 - q2^2 - q3^2))


   datan = 1/(1+x^2)
*/
void test_dpsi_dq ( void ) {
  float my_dcm00 =  1 - 2. * q2 * q2 - 2. * q3 * q3;
  float my_dcm01 = 2*(q1*q2 +q0*q3);
  float dcm00_sq = my_dcm00 * my_dcm00;
  float dcm01_sq = my_dcm01 * my_dcm01;
  C[0] = 2. * q3 * dcm00 / (dcm00_sq + dcm01_sq); 
  C[1] = 2. * q2 * dcm00 / (dcm00_sq + dcm01_sq);
  C[2] = 2. * q1 * dcm00 / (dcm00_sq + dcm01_sq);
  C[3] = 2. * q0 * dcm00 / (dcm00_sq + dcm01_sq);
}





int main (int argc, char** argv) {

 phi = 0.3;
 theta = 0.5;
 psi = 0.5;
 PrintEuler()

 quat_of_eulers();
 PrintQuat();
 DCM_of_quat();
 PrintDCM();
#if 1
 printf("phi\n");
 compute_dphi_dq();
 PrintC();
 test_dphi_dq();
 PrintC();
#endif

#if 1
 printf("theta\n");
 compute_dtheta_dq();
 PrintC();
 test_dtheta_dq();
 PrintC();
#endif

#if 1
 printf("psi\n");
 compute_dpsi_dq();
 PrintC();
 test_dpsi_dq();
 PrintC();
#endif

 return 0;
}
