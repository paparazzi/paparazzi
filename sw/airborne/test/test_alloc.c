/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

/**
 * @file test_alloc.c
 *
 * Test routine for WLS (weighted least squares) control allocation
 *
 * Comparing a precomputed solution from Matlab with the one coming
 * from our PPRZ implementation.
 */

#include <stdio.h>
#include "std.h"
#include "math/wls/wls_alloc.h"

#define INDI_NUM_ACT WLS_N_U_MAX
#define INDI_OUTPUTS WLS_N_V_MAX

void test_overdetermined(void);
void test_overdetermined2(void);
void calc_nu_out(float** Bwls, float* du, float* nu_out);

int main(int argc, char **argv)
{
// #define INDI_NUM_ACT 6
  // test_overdetermined();

  // Make sure to set CA_N_U to 8 in the test makefile!
  #define INDI_NUM_ACT 8
  test_overdetermined2();

/*#define INDI_NUM_ACT 4*/
  /*test_four_by_four();*/
}

/*
 * function to test wls with 4x4 (outputs x inputs) system
 */
void test_four_by_four(void)
{
  float u_min[INDI_NUM_ACT] = { -107, -19093, 0, -95, };
  float u_max[INDI_NUM_ACT] = {19093, 107, 4600, 4505, };

  float g1g2[INDI_OUTPUTS][INDI_NUM_ACT] = {
    {      0,         0,  -0.0105,  0.0107016},
    { -0.0030044, 0.0030044, 0.035, 0.035},
    { -0.004856, -0.004856, 0, 0},
    {       0,         0,   -0.0011,   -0.0011}
  };

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {100, 1000, 0.1, 10};
  /*static float Wv[INDI_OUTPUTS] = {10, 10, 0.1, 1};*/

  // The control objective in array format
  float indi_v[INDI_OUTPUTS] = {10.8487,  -10.5658,    6.8383,    1.8532};
  float indi_du[INDI_NUM_ACT];

  // Initialize the array of pointers to the rows of g1g2
  float *Bwls[INDI_OUTPUTS];
  uint8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // WLS Control Allocator
  int num_iter =
    wls_alloc(indi_du, indi_v, u_min, u_max, Bwls, 0, 0, Wv, 0, 0, 10000, 10, INDI_NUM_ACT, INDI_OUTPUTS);

  printf("finished in %d iterations\n", num_iter);
  printf("du = %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3]);
}

/* function to test wls for the oneloop controller 6x11 (outputs x inputs) system */
void test_oneloop_rw3c(void)
{
  float gamma = 20;
  float du_min[WLS_N_U_MAX]  = {0,	0,	0,	0,	-0.999999000000000,	-0.818607651741293,	-0.906420781094527,	-1.00441964179104,	-1.00551747263682,	-0.625892592039801,	-1.38242921393035};
  float du_max[WLS_N_U_MAX]  = {0,	0,	0,	0,	1.00000000000000e-06,	0.181392348258707,	1.09357921890547,	0.995580358208955,	0.994482527363184,	1.37410740796020,	0.617570786069651};
  float du_pref[WLS_N_U_MAX] = {0,	0,	0,	0,	-0.999999000000000,	-0.818607651741293,	0.0935792189054731,	-0.00441964179104483,	-0.00551747263681596,	0.374107407960200,	-0.382429213930349};
 

  float Wv[WLS_N_V_MAX] = {4,4,4,8,8,1};
  float Wu[WLS_N_U_MAX] = {0.75,0.75,0.75,0.75,1,0.1,0.1,0.1,0.1,1.2,1.2};
  float wls_v[WLS_N_V_MAX] = {-4.86661635467990,	7.02758381773395,	3.20567040886711,	-1.05634927093606,	-4.53403714285678,	-0.125133236453181};
  //float wls_v[WLS_N_V_MAX] = {0,0,0,0,0,0};
  float wls_du[WLS_N_U_MAX] = {0};
  float g1g2[WLS_N_V_MAX][WLS_N_U_MAX] = {
      {0, 0, 0, 0, -47.9216734146340, 0, 0, 0, 0, -23.2233317560976, 18.3478526585367},
      {0, 0, 0, 0, 104.951841268293, 0, 0, 0, 0, -13.1313056097561, 8.31975553658538},
      {0, 0, 0, 0, -33.4459766585367, 0, 0, 0, 0, -7.95380339024395, -67.3910292682927},
      {0, 0, 0, 0, 0, 0, 0, 834.716482762376, 623.043724059406, 0, 0},
      {0, 0, 0, 0, 0, 1841.05446648515, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 79.4693096831683, 0, 0, 0, 0}
  };

  // Scale to test numerical errors
  float scaler = 1; // replace with your desired value
  gamma *= scaler;
  for (int i = 0; i < WLS_N_U_MAX; i++) {
      du_min[i]  *= scaler;
      du_max[i]  *= scaler;
      du_pref[i] *= scaler;
  }
  
  for (int i = 0; i < WLS_N_V_MAX; i++) {
      for (int j = 0; j < WLS_N_U_MAX; j++) {
          g1g2[i][j] /= scaler;
      }
  }
    // Initialize the array of pointers to the rows of g1g2
  float *Bwls[WLS_N_V_MAX];
  uint8_t i;
  for (i = 0; i < WLS_N_V_MAX; i++) {
    Bwls[i] = g1g2[i];
  }
  // WLS Control Allocator
  //int num_iter = wls_alloc(indi_du, indi_v, du_min, du_max, Bwls, 0, 0, Wv, 0, u_p, 0, 10, INDI_NUM_ACT, INDI_OUTPUTS);
  int num_iter = wls_alloc(wls_du, wls_v, du_min, du_max, Bwls, du_pref, 0, Wv, Wu, du_pref, gamma, 10, WLS_N_U_MAX, WLS_N_V_MAX);
  float nu_out[WLS_N_V_MAX] = {0.0f};
  calc_nu_out(Bwls, wls_du, nu_out);
  float SC = 0;
  calc_secondary_cost(wls_du, du_pref, Wu, &SC);
  printf("finished in %d iterations\n", num_iter);
  printf("Input:\n");
  printf("--------- jN            jE            jD            jp            jq            jr\n");
  printf("v       = %-13f %-13f %-13f %-13f %-13f %-13f\n", wls_v[0], wls_v[1], wls_v[2], wls_v[3], wls_v[4], wls_v[5]);
  printf("v_out   = %-13f %-13f %-13f %-13f %-13f %-13f\n", nu_out[0], nu_out[1], nu_out[2], nu_out[3], nu_out[4], nu_out[5]);
  printf("Wv      = %-13f %-13f %-13f %-13f %-13f %-13f\n", Wv[0], Wv[1], Wv[2], Wv[3], Wv[4], Wv[5]);
  printf("Control effectiveness matrix g1g2 = \n");
  printf("--------- mF            mR            mB            mL            mP            ele           rud           ail           flp           phi           theta\n");
  printf("jN        %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", g1g2[0][0], g1g2[0][1], g1g2[0][2], g1g2[0][3], g1g2[0][4], g1g2[0][5], g1g2[0][6], g1g2[0][7], g1g2[0][8], g1g2[0][9], g1g2[0][10]);
  printf("jE        %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", g1g2[1][0], g1g2[1][1], g1g2[1][2], g1g2[1][3], g1g2[1][4], g1g2[1][5], g1g2[1][6], g1g2[1][7], g1g2[1][8], g1g2[1][9], g1g2[1][10]);
  printf("jD        %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", g1g2[2][0], g1g2[2][1], g1g2[2][2], g1g2[2][3], g1g2[2][4], g1g2[2][5], g1g2[2][6], g1g2[2][7], g1g2[2][8], g1g2[2][9], g1g2[2][10]);
  printf("jp        %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", g1g2[3][0], g1g2[3][1], g1g2[3][2], g1g2[3][3], g1g2[3][4], g1g2[3][5], g1g2[3][6], g1g2[3][7], g1g2[3][8], g1g2[3][9], g1g2[3][10]);
  printf("jq        %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", g1g2[4][0], g1g2[4][1], g1g2[4][2], g1g2[4][3], g1g2[4][4], g1g2[4][5], g1g2[4][6], g1g2[4][7], g1g2[4][8], g1g2[4][9], g1g2[4][10]);
  printf("jr        %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", g1g2[5][0], g1g2[5][1], g1g2[5][2], g1g2[5][3], g1g2[5][4], g1g2[5][5], g1g2[5][6], g1g2[5][7], g1g2[5][8], g1g2[5][9], g1g2[5][10]);
  printf("\n");
  printf("du_min  = %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", du_min[0], du_min[1], du_min[2], du_min[3], du_min[4], du_min[5], du_min[6], du_min[7], du_min[8], du_min[9], du_min[10]);
  printf("\n");
  printf("du_max  = %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", du_max[0], du_max[1], du_max[2], du_max[3], du_max[4], du_max[5], du_max[6], du_max[7], du_max[8], du_max[9], du_max[10]);
  printf("\n");
  printf("du_pref = %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", du_pref[0], du_pref[1], du_pref[2], du_pref[3], du_pref[4], du_pref[5], du_pref[6], du_pref[7], du_pref[8], du_pref[9], du_pref[10]);
  printf("\n");
  printf("Wu      = %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", Wu[0], Wu[1], Wu[2], Wu[3], Wu[4], Wu[5], Wu[6], Wu[7], Wu[8], Wu[9], Wu[10]);
  printf("\n");
  printf("du      = %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f %-13f\n", wls_du[0], wls_du[1], wls_du[2], wls_du[3], wls_du[4], wls_du[5], wls_du[6], wls_du[7], wls_du[8], wls_du[9], wls_du[10]);
  printf("Secondary cost = %f\n", SC);
}

/*
 * function to test wls for an overdetermined 4x6 (outputs x inputs) system
 */
void test_overdetermined(void)
{
  float u_min[INDI_NUM_ACT] = {0};
  float u_max[INDI_NUM_ACT] = {0};
  float du_min[INDI_NUM_ACT] = {0};
  float du_max[INDI_NUM_ACT] = {0};

  float u_p[INDI_NUM_ACT] = {0};

  float u_c[INDI_NUM_ACT] = {4614, 4210, 4210, 4614, 4210, 4210};

  printf("lower and upper bounds for du:\n");

  uint8_t k;
  for(k=0; k<INDI_NUM_ACT; k++) {
    u_max[k] = 9600 - u_min[k];

    du_min[k] = u_min[k] - u_c[k];
    du_max[k] = u_max[k] - u_c[k];

    u_p[k] = du_min[k];

    printf("%f ", du_min[k]);
    printf("%f \n", du_max[k]);
  }

  printf("\n");

  float g1g2[INDI_OUTPUTS][INDI_NUM_ACT] = {
    {  0.0,  -0.015,  0.015,  0.0,  -0.015,   0.015 },
    {  0.015,   -0.010, -0.010,   0.015,  -0.010,   -0.010 },
    {   0.103,   0.103,    0.103,   -0.103,    -0.103,    -0.103 },
    {-0.0009, -0.0009, -0.0009, -0.0009, -0.0009, -0.0009 }
  };

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {100, 100, 1, 10};

  // The control objective in array format
  float indi_v[INDI_OUTPUTS] = {240,  -240.5658,    600.0,    1.8532};
  float indi_du[INDI_NUM_ACT];

  // Initialize the array of pointers to the rows of g1g2
  float *Bwls[INDI_OUTPUTS];
  uint8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // WLS Control Allocator
  int num_iter =
    wls_alloc(indi_du, indi_v, du_min, du_max, Bwls, 0, 0, Wv, 0, u_p, 0, 10, INDI_NUM_ACT, INDI_OUTPUTS);

  printf("finished in %d iterations\n", num_iter);

  float nu_out[4] = {0.0f};
  calc_nu_out(Bwls, indi_du, nu_out);

  printf("du                 = %f, %f, %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3], indi_du[4], indi_du[5]);
  // Precomputed solution' in Matlab for this problem using lsqlin:
  printf("du (matlab_lsqlin) = %f, %f, %f, %f, %f, %f\n", -4614.0, 426.064612091305, 5390.0, -4614.0, -4210.0, 5390.0);
  printf("u = %f, %f, %f, %f, %f, %f\n", indi_du[0]+u_c[0], indi_du[1]+u_c[1], indi_du[2]+u_c[2], indi_du[3]+u_c[3], indi_du[4]+u_c[4], indi_du[5]+u_c[5]);
  printf("nu_in = %f, %f, %f, %f\n", indi_v[0], indi_v[1], indi_v[2], indi_v[3]);
  printf("nu_out = %f, %f, %f, %f\n", nu_out[0], nu_out[1], nu_out[2], nu_out[3]);
}

/*
 * function to test wls for an overdetermined 4x6 (outputs x inputs) system
 */
void test_overdetermined2(void)
{
  float u_min[INDI_NUM_ACT] = {0};
  float u_max[INDI_NUM_ACT] = {0};
  float du_min[INDI_NUM_ACT] = {1500.0, 1500.0, 1500.0, 1500.0, -9600.0, -9600.0, -9600.0, -9600.0};
  float du_max[INDI_NUM_ACT] = {9600.0, 9600.0, 9600.0, 9600.0, 9600.0, 9600.0, 9600.0, 9600.0};

  float u_p[INDI_NUM_ACT] = {0};

  printf("lower and upper bounds for du:\n");

  uint8_t k;
  for(k=0; k<INDI_NUM_ACT; k++) {
    printf("%f ", du_min[k]);
    printf("%f \n", du_max[k]);
  }

  printf("\n");

float g1g2[INDI_OUTPUTS][INDI_NUM_ACT] = {
   { 0.3310,   -0.3310,    0.8930,   -0.8930,         0,         0,         0,         0},
   { 0.4650,    0.4650,   -0.9300,   -0.9300,   -0.2240,   -0.2240,   -0.2240,   -0.2240},
   {      0,         0,         0,         0,    0.2150,   -0.2150,   -0.2150,    0.2150},
   {-0.3700,   -0.3700,   -0.5500,   -0.5500,         0,         0,         0,         0}
};
uint8_t i,j;
for (i=0; i< INDI_OUTPUTS;i++) {
  for (j=0; j< INDI_NUM_ACT;j++) {
    g1g2[i][j] = g1g2[i][j]/1000.0f;
  }
}

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {1000, 1000, 1, 100};

  // The control objective in array format
  float indi_v[INDI_OUTPUTS] = {0.0712,    0.5877,    0.1185,   -0.2238};
  // float indi_v[INDI_OUTPUTS] = {0.0738,    0.5876,    0.1187,   -0.2237};
  float indi_du[INDI_NUM_ACT];

  // Initialize the array of pointers to the rows of g1g2
  float *Bwls[INDI_OUTPUTS];
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  float indi_Wu[INDI_NUM_ACT] = {[0 ... INDI_NUM_ACT-1] = 1.0};

  // WLS Control Allocator
  int num_iter =
    wls_alloc(indi_du, indi_v, du_min, du_max, Bwls, 0, 0, Wv, indi_Wu, u_p, 0, 15, INDI_NUM_ACT, INDI_OUTPUTS);

  printf("finished in %d iterations\n", num_iter);

  float nu_out[4] = {0.0f};
  calc_nu_out(Bwls, indi_du, nu_out);

  printf("du                 = %f, %f, %f, %f, %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3], indi_du[4], indi_du[5], indi_du[6], indi_du[7]);
  // Precomputed solution' in Matlab for this problem using lsqlin:
  // printf("du (matlab_lsqlin) = %f, %f, %f, %f, %f, %f\n", -4614.0, 426.064612091305, 5390.0, -4614.0, -4210.0, 5390.0);
    printf("nu_in = %f, %f, %f, %f\n", indi_v[0], indi_v[1], indi_v[2], indi_v[3]);
  printf("nu_out = %f, %f, %f, %f\n", nu_out[0], nu_out[1], nu_out[2], nu_out[3]);
}

/*
 * Calculate the achieved control objective for some calculated control input
 */
void calc_nu_out(float** Bwls, float* du, float* nu_out) {

  for(int i=0; i<INDI_OUTPUTS; i++) {
    nu_out[i] = 0;
    for(int j=0; j<INDI_NUM_ACT; j++) {
      nu_out[i] += Bwls[i][j] * du[j];
    }
  }
}

