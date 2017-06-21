#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_int.h"
#include "pprz_algebra_print.h"
#include "firmwares/rotorcraft/stabilization/wls/wls_alloc.h"

#define INDI_OUTPUTS 4
#define INDI_NUM_ACT 4

int main(int argc, char **argv)
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
    wls_alloc(indi_du, indi_v, u_min, u_max, Bwls, INDI_NUM_ACT, INDI_OUTPUTS, 0, 0, Wv, 0, 0, 10000, 10);
  printf("du = %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3]);
}
