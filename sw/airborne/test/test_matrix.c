

#include "math/pprz_simple_matrix.h"
#include "pprz_simple_matrix_print.h"



int main(int argc, char **argv)
{

  float A[2][3] = {{ 1., 2., 3.},
    { 4., 5., 6.}
  };

  float B[3][2] = {{ 1., 2.},
    { 3., 4.},
    { 5., 6.}
  };

  float C[2][3] = {{ 1., 2., 3.},
    { 4., 5., 6.}
  };

  float D[2][2];

  float E[3][3] = {{ 1., 2., 3.},
    { 5., 1., 3.},
    { 3., 4., 6.}
  };

  float F[3][3];

  printf("\n");
  MAT_PRINT(2, 3, A);
  printf("\n");
  MAT_PRINT(3, 2, B);
  printf("\n");

  MAT_MUL_T(2, 3, 2, D, A, C);
  MAT_PRINT(2, 2, D);
  printf("\n");

  MAT_INV33(F, E);
  MAT_PRINT(3, 3, F);
  printf("\n");


  return 0;
}
