#ifndef PPRZ_SIMPLE_MATRIX_PRINT_H
#define PPRZ_SIMPLE_MATRIX_PRINT_H

#include <stdio.h>

#define MAT_PRINT(_i, _j, A) {      \
    for (int i=0; i<_i; i++) {      \
      for (int j=0; j<_j; j++)      \
        printf("%f ", A[i][j]);     \
      printf("\n");       \
    }           \
  }


#endif /* PPRZ_SIMPLE_MATRIX_PRINT_H */

