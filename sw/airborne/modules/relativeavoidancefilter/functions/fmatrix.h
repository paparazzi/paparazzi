#ifndef FMATRIX_H
#define FMATRIX_H

#ifndef ARM_COMPILER
#include <stdio.h> // needed for the printf statements
#endif /*ARM_COMPILER*/

extern void fmat_add(int n_row, int n_col, float* r, float* a, float* b);
extern void fmat_sub(int n_row, int n_col, float* r, float* a, float* b);
extern void fmat_transpose(int n_row, int n_col, float* r, float* a);
extern void fmat_scal_mult(int n_row, int n_col, float* r, float k, float* a);
extern void fmat_add_scal_mult(int n_row, int n_col, float* r, float*a, float k, float* b);
extern void fmat_mult(int n_rowa, int n_cola, int n_colb, float* r, float* a, float* b);

#ifndef ARM_COMPILER
extern void fmat_print(int n_row, int n_col, float* a);
#endif

extern void fmat_inverse(int n, float* inv, float* a);

extern void fmat_make_zeroes(float *matrix, int row, int col);
extern void fmat_make_identity(float *matrix, int n);


#endif /* FMATRIX_H */