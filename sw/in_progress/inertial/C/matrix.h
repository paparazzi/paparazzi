#ifndef MATRIX_H
#define MATRIX_H

extern void mat_add(int n_row, int n_col, double* r, double* a, double* b);
extern void mat_sub(int n_row, int n_col, double* r, double* a, double* b);
extern void mat_transpose(int n_row, int n_col, double* r, double* a);
extern void mat_scal_mult(int n_row, int n_col, double* r, double k, double* a);
extern void mat_add_scal_mult(int n_row, int n_col, double* r, double*a, double k, double* b);
extern void mat_mult(int n_rowa, int n_cola, int n_colb, double* r, double* a, double* b);

extern void mat_print(int n_row, int n_col, double* a);

#endif /* MATRIX_H */
