/*
 * Copyright (C) Mario Coppola
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/relativelocalizationfilter/fmatrix.h"
 * @author Mario Coppola
 * C matrix functions
 */

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
extern void fmat_inverse(int n, float* inv, float* a);
extern void fmat_make_zeroes(float *matrix, int row, int col);
extern void fmat_make_identity(float *matrix, int n);

#ifndef ARM_COMPILER
extern void fmat_print(int n_row, int n_col, float* a);
#endif

#endif /* FMATRIX_H */