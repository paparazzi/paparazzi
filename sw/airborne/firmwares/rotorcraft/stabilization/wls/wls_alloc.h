
//#include <stdio.h>
//#include <stdlib>
//#include <iostream>
//#include <cstring>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include "qr_solve.h"
#include "r8lib_min.h"
#ifndef DBL_EPSILON
#define DBL_EPSILON 2.2204460492503131e-16
#endif

void qr_solve_wrapper(int m, int n, float** A, float* b, float* x);

int wls_alloc(float* u, float* v, float* umin, float* umax, float** B,
              int n_u, int n_w, float* u_guess, float* W_init, float* Wv,
              float* Wu, float* ud, float gamma, int imax);
