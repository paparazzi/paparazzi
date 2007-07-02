#include "tilt_utils.h"

#include <stdio.h>

void tilt_init(struct tilt_data* td, int len, double* X) {
  /* initialisation : assume vehicle is still */
  int iter;
  X[0] = 0.;
  X[1] = 0.;
  for (iter=0; iter < len; iter++) {
    X[0] += td->m_angle[iter];
    X[1] += td->gyro[iter];
  }
  X[0] /= (double)len;
  X[1] /= (double)len;
  printf("initialisation angle %f bias %f\n", X[0], X[1]);
}
