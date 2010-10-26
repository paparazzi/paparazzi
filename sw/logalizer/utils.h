#ifndef UTILS_H
#define UTILS_H

#include <math.h>

extern void htond (double *x);
extern void htonf (float *x);

#define RAD_OF_DEG(d) (d*M_PI/180.)
#define NORM_ANGLE_RAD(r) {			\
    while (r>2*M_PI)				\
      r -= 2*M_PI;				\
    while (r<0)					\
      r += 2*M_PI;				\
  }

#define NORM_ANGLE_DEG(d) {			\
    while (d>180)				\
      d -= 360;					\
    while (d<=-180)				\
      d += 360;					\
  }


#endif /* UTILS_H */
