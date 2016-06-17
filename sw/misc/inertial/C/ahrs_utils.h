#ifndef AHRS_UTILS_H
#define AHRS_UTILS_H

#include "ahrs_data.h"

extern double phi_of_accel(double an, double ae, double ad);
extern double theta_of_accel(double an, double ae, double ad);
extern double psi_of_mag(double phi, double theta, double mx, double my, double mz);
extern void quat_of_eulers(double* quat, double phi, double theta, double psi );
extern void eulers_of_quat(double* euler, double* quat);
extern void norm_quat(double* quat);

extern void ahrs_quat_init(struct ahrs_data* ad, int len, double* X);
extern void ahrs_euler_init(struct ahrs_data* ad, int len, double* X);

#define WARP(x, y) {	 \
    while ( x < -y )     \
      x += 2 * y;	 \
    while ( x > y )	 \
      x -= 2 * y;	 \
}


#endif /* AHRS_UTILS_H */
