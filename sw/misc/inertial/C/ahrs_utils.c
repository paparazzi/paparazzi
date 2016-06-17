#include "ahrs_utils.h"

#include <math.h>
#include <stdio.h>

extern double phi_of_accel(double an, double ae, double ad) {
  return atan2(ae, ad);
}

extern double theta_of_accel(double an, double ae, double ad) {
  const double g2 = an * an + ae * ae + ad * ad;
  return -asin(an / sqrt(g2));
}

extern double psi_of_mag(double phi, double theta, double mx, double my, double mz) {
  const float cphi   = cos( phi );
  const float sphi   = sin( phi );
  const float ctheta = cos( theta );
  const float stheta = sin( theta );
  const float mn =
           ctheta * mx +
    sphi * stheta * my +
    cphi * stheta * mz;
  const float me =
     0    * mx+
     cphi * my+
    -sphi * mz;
  return -atan2( me, mn );
}

void quat_of_eulers (double* quat, double phi, double theta, double psi ) {
  const float phi2     = phi / 2.0;
  const float theta2   = theta / 2.0;
  const float psi2     = psi / 2.0;

  const float sinphi2   = sin( phi2 );
  const float cosphi2   = cos( phi2 );

  const float sintheta2 = sin( theta2 );
  const float costheta2 = cos( theta2 );

  const float sinpsi2   = sin( psi2 );
  const float cospsi2   = cos( psi2 );

  quat[0] =  cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2;
  quat[1] = -cosphi2 * sintheta2 * sinpsi2 + sinphi2 * costheta2 * cospsi2;
  quat[2] =  cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2;
  quat[3] =  cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2;
}

void eulers_of_quat(double* euler, double* quat) {
  double dcm00 = 1.0-2*(quat[2]*quat[2] + quat[3]*quat[3]);
  double dcm01 =     2*(quat[1]*quat[2] + quat[0]*quat[3]);
  double dcm02 =     2*(quat[1]*quat[3] - quat[0]*quat[2]);
  double dcm12 =     2*(quat[2]*quat[3] + quat[0]*quat[1]);
  double dcm22 = 1.0-2*(quat[1]*quat[1] + quat[2]*quat[2]);

  euler[0] = atan2( dcm12, dcm22 );
  euler[1] = -asin( dcm02 );
  euler[2] = atan2( dcm01, dcm00 );
}

void norm_quat(double* quat) {
  double  mag = quat[0]*quat[0] + quat[1]*quat[1] +
                quat[2]*quat[2] + quat[3]*quat[3];
  mag = sqrt( mag );
  quat[0] /= mag;
  quat[1] /= mag;
  quat[2] /= mag;
  quat[3] /= mag;
}

void ahrs_euler_init(struct ahrs_data* ad, int len, double* X) {
  double init_phi = 0.;
  double init_theta = 0.;
  double init_mx = 0.;
  double init_my = 0.;
  double init_mz = 0.;
  X[3] = 0.;
  X[4] = 0.;
  X[5] = 0.;
  int iter;
  for (iter = 0; iter < len; iter++) {
    /* average attitude */
    init_phi += phi_of_accel(ad->accel_x[iter], ad->accel_y[iter], ad->accel_z[iter]);
    init_theta += theta_of_accel(ad->accel_x[iter], ad->accel_y[iter], ad->accel_z[iter]);
    init_mx += ad->mag_x[iter];
    init_my += ad->mag_y[iter];
    init_mz += ad->mag_z[iter];
    /* sum gyros */
    X[3] += ad->gyro_p[iter];
    X[4] += ad->gyro_q[iter];
    X[5] += ad->gyro_r[iter];
  }
  init_phi /= (double)len;
  init_theta /= (double)len;
  init_mx /= (double)len;
  init_my /= (double)len;
  init_mz /= (double)len;
  double init_psi = psi_of_mag(init_phi, init_theta, init_mx, init_my, init_mz);
  X[0] = init_phi;
  X[1] = init_theta;
  X[2] = init_psi;
  X[3] /= (double)len;
  X[4] /= (double)len;
  X[5] /= (double)len;
  printf("ahrs init : eulers [%f %f %f] biases [%f %f %f]\n", init_phi, init_theta, init_psi, X[3], X[4], X[5]);
}

void ahrs_quat_init(struct ahrs_data* ad, int len, double* X) {
  ahrs_euler_init(ad, len, X);

  X[6] = X[5];
  X[5] = X[4];
  X[4] = X[3];
  quat_of_eulers(X, X[0], X[1], X[2]);

}
