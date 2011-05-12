#ifndef AHRS_FLOAT_UTILS_H
#define AHRS_FLOAT_UTILS_H

#include "subsystems/ahrs/ahrs_magnetic_field_model.h"

static inline void ahrs_float_get_euler_from_accel_mag(struct FloatEulers* e, struct Int32Vect3* accel, struct Int32Vect3* mag) {
  /* get phi and theta from accelerometer */
  struct FloatVect3 accelf;
  ACCELS_FLOAT_OF_BFP(accelf, *accel);
  const float phi   = atan2f(-accelf.y, -accelf.z);
  const float cphi = cosf(phi);
  const float theta = atan2f(cphi*accelf.x, -accelf.z);

  /* get psi from magnetometer */
  /* project mag on local tangeant plane */
  struct FloatVect3 magf;
  MAGS_FLOAT_OF_BFP(magf, *mag);
  const float sphi   = sinf(phi);
  const float ctheta = cosf(theta);
  const float stheta = sinf(theta);
  const float mn = ctheta * magf.x + sphi*stheta*magf.y + cphi*stheta*magf.z;
  const float me =     0. * magf.x + cphi       *magf.y - sphi       *magf.z;
  float psi = -atan2f(me, mn) + atan2(AHRS_H_Y, AHRS_H_X);
  if (psi > M_PI) psi -= 2.*M_PI; if (psi < -M_PI) psi+= 2.*M_PI;
  EULERS_ASSIGN(*e, phi, theta, psi);

}

#endif /* AHRS_FLOAT_UTILS_H */
