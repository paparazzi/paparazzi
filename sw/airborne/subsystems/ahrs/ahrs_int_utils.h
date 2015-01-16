/*
 * Copyright (C) 2009 Felix Ruess <felix.ruess@gmail.com>
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/ahrs/ahrs_int_utils.h
 *
 * Utility functions for fixed point AHRS implementations.
 *
 */

#ifndef AHRS_INT_UTILS_H
#define AHRS_INT_UTILS_H

//#include "../../test/pprz_algebra_print.h"
#include "math/pprz_algebra_int.h"

#include "subsystems/ahrs/ahrs_magnetic_field_model.h"

#include "subsystems/ahrs/ahrs_float_utils.h"

static inline void ahrs_int_get_euler_from_accel_mag(struct Int32Eulers *e, struct Int32Vect3 *accel,
    struct Int32Vect3 *mag)
{
  //  DISPLAY_INT32_VECT3("# accel", (*accel));
  const float fphi = atan2f(-accel->y, -accel->z);
  //  printf("# atan float %f\n", DegOfRad(fphi));
  e->phi = ANGLE_BFP_OF_REAL(fphi);

  int32_t cphi;
  PPRZ_ITRIG_COS(cphi, e->phi);
  int32_t cphi_ax = -INT_MULT_RSHIFT(cphi, accel->x, INT32_TRIG_FRAC);
  const float ftheta = atan2f(-cphi_ax, -accel->z);
  e->theta = ANGLE_BFP_OF_REAL(ftheta);

  int32_t sphi;
  PPRZ_ITRIG_SIN(sphi, e->phi);
  int32_t stheta;
  PPRZ_ITRIG_SIN(stheta, e->theta);
  int32_t ctheta;
  PPRZ_ITRIG_COS(ctheta, e->theta);

  int32_t sphi_stheta = (sphi * stheta) >> INT32_TRIG_FRAC;
  int32_t cphi_stheta = (cphi * stheta) >> INT32_TRIG_FRAC;
  //int32_t sphi_ctheta = (sphi*ctheta)>>INT32_TRIG_FRAC;
  //int32_t cphi_ctheta = (cphi*ctheta)>>INT32_TRIG_FRAC;

  const int32_t mn = ctheta * mag->x + sphi_stheta * mag->y + cphi_stheta * mag->z;
  const int32_t me = 0      * mag->x + cphi        * mag->y - sphi        * mag->z;
  //const int32_t md =
  //  -stheta     * imu.mag.x +
  //  sphi_ctheta * imu.mag.y +
  //  cphi_ctheta * imu.mag.z;
  //  float m_psi = -atan2(me, mn);
  const float mag_dec = atan2(-AHRS_H_Y, AHRS_H_X);
  const float fpsi = atan2f(-me, mn) - mag_dec;
  e->psi = ANGLE_BFP_OF_REAL(fpsi);
  INT32_ANGLE_NORMALIZE(e->psi);

}

static inline void ahrs_int_get_quat_from_accel(struct Int32Quat *q, struct Int32Vect3 *accel)
{
  struct FloatQuat q_f;
  ahrs_float_get_quat_from_accel(&q_f, accel);
  QUAT_BFP_OF_REAL(*q, q_f);
}

static inline void ahrs_int_get_quat_from_accel_mag(struct Int32Quat *q, struct Int32Vect3 *accel,
    struct Int32Vect3 *mag)
{

  struct FloatQuat q_f;
  ahrs_float_get_quat_from_accel_mag(&q_f, accel, mag);
  QUAT_BFP_OF_REAL(*q, q_f);
}

#endif /* AHRS_INT_UTILS_H */
