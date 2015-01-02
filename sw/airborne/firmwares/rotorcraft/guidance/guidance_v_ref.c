/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2013 Gautier Hattenberger
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

/** @file firmwares/rotorcraft/guidance/guidance_v_ref.c
 *  Reference generation for vertical guidance.
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_v_ref.h"
#include "generated/airframe.h"

/** reference model vertical accel in meters/s^2 (output)
 *  fixed point representation with #GV_ZDD_REF_FRAC
 *  Q23.8 : accuracy 0.0039 , range 8388km/s^2
 */
int32_t gv_zdd_ref;

/** reference model vertical speed in meters/sec (output)
 *  fixed point representation with #GV_ZD_REF_FRAC
 *  Q14.17 : accuracy 0.0000076 , range 16384m/s2
 */
int32_t gv_zd_ref;

/** reference model altitude in meters (output)
 *  fixed point representation with #GV_Z_REF_FRAC
 *  Q37.26 :
 */
int64_t gv_z_ref;


/* Saturations definition */
#ifndef GUIDANCE_V_REF_MIN_ZDD
#define GUIDANCE_V_REF_MIN_ZDD (-2.0*9.81)
#endif
#define GV_MIN_ZDD BFP_OF_REAL(GUIDANCE_V_REF_MIN_ZDD, GV_ZDD_REF_FRAC)

#ifndef GUIDANCE_V_REF_MAX_ZDD
#define GUIDANCE_V_REF_MAX_ZDD ( 0.8*9.81)
#endif
#define GV_MAX_ZDD BFP_OF_REAL(GUIDANCE_V_REF_MAX_ZDD, GV_ZDD_REF_FRAC)

/** maximum distance altitude setpoint is advanced in climb mode */
#ifndef GUIDANCE_V_REF_MAX_Z_DIFF
#define GUIDANCE_V_REF_MAX_Z_DIFF 2.0
#endif
#define GV_MAX_Z_DIFF BFP_OF_REAL(GUIDANCE_V_REF_MAX_Z_DIFF, GV_Z_REF_FRAC)

#define GV_MIN_ZD  BFP_OF_REAL(GUIDANCE_V_REF_MIN_ZD , GV_ZD_REF_FRAC)
#define GV_MAX_ZD  BFP_OF_REAL(GUIDANCE_V_REF_MAX_ZD , GV_ZD_REF_FRAC)

/* second order model natural frequency and damping */
#ifndef GUIDANCE_V_REF_OMEGA
#define GUIDANCE_V_REF_OMEGA RadOfDeg(100.)
#endif
#ifndef GUIDANCE_V_REF_ZETA
#define GUIDANCE_V_REF_ZETA  0.85
#endif
#define GV_ZETA_OMEGA_FRAC 10
#define GV_ZETA_OMEGA BFP_OF_REAL((GUIDANCE_V_REF_ZETA*GUIDANCE_V_REF_OMEGA), GV_ZETA_OMEGA_FRAC)
#define GV_OMEGA_2_FRAC 7
#define GV_OMEGA_2    BFP_OF_REAL((GUIDANCE_V_REF_OMEGA*GUIDANCE_V_REF_OMEGA), GV_OMEGA_2_FRAC)

/* first order time constant */
#define GV_REF_THAU_F  0.25
#define GV_REF_INV_THAU_FRAC 16
#define GV_REF_INV_THAU  BFP_OF_REAL((1./0.25), GV_REF_INV_THAU_FRAC)

void gv_set_ref(int32_t alt, int32_t speed, int32_t accel)
{
  int64_t new_z = ((int64_t)alt) << (GV_Z_REF_FRAC - INT32_POS_FRAC);
  gv_z_ref   = new_z;
  gv_zd_ref  = speed >> (INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
  gv_zdd_ref = accel >> (INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
}

void gv_update_ref_from_z_sp(int32_t z_sp)
{

  gv_z_ref  += gv_zd_ref;
  gv_zd_ref += gv_zdd_ref;

  // compute the "speed part" of zdd = -2*zeta*omega*zd -omega^2(z_sp - z)
  int32_t zd_zdd_res = gv_zd_ref >> (GV_ZD_REF_FRAC - GV_ZDD_REF_FRAC);
  int32_t zdd_speed = ((int32_t)(-2 * GV_ZETA_OMEGA) * zd_zdd_res) >> (GV_ZETA_OMEGA_FRAC);
  // compute z error in z_sp resolution
  int32_t z_err_sp = z_sp - (int32_t)(gv_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC));
  // convert to accel resolution
  int32_t z_err_accel = z_err_sp >> (INT32_POS_FRAC - GV_ZDD_REF_FRAC);
  int32_t zdd_pos = ((int32_t)(GV_OMEGA_2) * z_err_accel) >> GV_OMEGA_2_FRAC;
  gv_zdd_ref = zdd_speed + zdd_pos;

  /* Saturate accelerations */
  Bound(gv_zdd_ref, GV_MIN_ZDD, GV_MAX_ZDD);

  /* Saturate speed and adjust acceleration accordingly */
  if (gv_zd_ref <= GV_MIN_ZD) {
    gv_zd_ref = GV_MIN_ZD;
    if (gv_zdd_ref < 0) {
      gv_zdd_ref = 0;
    }
  } else if (gv_zd_ref >= GV_MAX_ZD) {
    gv_zd_ref = GV_MAX_ZD;
    if (gv_zdd_ref > 0) {
      gv_zdd_ref = 0;
    }
  }
}


void gv_update_ref_from_zd_sp(int32_t zd_sp, int32_t z_pos)
{

  gv_z_ref  += gv_zd_ref;
  gv_zd_ref += gv_zdd_ref;

  /* limit z_ref to GUIDANCE_V_REF_MAX_Z_DIFF from current z pos */
  int64_t cur_z = ((int64_t)z_pos) << (GV_Z_REF_FRAC - INT32_POS_FRAC);
  Bound(gv_z_ref, cur_z - GV_MAX_Z_DIFF, cur_z + GV_MAX_Z_DIFF);

  int32_t zd_err = gv_zd_ref - (zd_sp >> (INT32_SPEED_FRAC - GV_ZD_REF_FRAC));
  int32_t zd_err_zdd_res = zd_err >> (GV_ZD_REF_FRAC - GV_ZDD_REF_FRAC);
  gv_zdd_ref = (-(int32_t)GV_REF_INV_THAU * zd_err_zdd_res) >> GV_REF_INV_THAU_FRAC;

  /* Saturate accelerations */
  Bound(gv_zdd_ref, GV_MIN_ZDD, GV_MAX_ZDD);

  /* Saturate speed and adjust acceleration accordingly */
  if (gv_zd_ref <= GV_MIN_ZD) {
    gv_zd_ref = GV_MIN_ZD;
    if (gv_zdd_ref < 0) {
      gv_zdd_ref = 0;
    }
  } else if (gv_zd_ref >= GV_MAX_ZD) {
    gv_zd_ref = GV_MAX_ZD;
    if (gv_zdd_ref > 0) {
      gv_zdd_ref = 0;
    }
  }
}

