/*
 * Copyright (C) 2008-2009 ENAC <poinix@gmail.com>
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

/** @file firmware/rotorcraft/guidance/guidance_h_ref.h
 *  Reference generation for horizontal guidance.
 *
 */

#ifndef GUIDANCE_H_REF_H
#define GUIDANCE_H_REF_H

#include "generated/airframe.h"
#include "inttypes.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"

/* update frequency                               */
#define B2_GH_FREQ_FRAC 9
#define B2_GH_FREQ (1<<B2_GH_FREQ_FRAC)

/* reference model accel in meters/sec2 (output) */
/* Q23.8 : accuracy 0.0039 , range 8388km/s2      */
/* int32_4_8_t */
extern struct Int32Vect2 b2_gh_accel_ref;
#define B2_GH_ACCEL_REF_FRAC 8

/* reference model speed in meters/sec (output)  */
/* Q14.17 : accuracy 0.0000076 , range 16384m/s2  */
extern struct Int32Vect2 b2_gh_speed_ref;
#define B2_GH_SPEED_REF_FRAC (B2_GH_ACCEL_REF_FRAC + B2_GH_FREQ_FRAC)

/* reference model position in meters (output)    */
/* Q37.26 :                                       */
extern struct Int64Vect2 b2_gh_pos_ref;
#define B2_GH_POS_REF_FRAC (B2_GH_SPEED_REF_FRAC + B2_GH_FREQ_FRAC)

/* Saturations definition */
#ifndef GUIDANCE_H_REF_MAX_ACCEL
#define GUIDANCE_H_REF_MAX_ACCEL ( tanf(RadOfDeg(30.))*9.81 )
#endif
#define B2_GH_MAX_ACCEL BFP_OF_REAL(GUIDANCE_H_REF_MAX_ACCEL, B2_GH_ACCEL_REF_FRAC)

#ifndef GUIDANCE_H_REF_MAX_SPEED
#define GUIDANCE_H_REF_MAX_SPEED ( 5. )
#endif
#define B2_GH_MAX_SPEED BFP_OF_REAL(GUIDANCE_H_REF_MAX_SPEED, B2_GH_SPEED_REF_FRAC)

/* second order model natural frequency and damping */
#ifndef GUIDANCE_H_REF_OMEGA
#define GUIDANCE_H_REF_OMEGA RadOfDeg(67.)
#endif
#ifndef GUIDANCE_H_REF_ZETA
#define GUIDANCE_H_REF_ZETA  0.85
#endif
#define B2_GH_ZETA_OMEGA_FRAC 10
#define B2_GH_ZETA_OMEGA BFP_OF_REAL((GUIDANCE_H_REF_ZETA*GUIDANCE_H_REF_OMEGA), B2_GH_ZETA_OMEGA_FRAC)
#define B2_GH_OMEGA_2_FRAC 7
#define B2_GH_OMEGA_2    BFP_OF_REAL((GUIDANCE_H_REF_OMEGA*GUIDANCE_H_REF_OMEGA), B2_GH_OMEGA_2_FRAC)

/* first order time constant */
#define B2_GH_REF_THAU_F  0.5
#define B2_GH_REF_INV_THAU_FRAC 16
#define B2_GH_REF_INV_THAU  BFP_OF_REAL((1./B2_GH_REF_THAU_F), B2_GH_REF_INV_THAU_FRAC)

#ifdef GUIDANCE_H_C
static inline void b2_gh_set_ref(struct Int32Vect2 pos, struct Int32Vect2 speed, struct Int32Vect2 accel);
static inline void b2_gh_update_ref_from_pos_sp(struct Int32Vect2 pos_sp);
static inline void b2_gh_update_ref_from_speed_sp(struct Int32Vect2 speed_sp);

struct Int64Vect2 b2_gh_pos_ref;
struct Int32Vect2 b2_gh_speed_ref;
struct Int32Vect2 b2_gh_accel_ref;

static inline void b2_gh_set_ref(struct Int32Vect2 pos, struct Int32Vect2 speed, struct Int32Vect2 accel) {
  struct Int64Vect2 new_pos;
  new_pos.x = ((int64_t)pos.x)<<(B2_GH_POS_REF_FRAC - INT32_POS_FRAC);
  new_pos.y = ((int64_t)pos.y)<<(B2_GH_POS_REF_FRAC - INT32_POS_FRAC);
  b2_gh_pos_ref = new_pos;
  INT32_VECT2_RSHIFT(b2_gh_speed_ref, speed, (INT32_SPEED_FRAC - B2_GH_SPEED_REF_FRAC));
  INT32_VECT2_RSHIFT(b2_gh_accel_ref, accel, (INT32_ACCEL_FRAC - B2_GH_ACCEL_REF_FRAC));
}

static inline void b2_gh_update_ref_from_pos_sp(struct Int32Vect2 pos_sp) {

  VECT2_ADD(b2_gh_pos_ref, b2_gh_speed_ref);
  VECT2_ADD(b2_gh_speed_ref, b2_gh_accel_ref);

  // compute the "speed part" of accel = -2*zeta*omega*speed -omega^2(pos - pos_sp)
  struct Int32Vect2 speed;
  INT32_VECT2_RSHIFT(speed, b2_gh_speed_ref, (B2_GH_SPEED_REF_FRAC - B2_GH_ACCEL_REF_FRAC));
  VECT2_SMUL(speed, speed, -2*B2_GH_ZETA_OMEGA);
  INT32_VECT2_RSHIFT(speed, speed, B2_GH_ZETA_OMEGA_FRAC);
  // compute pos error in pos_sp resolution
  struct Int32Vect2 pos_err;
  INT32_VECT2_RSHIFT(pos_err, b2_gh_pos_ref, (B2_GH_POS_REF_FRAC - INT32_POS_FRAC));
  VECT2_DIFF(pos_err, pos_err, pos_sp);
  // convert to accel resolution
  INT32_VECT2_RSHIFT(pos_err, pos_err, (INT32_POS_FRAC - B2_GH_ACCEL_REF_FRAC));
  // compute the "pos part" of accel
  struct Int32Vect2 pos;
  VECT2_SMUL(pos, pos_err, (-B2_GH_OMEGA_2));
  INT32_VECT2_RSHIFT(pos, pos, B2_GH_OMEGA_2_FRAC);
  // sum accel
  VECT2_SUM(b2_gh_accel_ref, speed, pos);

  /* Saturate accelerations */
  VECT2_STRIM(b2_gh_accel_ref, -B2_GH_MAX_ACCEL, B2_GH_MAX_ACCEL);

  /* Saturate speed and adjust acceleration accordingly */
  if (b2_gh_speed_ref.x <= -B2_GH_MAX_SPEED) {
    b2_gh_speed_ref.x = -B2_GH_MAX_SPEED;
    if (b2_gh_accel_ref.x < 0)
      b2_gh_accel_ref.x = 0;
  }
  else if (b2_gh_speed_ref.x >= B2_GH_MAX_SPEED) {
    b2_gh_speed_ref.x = B2_GH_MAX_SPEED;
    if (b2_gh_accel_ref.x > 0)
      b2_gh_accel_ref.x = 0;
  }
  if (b2_gh_speed_ref.y <= -B2_GH_MAX_SPEED) {
    b2_gh_speed_ref.y = -B2_GH_MAX_SPEED;
    if (b2_gh_accel_ref.y < 0)
      b2_gh_accel_ref.y = 0;
  }
  else if (b2_gh_speed_ref.y >= B2_GH_MAX_SPEED) {
    b2_gh_speed_ref.y = B2_GH_MAX_SPEED;
    if (b2_gh_accel_ref.y > 0)
      b2_gh_accel_ref.y = 0;
  }
}


static inline void b2_gh_update_ref_from_speed_sp(struct Int32Vect2 speed_sp) {

  VECT2_ADD(b2_gh_pos_ref, b2_gh_speed_ref);
  VECT2_ADD(b2_gh_speed_ref, b2_gh_accel_ref);

  // compute speed error
  struct Int32Vect2 speed_err;
  INT32_VECT2_RSHIFT(speed_err, speed_sp, (INT32_SPEED_FRAC - B2_GH_SPEED_REF_FRAC));
  VECT2_DIFF(speed_err, b2_gh_speed_ref, speed_err);
  // convert to accel resolution
  INT32_VECT2_RSHIFT(speed_err, speed_err, (B2_GH_SPEED_REF_FRAC - B2_GH_ACCEL_REF_FRAC));
  // compute accel from speed_sp
  VECT2_SMUL(b2_gh_accel_ref, speed_err, -B2_GH_REF_INV_THAU);
  INT32_VECT2_RSHIFT(b2_gh_accel_ref, b2_gh_accel_ref, B2_GH_REF_INV_THAU_FRAC);

  /* Saturate accelerations */
  VECT2_STRIM(b2_gh_accel_ref, -B2_GH_MAX_ACCEL, B2_GH_MAX_ACCEL);

  /* Saturate speed and adjust acceleration accordingly */
  if (b2_gh_speed_ref.x <= -B2_GH_MAX_SPEED) {
    b2_gh_speed_ref.x = -B2_GH_MAX_SPEED;
    if (b2_gh_accel_ref.x < 0)
      b2_gh_accel_ref.x = 0;
  }
  else if (b2_gh_speed_ref.x >= B2_GH_MAX_SPEED) {
    b2_gh_speed_ref.x = B2_GH_MAX_SPEED;
    if (b2_gh_accel_ref.x > 0)
      b2_gh_accel_ref.x = 0;
  }
  if (b2_gh_speed_ref.y <= -B2_GH_MAX_SPEED) {
    b2_gh_speed_ref.y = -B2_GH_MAX_SPEED;
    if (b2_gh_accel_ref.y < 0)
      b2_gh_accel_ref.y = 0;
  }
  else if (b2_gh_speed_ref.y >= B2_GH_MAX_SPEED) {
    b2_gh_speed_ref.y = B2_GH_MAX_SPEED;
    if (b2_gh_accel_ref.y > 0)
      b2_gh_accel_ref.y = 0;
  }
}

#endif /* GUIDANCE_H_C */

#endif /* GUIDANCE_H_REF_H */
