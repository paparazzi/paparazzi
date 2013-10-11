/*
 * Copyright (C) 2008-2013 The Paparazzi Team
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

/** @file firmwares/rotorcraft/guidance/guidance_h_ref.c
 *  Reference generation for horizontal guidance.
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
#include "generated/airframe.h"

/** Reference model acceleration.
 * in meters/sec2 (output)
 * fixed point representation: Q23.8
 * accuracy 0.0039, range 8388km/s2
 */
struct Int32Vect2 gh_accel_ref;

/** Reference model speed.
 * in meters/sec
 * with fixedpoint representation: Q14.17
 * accuracy 0.0000076 , range 16384m/s
 */
struct Int32Vect2 gh_speed_ref;

/* Reference model position.
 * in meters
 * with fixedpoint representation: Q37.26
 */
struct Int64Vect2 gh_pos_ref;


static const int32_t gh_max_accel =  BFP_OF_REAL(GUIDANCE_H_REF_MAX_ACCEL, GH_ACCEL_REF_FRAC);

/** @todo GH_MAX_SPEED must be limited to 2^14 to avoid overflow */
#define GH_MAX_SPEED_REF_FRAC 7
static const int32_t gh_max_speed = BFP_OF_REAL(GUIDANCE_H_REF_MAX_SPEED, GH_MAX_SPEED_REF_FRAC);

/** second order model natural frequency */
#ifndef GUIDANCE_H_REF_OMEGA
#define GUIDANCE_H_REF_OMEGA RadOfDeg(67.)
#endif
/** second order model damping */
#ifndef GUIDANCE_H_REF_ZETA
#define GUIDANCE_H_REF_ZETA  0.85
#endif
#define GH_ZETA_OMEGA_FRAC 10
#define GH_OMEGA_2_FRAC 7
static const int32_t gh_zeta_omega = BFP_OF_REAL((GUIDANCE_H_REF_ZETA*GUIDANCE_H_REF_OMEGA), GH_ZETA_OMEGA_FRAC);
static const int32_t gh_omega_2= BFP_OF_REAL((GUIDANCE_H_REF_OMEGA*GUIDANCE_H_REF_OMEGA), GH_OMEGA_2_FRAC);

/** first order time constant */
#ifndef GUIDANCE_H_REF_TAU
#define GUIDANCE_H_REF_TAU 0.5
#endif
#define GH_REF_INV_TAU_FRAC 16
static const int32_t gh_ref_inv_tau = BFP_OF_REAL((1./GUIDANCE_H_REF_TAU), GH_REF_INV_TAU_FRAC);

static struct Int32Vect2 gh_max_speed_ref;
static struct Int32Vect2 gh_max_accel_ref;

static int32_t route_ref;
static int32_t s_route_ref;
static int32_t c_route_ref;

static void gh_compute_route_ref(struct Int32Vect2* ref_vector);
static void gh_compute_ref_max(struct Int32Vect2* ref_vector);
static void gh_compute_ref_max_accel(struct Int32Vect2* ref_vector);
static void gh_compute_ref_max_speed(struct Int32Vect2* ref_vector);
static void gh_saturate_ref_accel(void);
static void gh_saturate_ref_speed(void);

void gh_set_ref(struct Int32Vect2 pos, struct Int32Vect2 speed, struct Int32Vect2 accel) {
  struct Int64Vect2 new_pos;
  new_pos.x = ((int64_t)pos.x)<<(GH_POS_REF_FRAC - INT32_POS_FRAC);
  new_pos.y = ((int64_t)pos.y)<<(GH_POS_REF_FRAC - INT32_POS_FRAC);
  gh_pos_ref = new_pos;
  INT32_VECT2_RSHIFT(gh_speed_ref, speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
  INT32_VECT2_RSHIFT(gh_accel_ref, accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
}

void gh_update_ref_from_pos_sp(struct Int32Vect2 pos_sp) {

  VECT2_ADD(gh_pos_ref, gh_speed_ref);
  VECT2_ADD(gh_speed_ref, gh_accel_ref);

  // compute the "speed part" of accel = -2*zeta*omega*speed -omega^2(pos - pos_sp)
  struct Int32Vect2 speed;
  INT32_VECT2_RSHIFT(speed, gh_speed_ref, (GH_SPEED_REF_FRAC - GH_ACCEL_REF_FRAC));
  VECT2_SMUL(speed, speed, -2 * gh_zeta_omega);
  INT32_VECT2_RSHIFT(speed, speed, GH_ZETA_OMEGA_FRAC);
  // compute pos error in pos_sp resolution
  struct Int32Vect2 pos_err;
  INT32_VECT2_RSHIFT(pos_err, gh_pos_ref, (GH_POS_REF_FRAC - INT32_POS_FRAC));
  VECT2_DIFF(pos_err, pos_err, pos_sp);
  // convert to accel resolution
  INT32_VECT2_RSHIFT(pos_err, pos_err, (INT32_POS_FRAC - GH_ACCEL_REF_FRAC));
  // compute the "pos part" of accel
  struct Int32Vect2 pos;
  VECT2_SMUL(pos, pos_err, -gh_omega_2);
  INT32_VECT2_RSHIFT(pos, pos, GH_OMEGA_2_FRAC);
  // sum accel
  VECT2_SUM(gh_accel_ref, speed, pos);

  /* Compute max ref accel/speed along route before saturation */
  gh_compute_ref_max(&pos_err);

  gh_saturate_ref_accel();
  gh_saturate_ref_speed();
}


void gh_update_ref_from_speed_sp(struct Int32Vect2 speed_sp) {
/* WARNING: SPEED SATURATION UNTESTED */
  VECT2_ADD(gh_pos_ref, gh_speed_ref);
  VECT2_ADD(gh_speed_ref, gh_accel_ref);

  // compute speed error
  struct Int32Vect2 speed_err;
  INT32_VECT2_RSHIFT(speed_err, speed_sp, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
  VECT2_DIFF(speed_err, gh_speed_ref, speed_err);
  // convert to accel resolution
  INT32_VECT2_RSHIFT(speed_err, speed_err, (GH_SPEED_REF_FRAC - GH_ACCEL_REF_FRAC));
  // compute accel from speed_sp
  VECT2_SMUL(gh_accel_ref, speed_err, -gh_ref_inv_tau);
  INT32_VECT2_RSHIFT(gh_accel_ref, gh_accel_ref, GH_REF_INV_TAU_FRAC);

  /* Compute max ref accel/speed along route before saturation */
  gh_compute_ref_max_speed(&speed_sp);
  gh_compute_ref_max_accel(&speed_err);

  gh_saturate_ref_accel();
  gh_saturate_ref_speed();
}

static void gh_compute_route_ref(struct Int32Vect2* ref_vector) {
  float f_route_ref = atan2f(-ref_vector->y, -ref_vector->x);
  route_ref = ANGLE_BFP_OF_REAL(f_route_ref);
  /* Compute North and East route components */
  PPRZ_ITRIG_SIN(s_route_ref, route_ref);
  PPRZ_ITRIG_COS(c_route_ref, route_ref);
  c_route_ref = abs(c_route_ref);
  s_route_ref = abs(s_route_ref);
}

static void gh_compute_ref_max(struct Int32Vect2* ref_vector) {
  /* Compute route reference before saturation */
  if (ref_vector->x == 0 && ref_vector->y == 0) {
    gh_max_accel_ref.x = 0;
    gh_max_accel_ref.y = 0;
    gh_max_speed_ref.x = 0;
    gh_max_speed_ref.y = 0;
  }
  else {
    gh_compute_route_ref(ref_vector);
    /* Compute maximum acceleration*/
    gh_max_accel_ref.x = INT_MULT_RSHIFT(gh_max_accel, c_route_ref, INT32_TRIG_FRAC);
    gh_max_accel_ref.y = INT_MULT_RSHIFT(gh_max_accel, s_route_ref, INT32_TRIG_FRAC);
    /* Compute maximum speed*/
    gh_max_speed_ref.x = INT_MULT_RSHIFT(gh_max_speed, c_route_ref, INT32_TRIG_FRAC);
    gh_max_speed_ref.y = INT_MULT_RSHIFT(gh_max_speed, s_route_ref, INT32_TRIG_FRAC);
    /* restore gh_speed_ref range (Q14.17) */
    INT32_VECT2_LSHIFT(gh_max_speed_ref, gh_max_speed_ref, (GH_SPEED_REF_FRAC - GH_MAX_SPEED_REF_FRAC));
  }
}

static void gh_compute_ref_max_accel(struct Int32Vect2* ref_vector) {
  /* Compute route reference before saturation */
  if (ref_vector->x == 0 && ref_vector->y == 0) {
    gh_max_accel_ref.x = 0;
    gh_max_accel_ref.y = 0;
  }
  else {
    gh_compute_route_ref(ref_vector);
    /* Compute maximum acceleration*/
    gh_max_accel_ref.x = INT_MULT_RSHIFT(gh_max_accel, c_route_ref, INT32_TRIG_FRAC);
    gh_max_accel_ref.y = INT_MULT_RSHIFT(gh_max_accel, s_route_ref, INT32_TRIG_FRAC);
  }
}

static void gh_compute_ref_max_speed(struct Int32Vect2* ref_vector) {
  /* Compute route reference before saturation */
  if (ref_vector->x == 0 && ref_vector->y == 0) {
    gh_max_speed_ref.x = 0;
    gh_max_speed_ref.y = 0;
  }
  else {
    gh_compute_route_ref(ref_vector);
    /* Compute maximum speed*/
    gh_max_speed_ref.x = INT_MULT_RSHIFT(gh_max_speed, c_route_ref, INT32_TRIG_FRAC);
    gh_max_speed_ref.y = INT_MULT_RSHIFT(gh_max_speed, s_route_ref, INT32_TRIG_FRAC);
    /* restore gh_speed_ref range (Q14.17) */
    INT32_VECT2_LSHIFT(gh_max_speed_ref, gh_max_speed_ref, (GH_SPEED_REF_FRAC - GH_MAX_SPEED_REF_FRAC));
  }
}

/** saturate reference accelerations */
static void gh_saturate_ref_accel(void) {
  /* Saturate accelerations */
  if (gh_accel_ref.x <= -gh_max_accel_ref.x) {
    gh_accel_ref.x = -gh_max_accel_ref.x;
  }
  else if (gh_accel_ref.x >=  gh_max_accel_ref.x) {
    gh_accel_ref.x =  gh_max_accel_ref.x;
  }
  if (gh_accel_ref.y <= -gh_max_accel_ref.y) {
    gh_accel_ref.y = -gh_max_accel_ref.y;
  }
  else if (gh_accel_ref.y >= gh_max_accel_ref.y) {
    gh_accel_ref.y = gh_max_accel_ref.y;
  }
}

/** Saturate ref speed and adjust acceleration accordingly */
static void gh_saturate_ref_speed(void) {
  if (gh_speed_ref.x <= -gh_max_speed_ref.x) {
    gh_speed_ref.x = -gh_max_speed_ref.x;
    if (gh_accel_ref.x < 0)
      gh_accel_ref.x = 0;
  }
  else if (gh_speed_ref.x >=  gh_max_speed_ref.x) {
    gh_speed_ref.x =  gh_max_speed_ref.x;
    if (gh_accel_ref.x > 0)
      gh_accel_ref.x = 0;
  }
  if (gh_speed_ref.y <= -gh_max_speed_ref.y) {
    gh_speed_ref.y = -gh_max_speed_ref.y;
    if (gh_accel_ref.y < 0)
      gh_accel_ref.y = 0;
  }
  else if (gh_speed_ref.y >= gh_max_speed_ref.y) {
    gh_speed_ref.y = gh_max_speed_ref.y;
    if (gh_accel_ref.y > 0)
      gh_accel_ref.y = 0;
  }
}

