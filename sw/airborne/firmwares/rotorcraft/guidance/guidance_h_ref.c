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

struct GuidanceHRef gh_ref;

static const int32_t gh_max_accel = BFP_OF_REAL(GUIDANCE_H_REF_MAX_ACCEL, GH_ACCEL_REF_FRAC);

#define GH_MAX_SPEED_REF_FRAC 7

/** default second order model natural frequency */
#ifndef GUIDANCE_H_REF_OMEGA
#define GUIDANCE_H_REF_OMEGA RadOfDeg(67.)
#endif
/** default second order model damping */
#ifndef GUIDANCE_H_REF_ZETA
#define GUIDANCE_H_REF_ZETA  0.85
#endif

#define GH_ZETA_OMEGA_FRAC 10
#define GH_OMEGA_2_FRAC 7


/** first order time constant */
#ifndef GUIDANCE_H_REF_TAU
#define GUIDANCE_H_REF_TAU 0.5
#endif
#define GH_REF_INV_TAU_FRAC 16

static void gh_compute_route_ref(struct Int32Vect2 *ref_vector);
static void gh_compute_ref_max(struct Int32Vect2 *ref_vector);
static void gh_compute_ref_max_accel(struct Int32Vect2 *ref_vector);
static void gh_compute_ref_max_speed(struct Int32Vect2 *ref_vector);
static void gh_saturate_ref_accel(void);
static void gh_saturate_ref_speed(void);

void gh_ref_init(void)
{
  gh_ref.omega = GUIDANCE_H_REF_OMEGA;
  gh_ref.zeta = GUIDANCE_H_REF_ZETA;
  gh_ref.zeta_omega = BFP_OF_REAL((GUIDANCE_H_REF_ZETA * GUIDANCE_H_REF_OMEGA), GH_ZETA_OMEGA_FRAC);
  gh_ref.omega_2 = BFP_OF_REAL((GUIDANCE_H_REF_OMEGA * GUIDANCE_H_REF_OMEGA), GH_OMEGA_2_FRAC);
  gh_set_tau(GUIDANCE_H_REF_TAU);
  gh_set_max_speed(GUIDANCE_H_REF_MAX_SPEED);
}


float gh_set_max_speed(float max_speed)
{
  /* limit to 100m/s as int version would overflow at  2^14 = 128 m/s */
  gh_ref.max_speed = Min(fabs(max_speed), 100.0f);
  gh_ref.max_speed_int = BFP_OF_REAL(gh_ref.max_speed, GH_MAX_SPEED_REF_FRAC);
  return gh_ref.max_speed;
}

float gh_set_tau(float tau)
{
  gh_ref.tau = tau;
  Bound(gh_ref.tau, 0.01f, 2.0f);
  gh_ref.inv_tau = BFP_OF_REAL((1. / gh_ref.tau), GH_REF_INV_TAU_FRAC);
  return gh_ref.tau;
}

float gh_set_omega(float omega)
{
  gh_ref.omega = omega;
  Bound(gh_ref.omega, 0.1f, 5.0f);
  gh_ref.omega_2 = BFP_OF_REAL((gh_ref.omega * gh_ref.omega), GH_OMEGA_2_FRAC);
  gh_ref.zeta_omega = BFP_OF_REAL((gh_ref.zeta * gh_ref.omega), GH_ZETA_OMEGA_FRAC);
  return gh_ref.omega;
}

float gh_set_zeta(float zeta)
{
  gh_ref.zeta = zeta;
  Bound(gh_ref.zeta, 0.7f, 1.2f);
  gh_ref.zeta_omega = BFP_OF_REAL((gh_ref.zeta * gh_ref.omega), GH_ZETA_OMEGA_FRAC);
  return gh_ref.zeta;
}

void gh_set_ref(struct Int32Vect2 pos, struct Int32Vect2 speed, struct Int32Vect2 accel)
{
  struct Int64Vect2 new_pos;
  new_pos.x = ((int64_t)pos.x) << (GH_POS_REF_FRAC - INT32_POS_FRAC);
  new_pos.y = ((int64_t)pos.y) << (GH_POS_REF_FRAC - INT32_POS_FRAC);
  gh_ref.pos = new_pos;
  INT32_VECT2_RSHIFT(gh_ref.speed, speed, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
  INT32_VECT2_RSHIFT(gh_ref.accel, accel, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
}

void gh_update_ref_from_pos_sp(struct Int32Vect2 pos_sp)
{

  VECT2_ADD(gh_ref.pos, gh_ref.speed);
  VECT2_ADD(gh_ref.speed, gh_ref.accel);

  // compute the "speed part" of accel = -2*zeta*omega*speed -omega^2(pos - pos_sp)
  struct Int32Vect2 speed;
  INT32_VECT2_RSHIFT(speed, gh_ref.speed, (GH_SPEED_REF_FRAC - GH_ACCEL_REF_FRAC));
  VECT2_SMUL(speed, speed, -2 * gh_ref.zeta_omega);
  INT32_VECT2_RSHIFT(speed, speed, GH_ZETA_OMEGA_FRAC);
  // compute pos error in pos_sp resolution
  struct Int32Vect2 pos_err;
  INT32_VECT2_RSHIFT(pos_err, gh_ref.pos, (GH_POS_REF_FRAC - INT32_POS_FRAC));
  VECT2_DIFF(pos_err, pos_err, pos_sp);
  // convert to accel resolution
  INT32_VECT2_RSHIFT(pos_err, pos_err, (INT32_POS_FRAC - GH_ACCEL_REF_FRAC));
  // compute the "pos part" of accel
  struct Int32Vect2 pos;
  VECT2_SMUL(pos, pos_err, -gh_ref.omega_2);
  INT32_VECT2_RSHIFT(pos, pos, GH_OMEGA_2_FRAC);
  // sum accel
  VECT2_SUM(gh_ref.accel, speed, pos);

  /* Compute max ref accel/speed along route before saturation */
  gh_compute_ref_max(&pos_err);

  gh_saturate_ref_accel();
  gh_saturate_ref_speed();
}


void gh_update_ref_from_speed_sp(struct Int32Vect2 speed_sp)
{
  /* WARNING: SPEED SATURATION UNTESTED */
  VECT2_ADD(gh_ref.pos, gh_ref.speed);
  VECT2_ADD(gh_ref.speed, gh_ref.accel);

  // compute speed error
  struct Int32Vect2 speed_err;
  INT32_VECT2_RSHIFT(speed_err, speed_sp, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
  VECT2_DIFF(speed_err, gh_ref.speed, speed_err);
  // convert to accel resolution
  INT32_VECT2_RSHIFT(speed_err, speed_err, (GH_SPEED_REF_FRAC - GH_ACCEL_REF_FRAC));
  // compute accel from speed_sp
  VECT2_SMUL(gh_ref.accel, speed_err, -gh_ref.inv_tau);
  INT32_VECT2_RSHIFT(gh_ref.accel, gh_ref.accel, GH_REF_INV_TAU_FRAC);

  /* Compute max ref accel/speed along route before saturation */
  gh_compute_ref_max_speed(&speed_sp);
  gh_compute_ref_max_accel(&speed_err);

  gh_saturate_ref_accel();
  gh_saturate_ref_speed();
}

static void gh_compute_route_ref(struct Int32Vect2 *ref_vector)
{
  float f_route_ref = atan2f(-ref_vector->y, -ref_vector->x);
  gh_ref.route_ref = ANGLE_BFP_OF_REAL(f_route_ref);
  /* Compute North and East route components */
  PPRZ_ITRIG_SIN(gh_ref.s_route_ref, gh_ref.route_ref);
  PPRZ_ITRIG_COS(gh_ref.c_route_ref, gh_ref.route_ref);
  gh_ref.c_route_ref = abs(gh_ref.c_route_ref);
  gh_ref.s_route_ref = abs(gh_ref.s_route_ref);
}

static void gh_compute_ref_max(struct Int32Vect2 *ref_vector)
{
  /* Bound ref to max speed/accel along route reference angle.
   * If angle can't be computed, simply set both axes to max magnitude/sqrt(2).
   */
  if (ref_vector->x == 0 && ref_vector->y == 0) {
    gh_ref.max_accel.x = gh_ref.max_accel.y = gh_max_accel * 0.707;
    gh_ref.max_vel.x = gh_ref.max_vel.y = gh_ref.max_speed_int * 0.707;
  } else {
    gh_compute_route_ref(ref_vector);
    /* Compute maximum acceleration*/
    gh_ref.max_accel.x = INT_MULT_RSHIFT(gh_max_accel, gh_ref.c_route_ref, INT32_TRIG_FRAC);
    gh_ref.max_accel.y = INT_MULT_RSHIFT(gh_max_accel, gh_ref.s_route_ref, INT32_TRIG_FRAC);
    /* Compute maximum reference x/y velocity from absolute max_speed */
    gh_ref.max_vel.x = INT_MULT_RSHIFT(gh_ref.max_speed_int, gh_ref.c_route_ref, INT32_TRIG_FRAC);
    gh_ref.max_vel.y = INT_MULT_RSHIFT(gh_ref.max_speed_int, gh_ref.s_route_ref, INT32_TRIG_FRAC);
  }
  /* restore gh_ref.speed range (Q14.17) */
  INT32_VECT2_LSHIFT(gh_ref.max_vel, gh_ref.max_vel, (GH_SPEED_REF_FRAC - GH_MAX_SPEED_REF_FRAC));
}

static void gh_compute_ref_max_accel(struct Int32Vect2 *ref_vector)
{
  /* Bound ref to max accel along reference vector.
   * If angle can't be computed, simply set both axes to max magnitude/sqrt(2).
   */
  if (ref_vector->x == 0 && ref_vector->y == 0) {
    gh_ref.max_accel.x = gh_ref.max_accel.y = gh_max_accel * 0.707;
  } else {
    gh_compute_route_ref(ref_vector);
    /* Compute maximum acceleration*/
    gh_ref.max_accel.x = INT_MULT_RSHIFT(gh_max_accel, gh_ref.c_route_ref, INT32_TRIG_FRAC);
    gh_ref.max_accel.y = INT_MULT_RSHIFT(gh_max_accel, gh_ref.s_route_ref, INT32_TRIG_FRAC);
  }
}

static void gh_compute_ref_max_speed(struct Int32Vect2 *ref_vector)
{
  /* Bound ref to max speed along reference vector.
   * If angle can't be computed, simply set both axes to max magnitude/sqrt(2).
   */
  if (ref_vector->x == 0 && ref_vector->y == 0) {
    gh_ref.max_vel.x = gh_ref.max_vel.y = gh_ref.max_speed_int * 0.707;
  } else {
    gh_compute_route_ref(ref_vector);
    /* Compute maximum reference x/y velocity from absolute max_speed */
    gh_ref.max_vel.x = INT_MULT_RSHIFT(gh_ref.max_speed_int, gh_ref.c_route_ref, INT32_TRIG_FRAC);
    gh_ref.max_vel.y = INT_MULT_RSHIFT(gh_ref.max_speed_int, gh_ref.s_route_ref, INT32_TRIG_FRAC);
  }
  /* restore gh_ref.speed range (Q14.17) */
  INT32_VECT2_LSHIFT(gh_ref.max_vel, gh_ref.max_vel, (GH_SPEED_REF_FRAC - GH_MAX_SPEED_REF_FRAC));
}

/** saturate reference accelerations */
static void gh_saturate_ref_accel(void)
{
  /* Saturate accelerations */
  BoundAbs(gh_ref.accel.x, gh_ref.max_accel.x);
  BoundAbs(gh_ref.accel.y, gh_ref.max_accel.y);
}

/** Saturate ref speed and adjust acceleration accordingly */
static void gh_saturate_ref_speed(void)
{
  if (gh_ref.speed.x < -gh_ref.max_vel.x) {
    gh_ref.speed.x = -gh_ref.max_vel.x;
    if (gh_ref.accel.x < 0) {
      gh_ref.accel.x = 0;
    }
  } else if (gh_ref.speed.x > gh_ref.max_vel.x) {
    gh_ref.speed.x = gh_ref.max_vel.x;
    if (gh_ref.accel.x > 0) {
      gh_ref.accel.x = 0;
    }
  }
  if (gh_ref.speed.y < -gh_ref.max_vel.y) {
    gh_ref.speed.y = -gh_ref.max_vel.y;
    if (gh_ref.accel.y < 0) {
      gh_ref.accel.y = 0;
    }
  } else if (gh_ref.speed.y > gh_ref.max_vel.y) {
    gh_ref.speed.y = gh_ref.max_vel.y;
    if (gh_ref.accel.y > 0) {
      gh_ref.accel.y = 0;
    }
  }
}

