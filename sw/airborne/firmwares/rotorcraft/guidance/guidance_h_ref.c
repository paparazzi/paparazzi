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

static const float gh_max_accel = GUIDANCE_H_REF_MAX_ACCEL;

/** default second order model natural frequency */
#ifndef GUIDANCE_H_REF_OMEGA
#define GUIDANCE_H_REF_OMEGA RadOfDeg(67.)
#endif
/** default second order model damping */
#ifndef GUIDANCE_H_REF_ZETA
#define GUIDANCE_H_REF_ZETA  0.85f
#endif

/** first order time constant */
#ifndef GUIDANCE_H_REF_TAU
#define GUIDANCE_H_REF_TAU 0.5f
#endif

static void gh_saturate_speed(struct FloatVect2 *speed_sp);
static void gh_saturate_accel(struct FloatVect2 *accel_sp);

void gh_ref_init(void)
{
  gh_ref.omega = GUIDANCE_H_REF_OMEGA;
  gh_ref.zeta = GUIDANCE_H_REF_ZETA;
  gh_ref.zeta_omega = GUIDANCE_H_REF_ZETA * GUIDANCE_H_REF_OMEGA;
  gh_ref.omega_2 = GUIDANCE_H_REF_OMEGA * GUIDANCE_H_REF_OMEGA;
  gh_set_tau(GUIDANCE_H_REF_TAU);
  gh_set_max_speed(GUIDANCE_H_REF_MAX_SPEED);
  gh_ref.dt = (1.0f/PERIODIC_FREQUENCY);
}


float gh_set_max_speed(float max_speed)
{
  /* limit to 100m/s as int version would overflow at  2^14 = 128 m/s */
  gh_ref.max_speed = Min(fabsf(max_speed), 100.0f);
  return gh_ref.max_speed;
}

float gh_set_tau(float tau)
{
  gh_ref.tau = tau;
  Bound(gh_ref.tau, 0.01f, 2.0f);
  gh_ref.inv_tau = (1.f / gh_ref.tau);
  return gh_ref.tau;
}

float gh_set_omega(float omega)
{
  gh_ref.omega = omega;
  Bound(gh_ref.omega, 0.1f, 5.0f);
  gh_ref.omega_2 = gh_ref.omega * gh_ref.omega;
  gh_ref.zeta_omega = gh_ref.zeta * gh_ref.omega;
  return gh_ref.omega;
}

float gh_set_zeta(float zeta)
{
  gh_ref.zeta = zeta;
  Bound(gh_ref.zeta, 0.7f, 1.2f);
  gh_ref.zeta_omega = gh_ref.zeta * gh_ref.omega;
  return gh_ref.zeta;
}

void gh_set_ref(struct Int32Vect2 pos, struct FloatVect2 speed, struct FloatVect2 accel)
{
  struct Int64Vect2 new_pos;
  new_pos.x = ((int64_t)pos.x) << (GH_POS_REF_FRAC - INT32_POS_FRAC);
  new_pos.y = ((int64_t)pos.y) << (GH_POS_REF_FRAC - INT32_POS_FRAC);
  gh_ref.pos = new_pos;
  VECT2_COPY(gh_ref.speed, speed);
  VECT2_COPY(gh_ref.accel, accel);
}

void gh_update_ref_from_pos_sp(struct Int32Vect2 pos_sp)
{
  struct FloatVect2 pos_step, speed_step;

  VECT2_SMUL(pos_step, gh_ref.speed, gh_ref.dt);
  VECT2_SMUL(speed_step, gh_ref.accel, gh_ref.dt);

  struct Int64Vect2 pos_update;
  pos_update.x = LBFP_OF_REAL(pos_step.x, GH_POS_REF_FRAC);
  pos_update.y = LBFP_OF_REAL(pos_step.y, GH_POS_REF_FRAC);

  VECT2_ADD(gh_ref.pos, pos_update);
  VECT2_ADD(gh_ref.speed, speed_step);

  // compute pos error in pos_frac resolution
  struct FloatVect2 pos_err;
  pos_err.x = POS_FLOAT_OF_BFP(pos_sp.x - (gh_ref.pos.x >> (GH_POS_REF_FRAC - INT32_POS_FRAC)));
  pos_err.y = POS_FLOAT_OF_BFP(pos_sp.y - (gh_ref.pos.y >> (GH_POS_REF_FRAC - INT32_POS_FRAC)));

  // Calculate velocity error
  struct FloatVect2 vel_sp;
  VECT2_SMUL(vel_sp, pos_err, gh_ref.omega*0.5/gh_ref.zeta);

  // Saturate vel_sp
  gh_saturate_speed(&vel_sp);

  // compute the "speed part" of accel = -2*zeta*omega*speed -omega^2(pos - pos_sp)
  struct FloatVect2 accel_sp;
  struct FloatVect2 speed_err;
  VECT2_DIFF(speed_err, vel_sp, gh_ref.speed);
  VECT2_SMUL(accel_sp, speed_err, 2 * gh_ref.zeta_omega);

  gh_saturate_accel(&accel_sp);

  // copy accel
  VECT2_COPY(gh_ref.accel, accel_sp);
}

void gh_update_ref_from_speed_sp(struct FloatVect2 speed_sp)
{
  struct FloatVect2 pos_step, speed_step;

  VECT2_SMUL(pos_step, gh_ref.speed, gh_ref.dt);
  VECT2_SMUL(speed_step, gh_ref.accel, gh_ref.dt);

  struct Int64Vect2 pos_update;
  pos_update.x = LBFP_OF_REAL(pos_step.x, GH_POS_REF_FRAC);
  pos_update.y = LBFP_OF_REAL(pos_step.y, GH_POS_REF_FRAC);

  VECT2_ADD(gh_ref.pos, pos_update);
  VECT2_ADD(gh_ref.speed, speed_step);

  // compute speed error
  struct FloatVect2 speed_err;
  VECT2_DIFF(speed_err, gh_ref.speed, speed_sp);
  // compute accel from speed_sp
  struct FloatVect2 accel_sp;
  VECT2_SMUL(accel_sp, speed_err, -gh_ref.inv_tau);

  gh_saturate_accel(&accel_sp);

  // copy accel
  VECT2_COPY(gh_ref.accel, accel_sp);
}

static void gh_saturate_speed(struct FloatVect2 *speed_sp)
{
  // Speed squared
  float v_norm2 = VECT2_NORM2(*speed_sp);

  // Apply saturation if above max speed
  if (v_norm2 > (gh_ref.max_speed * gh_ref.max_speed)) {
    // speed_sp/sqrt(v_norm2)*vmax
    float factor = gh_ref.max_speed / sqrtf(v_norm2);
    VECT2_SMUL(*speed_sp, *speed_sp, factor);
  }
}

static void gh_saturate_accel(struct FloatVect2 *accel_sp)
{
  // Accel squared
  float a_norm2 = VECT2_NORM2(*accel_sp);

  // Apply saturation if above max speed
  if (a_norm2 > (gh_max_accel * gh_max_accel)) {
    // accel_sp/sqrt(a_norm2)*amax
    float factor = gh_max_accel / sqrtf(a_norm2);
    VECT2_SMUL(*accel_sp, *accel_sp, factor);
  }
}

