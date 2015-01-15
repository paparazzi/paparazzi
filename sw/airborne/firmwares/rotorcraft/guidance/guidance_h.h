/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file firmwares/rotorcraft/guidance/guidance_h.h
 *  Horizontal guidance for rotorcrafts.
 *
 */

#ifndef GUIDANCE_H_H
#define GUIDANCE_H_H


#include "math/pprz_algebra_int.h"

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
#include "generated/airframe.h"
#include "std.h"

/** Use horizontal guidance reference trajectory.
 * Default is TRUE, define to FALSE to always disable it.
 */
#ifndef GUIDANCE_H_USE_REF
#define GUIDANCE_H_USE_REF TRUE
#endif

/** Use horizontal guidance speed reference.
 * This also allows to give velocity commands via RC in GUIDANCE_H_MODE_HOVER.
 * Default is TRUE, define to FALSE to always disable it.
 */
#ifndef GUIDANCE_H_USE_SPEED_REF
#define GUIDANCE_H_USE_SPEED_REF TRUE
#endif

#define GUIDANCE_H_MODE_KILL        0
#define GUIDANCE_H_MODE_RATE        1
#define GUIDANCE_H_MODE_ATTITUDE    2
#define GUIDANCE_H_MODE_HOVER       3
#define GUIDANCE_H_MODE_NAV         4
#define GUIDANCE_H_MODE_RC_DIRECT   5
#define GUIDANCE_H_MODE_CARE_FREE   6
#define GUIDANCE_H_MODE_FORWARD     7
#define GUIDANCE_H_MODE_MODULE      8


extern uint8_t guidance_h_mode;
extern bool_t guidance_h_use_ref;
extern bool_t guidance_h_approx_force_by_thrust;

/** horizontal position setpoint in NED.
 *  fixed point representation: Q23.8
 *  accuracy 0.0039, range 8388km
 */
extern struct Int32Vect2 guidance_h_pos_sp;

extern struct Int32Vect2 guidance_h_pos_ref;        ///< with #INT32_POS_FRAC
extern struct Int32Vect2 guidance_h_speed_ref;      ///< with #INT32_SPEED_FRAC
extern struct Int32Vect2 guidance_h_accel_ref;      ///< with #INT32_ACCEL_FRAC

extern struct Int32Vect2 guidance_h_pos_err;
extern struct Int32Vect2 guidance_h_speed_err;
extern struct Int32Vect2 guidance_h_trim_att_integrator;


/** horizontal guidance command.
 * In north/east with #INT32_ANGLE_FRAC
 * @todo convert to real force command
 */
extern struct Int32Vect2  guidance_h_cmd_earth;
extern struct Int32Eulers guidance_h_rc_sp;         ///< with #INT32_ANGLE_FRAC
extern int32_t guidance_h_heading_sp;               ///< with #INT32_ANGLE_FRAC

extern int32_t guidance_h_pgain;
extern int32_t guidance_h_dgain;
extern int32_t guidance_h_igain;
extern int32_t guidance_h_vgain;
extern int32_t guidance_h_again;

extern int32_t transition_percentage;
extern int32_t transition_theta_offset;

extern void guidance_h_init(void);
extern void guidance_h_mode_changed(uint8_t new_mode);
extern void guidance_h_read_rc(bool_t in_flight);
extern void guidance_h_run(bool_t in_flight);


#define guidance_h_SetKi(_val) {            \
    guidance_h_igain = _val;                \
    INT_VECT2_ZERO(guidance_h_trim_att_integrator); \
  }

/* Make sure that ref can only be temporarily disabled for testing,
 * but not enabled if GUIDANCE_H_USE_REF was defined to FALSE.
 */
#define guidance_h_SetUseRef(_val) {                    \
    guidance_h_use_ref = _val && GUIDANCE_H_USE_REF;    \
  }

static inline void guidance_h_SetMaxSpeed(float speed)
{
  gh_set_max_speed(speed);
}

static inline void guidance_h_SetOmega(float omega)
{
  gh_set_omega(omega);
}

static inline void guidance_h_SetZeta(float zeta)
{
  gh_set_zeta(zeta);
}

static inline void guidance_h_SetTau(float tau)
{
  gh_set_tau(tau);
}

#endif /* GUIDANCE_H_H */
