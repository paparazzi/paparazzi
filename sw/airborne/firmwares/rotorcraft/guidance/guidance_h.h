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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file firmwares/rotorcraft/guidance/guidance_h.h
 *  Horizontal guidance for rotorcrafts.
 *
 */

#ifndef GUIDANCE_H_H
#define GUIDANCE_H_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "firmwares/rotorcraft/guidance/guidance_h_ref.h"
#include "firmwares/rotorcraft/stabilization.h"
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
#define GUIDANCE_H_MODE_FLIP        9
#define GUIDANCE_H_MODE_GUIDED      10


struct HorizontalGuidanceSetpoint {
  /** horizontal position setpoint in NED.
   *  fixed point representation: Q23.8
   *  accuracy 0.0039, range 8388km
   */
  struct Int32Vect2 pos;
  struct Int32Vect2 speed;  ///< only used in HOVER mode if GUIDANCE_H_USE_SPEED_REF or in GUIDED mode
  float heading;
  float heading_rate;
  uint8_t mask;             ///< bit 5: vx & vy, bit 6: vz, bit 7: vyaw
};

struct HorizontalGuidanceReference {
  struct Int32Vect2 pos;     ///< with #INT32_POS_FRAC
  struct Int32Vect2 speed;   ///< with #INT32_SPEED_FRAC
  struct Int32Vect2 accel;   ///< with #INT32_ACCEL_FRAC
};

struct HorizontalGuidance {
  uint8_t mode;
  /* configuration options */
  bool use_ref;

  struct HorizontalGuidanceSetpoint sp;   ///< setpoints
  struct HorizontalGuidanceReference ref; ///< reference calculated from setpoints

  struct FloatEulers rc_sp;               ///< remote control setpoint
};

extern struct HorizontalGuidance guidance_h;

extern int32_t transition_percentage;

extern void guidance_h_init(void);
extern void guidance_h_mode_changed(uint8_t new_mode);
extern void guidance_h_read_rc(bool in_flight);
extern void guidance_h_run(bool in_flight);
extern void guidance_h_run_enter(void);
extern struct StabilizationSetpoint guidance_h_run_pos(bool in_flight, struct HorizontalGuidance *gh);
extern struct StabilizationSetpoint guidance_h_run_speed(bool in_flight, struct HorizontalGuidance *gh);
extern struct StabilizationSetpoint guidance_h_run_accel(bool in_flight, struct HorizontalGuidance *gh);

extern void guidance_h_hover_enter(void);
extern void guidance_h_nav_enter(void);

/** Set horizontal guidance from NAV and run control loop
 */
extern void guidance_h_from_nav(bool in_flight);

/** Run GUIDED mode control
 */
extern void guidance_h_guided_run(bool in_flight);

/** Set horizontal position setpoint.
 * @param x North position (local NED frame) in meters.
 * @param y East position (local NED frame) in meters.
 */
extern void guidance_h_set_pos(float x, float y);

/** Set heading setpoint.
 * @param heading Setpoint in radians.
 */
extern void guidance_h_set_heading(float heading);

/** Set body relative horizontal velocity setpoint.
 * @param vx forward velocity (body frame) in meters/sec.
 * @param vy right velocity (body frame) in meters/sec.
 */
extern void guidance_h_set_body_vel(float vx, float vy);

/** Set horizontal velocity setpoint.
 * @param vx North velocity (local NED frame) in meters/sec.
 * @param vy East velocity (local NED frame) in meters/sec.
 */
extern void guidance_h_set_vel(float vx, float vy);

/** Set heading rate setpoint.
 * @param rate Heading rate in radians.
 */
extern void guidance_h_set_heading_rate(float rate);

/* Make sure that ref can only be temporarily disabled for testing,
 * but not enabled if GUIDANCE_H_USE_REF was defined to FALSE.
 */
#define guidance_h_SetUseRef(_val) {                    \
    guidance_h.use_ref = _val && GUIDANCE_H_USE_REF;    \
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

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_H_H */
