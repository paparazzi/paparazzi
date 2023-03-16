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

/** @file firmwares/rotorcraft/guidance/guidance_v.h
 *  Vertical guidance for rotorcrafts.
 *
 */

#ifndef GUIDANCE_V_H
#define GUIDANCE_V_H

#include "std.h"

#include "firmwares/rotorcraft/guidance/guidance_v_ref.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"

#define GUIDANCE_V_MODE_KILL      0
#define GUIDANCE_V_MODE_RC_DIRECT 1
#define GUIDANCE_V_MODE_RC_CLIMB  2
#define GUIDANCE_V_MODE_CLIMB     3
#define GUIDANCE_V_MODE_HOVER     4
#define GUIDANCE_V_MODE_NAV       5
#define GUIDANCE_V_MODE_MODULE    6
#define GUIDANCE_V_MODE_FLIP      7
#define GUIDANCE_V_MODE_GUIDED    8

struct VerticalGuidance {
  uint8_t mode;

  /** altitude setpoint in meters (input).
   *  fixed point representation: Q23.8
   *  accuracy 0.0039, range 8388km
   */
  int32_t z_sp;

  /** vertical speed setpoint in meter/s (input).
   *  fixed point representation: Q12.19
   *  accuracy 0.0000019, range +/-4096
   */
  int32_t zd_sp;

  /** altitude reference in meters.
   *  fixed point representation: Q23.8
   *  accuracy 0.0039, range 8388km
   */
  int32_t z_ref;

  /** vertical speed reference in meter/s.
   *  fixed point representation: Q12.19
   *  accuracy 0.0000038, range 4096
   */
  int32_t zd_ref;

  /** vertical acceleration reference in meter/s^2.
   *  fixed point representation: Q21.10
   *  accuracy 0.0009766, range 2097152
   */
  int32_t zdd_ref;

  /** Direct throttle from radio control.
   *  range 0:#MAX_PPRZ
   */
  int32_t rc_delta_t;

  /** Vertical speed setpoint from radio control.
   *  fixed point representation: Q12.19
   *  accuracy 0.0000019, range +/-4096
   */
  int32_t rc_zd_sp;

  /** thrust setpoint.
   *  valid range 0 : #MAX_PPRZ
   */
  int32_t th_sp;

  /** thrust command.
   *  summation of feed-forward and feed-back commands,
   *  valid range 0 : #MAX_PPRZ
   */
  int32_t delta_t;

  /** nominal throttle for hover.
   * This is only used if #GUIDANCE_V_NOMINAL_HOVER_THROTTLE is defined!
   * Unit: factor of #MAX_PPRZ with range 0.1 : 0.9
   */
  float nominal_throttle;

  int32_t thrust_coeff;
};

extern struct VerticalGuidance guidance_v;

extern void guidance_v_init(void);
extern void guidance_v_read_rc(void);
extern void guidance_v_mode_changed(uint8_t new_mode);
extern void guidance_v_notify_in_flight(bool in_flight);
extern void guidance_v_thrust_adapt(bool in_flight);
extern void guidance_v_update_ref(void);
extern void guidance_v_run(bool in_flight);
extern void guidance_v_z_enter(void);

extern void guidance_v_run_enter(void);
extern int32_t guidance_v_run_pos(bool in_flight, struct VerticalGuidance *gv);
extern int32_t guidance_v_run_speed(bool in_flight, struct VerticalGuidance *gv);
extern int32_t guidance_v_run_accel(bool in_flight, struct VerticalGuidance *gv);

/** Set guidance ref parameters
*/
extern void guidance_v_set_ref(int32_t pos, int32_t speed, int32_t accel);
// macro for backward compatibility
#define GuidanceVSetRef guidance_v_set_ref

/** Set guidance setpoint from NAV and run hover loop
*/
extern void guidance_v_from_nav(bool in_flight);

/** Enter GUIDED mode control
*/
extern void guidance_v_guided_enter(void);

/** Run GUIDED mode control
*/
extern void guidance_v_guided_run(bool in_flight);

/** Set z position setpoint.
 * @param z Setpoint (down is positive) in meters.
 */
extern void guidance_v_set_z(float z);

/** Set z velocity setpoint.
 * @param vz Setpoint (down is positive) in meters/second.
 */
extern void guidance_v_set_vz(float vz);

/** Set throttle setpoint.
 * @param th Throttle setpoint between 0. and 1.
 */
extern void guidance_v_set_th(float th);

#endif /* GUIDANCE_V_H */
