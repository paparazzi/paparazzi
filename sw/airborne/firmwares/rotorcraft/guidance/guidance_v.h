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

extern uint8_t guidance_v_mode;

/** altitude setpoint in meters (input).
 *  fixed point representation: Q23.8
 *  accuracy 0.0039, range 8388km
 */
extern int32_t guidance_v_z_sp;

/** vertical speed setpoint in meter/s (input).
 *  fixed point representation: Q12.19
 *  accuracy 0.0000019, range +/-4096
 */
extern int32_t guidance_v_zd_sp;

/** altitude reference in meters.
 *  fixed point representation: Q23.8
 *  accuracy 0.0039, range 8388km
 */
extern int32_t guidance_v_z_ref;

/** vertical speed reference in meter/s.
 *  fixed point representation: Q12.19
 *  accuracy 0.0000038, range 4096
 */
extern int32_t guidance_v_zd_ref;

/** vertical acceleration reference in meter/s^2.
 *  fixed point representation: Q21.10
 *  accuracy 0.0009766, range 2097152
 */
extern int32_t guidance_v_zdd_ref;

extern int32_t guidance_v_z_sum_err; ///< accumulator for I-gain
extern int32_t guidance_v_ff_cmd;    ///< feed-forward command
extern int32_t guidance_v_fb_cmd;    ///< feed-back command

/** thrust command.
 *  summation of feed-forward and feed-back commands,
 *  valid range 0 : #MAX_PPRZ
 */
extern int32_t guidance_v_delta_t;

/** nominal throttle for hover.
 * This is only used if #GUIDANCE_V_NOMINAL_HOVER_THROTTLE is defined!
 * Unit: factor of #MAX_PPRZ with range 0.1 : 0.9
 */
extern float guidance_v_nominal_throttle;

/** Use adaptive throttle command estimation.
 */
extern bool_t guidance_v_adapt_throttle_enabled;


extern int32_t guidance_v_thrust_coeff;

extern int32_t guidance_v_kp; ///< vertical control P-gain
extern int32_t guidance_v_kd; ///< vertical control D-gain
extern int32_t guidance_v_ki; ///< vertical control I-gain

extern void guidance_v_init(void);
extern void guidance_v_read_rc(void);
extern void guidance_v_mode_changed(uint8_t new_mode);
extern void guidance_v_notify_in_flight(bool_t in_flight);
extern void guidance_v_run(bool_t in_flight);

#define guidance_v_SetKi(_val) {      \
    guidance_v_ki = _val;       \
    guidance_v_z_sum_err = 0;     \
  }

#endif /* GUIDANCE_V_H */
