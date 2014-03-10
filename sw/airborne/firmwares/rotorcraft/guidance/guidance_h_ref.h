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

/** @file firmwares/rotorcraft/guidance/guidance_h_ref.h
 *  Reference generation for horizontal guidance.
 *
 */

#ifndef GUIDANCE_H_REF_H
#define GUIDANCE_H_REF_H

#include "inttypes.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"
#include "generated/airframe.h"

/** Default speed saturation */
#ifndef GUIDANCE_H_REF_MAX_SPEED
#define GUIDANCE_H_REF_MAX_SPEED 5.
#endif

/** Current maximum speed for waypoint navigation.
 * Defaults to #GUIDANCE_H_REF_MAX_SPEED
 */
extern float gh_max_speed;

/** Accel saturation.
 * tanf(RadOfDeg(30.))*9.81 = 5.66
 */
#ifndef GUIDANCE_H_REF_MAX_ACCEL
#define GUIDANCE_H_REF_MAX_ACCEL 5.66
#endif

/** Update frequency
 */
#define GH_FREQ_FRAC 9
#define GH_FREQ (1<<GH_FREQ_FRAC)

/** Reference model acceleration.
 * in meters/sec2 (output)
 * fixed point representation: Q23.8
 * accuracy 0.0039, range 8388km/s2
 */
extern struct Int32Vect2 gh_accel_ref;
#define GH_ACCEL_REF_FRAC 8

/** Reference model speed.
 * in meters/sec
 * with fixedpoint representation: Q14.17
 * accuracy 0.0000076 , range 16384m/s
 */
extern struct Int32Vect2 gh_speed_ref;
#define GH_SPEED_REF_FRAC (GH_ACCEL_REF_FRAC + GH_FREQ_FRAC)

/* Reference model position.
 * in meters
 * with fixedpoint representation: Q37.26
 */
extern struct Int64Vect2 gh_pos_ref;
#define GH_POS_REF_FRAC (GH_SPEED_REF_FRAC + GH_FREQ_FRAC)

extern void gh_set_ref(struct Int32Vect2 pos, struct Int32Vect2 speed, struct Int32Vect2 accel);
extern void gh_update_ref_from_pos_sp(struct Int32Vect2 pos_sp);
extern void gh_update_ref_from_speed_sp(struct Int32Vect2 speed_sp);

/**
 * Set a new maximum speed for waypoint navigation.
 * @param max_speed speed saturation in m/s
 * @return new maximum speed
 */
extern float gh_set_max_speed(float max_speed);

#endif /* GUIDANCE_H_REF_H */
