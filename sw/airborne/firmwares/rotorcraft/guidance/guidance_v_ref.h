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

/** @file firmwares/rotorcraft/guidance/guidance_v_ref.h
 *  Reference generation for vertical guidance.
 *
 */

#ifndef GUIDANCE_V_REF_H
#define GUIDANCE_V_REF_H

#include "inttypes.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"
#include "generated/airframe.h"

#ifndef GUIDANCE_V_REF_MIN_ZD
#define GUIDANCE_V_REF_MIN_ZD (-3.)
#endif

#ifndef GUIDANCE_V_REF_MAX_ZD
#define GUIDANCE_V_REF_MAX_ZD ( 3.)
#endif


/** Update frequency
 */
#define GV_FREQ_FRAC 9
#define GV_FREQ (1<<GV_FREQ_FRAC)

/** reference model vertical accel in meters/s^2 (output)
 *  fixed point representation with #GV_ZDD_REF_FRAC
 *  Q23.8 : accuracy 0.0039 , range 8388km/s^2
 */
extern int32_t gv_zdd_ref;

/** number of bits for the fractional part of #gv_zdd_ref */
#define GV_ZDD_REF_FRAC 8

/** reference model vertical speed in meters/sec (output)
 *  fixed point representation with #GV_ZD_REF_FRAC
 *  Q14.17 : accuracy 0.0000076 , range 16384m/s2
 */
extern int32_t gv_zd_ref;

/** number of bits for the fractional part of #gv_zd_ref */
#define GV_ZD_REF_FRAC (GV_ZDD_REF_FRAC + GV_FREQ_FRAC)

/** reference model altitude in meters (output)
 *  fixed point representation with #GV_Z_REF_FRAC
 *  Q37.26 :
 */
extern int64_t gv_z_ref;

/** number of bits for the fractional part of #gv_z_ref */
#define GV_Z_REF_FRAC (GV_ZD_REF_FRAC + GV_FREQ_FRAC)

extern void gv_set_ref(int32_t alt, int32_t speed, int32_t accel);
extern void gv_update_ref_from_z_sp(int32_t z_sp);

/** update vertical reference from speed setpoint.
 * @param zd_sp  vertical speed setpoint with INT32_SPEED_FRAC
 * @param z_pos  current vertical position (z-down) with INT32_POS_FRAC
 */
extern void gv_update_ref_from_zd_sp(int32_t zd_sp, int32_t z_pos);

#endif /* GUIDANCE_V_REF_H */
