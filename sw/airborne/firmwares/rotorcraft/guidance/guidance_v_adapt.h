/*
 * Copyright (C) 2009-2013 The Paparazzi Team
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

/** @file firmwares/rotorcraft/guidance/guidance_v_adapt.h
 *  Adaptation block of the vertical guidance.
 *
 *  This is a dimension one kalman filter estimating
 *  the ratio of vertical acceleration over thrust command ( ~ inverse of the mass )
 *  needed by the invert dynamic model to produce a nominal command.
 */

#ifndef GUIDANCE_V_ADAPT_H
#define GUIDANCE_V_ADAPT_H

#include "std.h"

/** State of the estimator.
 *  fixed point representation with #GV_ADAPT_X_FRAC
 *  Q13.18
 */
extern int32_t gv_adapt_X;
#define GV_ADAPT_X_FRAC 24

/** Covariance.
 *  fixed point representation with #GV_ADAPT_P_FRAC
 *  Q13.18
 */
extern int32_t gv_adapt_P;
#define GV_ADAPT_P_FRAC 18

/** Measurement */
extern int32_t gv_adapt_Xmeas;


extern void gv_adapt_init(void);
extern void gv_adapt_run(int32_t zdd_meas, int32_t thrust_applied, int32_t zd_ref);


#endif /* GUIDANCE_V_ADAPT_H */
