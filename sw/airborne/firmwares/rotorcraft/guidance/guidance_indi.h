/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi.h
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 */

#ifndef GUIDANCE_INDI_H
#define GUIDANCE_INDI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"

extern void guidance_indi_init(void);
extern void guidance_indi_enter(void);

enum GuidanceIndi_HMode {
  GUIDANCE_INDI_H_POS,
  GUIDANCE_INDI_H_SPEED,
  GUIDANCE_INDI_H_ACCEL
};

enum GuidanceIndi_VMode {
  GUIDANCE_INDI_V_POS,
  GUIDANCE_INDI_V_SPEED,
  GUIDANCE_INDI_V_ACCEL
};

extern struct StabilizationSetpoint guidance_indi_run(struct FloatVect3 *accep_sp, float heading_sp);
extern struct StabilizationSetpoint guidance_indi_run_mode(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode);

extern struct FloatVect3 guidance_indi_controller(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode);

// Default number of virtual commands (e.g. [dax, day, daz])
#ifndef GUIDANCE_INDI_NV
#define GUIDANCE_INDI_NV 3
#endif

// Default number of outputs (e.g. [dtheta, dphi, dthrust])
#ifndef GUIDANCE_INDI_NU
#define GUIDANCE_INDI_NU 3
#endif

// Function to compute efficiency matrix G
extern void guidance_indi_calcG(float Gmat[GUIDANCE_INDI_NV][GUIDANCE_INDI_NU], struct FloatEulers att);
#if GUIDANCE_INDI_USE_WLS
#include "math/wls/wls_alloc.h"
extern void guidance_indi_set_wls_settings(struct WLS_t *wls, struct FloatEulers *euler_yxz, float heading_sp);
#endif

extern float guidance_indi_specific_force_gain;

// settings for guidance INDI
extern float guidance_indi_pos_gain;
extern float guidance_indi_speed_gain;
extern float guidance_indi_max_bank;

#endif /* GUIDANCE_INDI_H */
