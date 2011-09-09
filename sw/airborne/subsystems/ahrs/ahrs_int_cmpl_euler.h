/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

#ifndef AHRS_INT_CMPL_EULER_H
#define AHRS_INT_CMPL_EULER_H

#include "subsystems/ahrs.h"
#include "std.h"
#include "math/pprz_algebra_int.h"

struct AhrsIntCmplEuler {
  struct Int32Rates  gyro_bias;
  struct Int32Eulers hi_res_euler;
  struct Int32Eulers measure;
  struct Int32Eulers residual;
  struct Int32Eulers measurement;
  int32_t reinj_1;
};

extern struct AhrsIntCmplEuler ahrs_impl;


#ifdef AHRS_UPDATE_FW_ESTIMATOR
// TODO copy ahrs to state instead of estimator
void ahrs_update_fw_estimator(void);
extern float ins_roll_neutral;
extern float ins_pitch_neutral;
#endif

#endif /* AHRS_INT_CMPL_EULER_H */
