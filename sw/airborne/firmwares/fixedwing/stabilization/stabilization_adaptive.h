/*
 * Paparazzi $Id: fw_h_ctl.h 3784 2009-07-24 14:55:54Z poine $
 *
 * Copyright (C) 2009  ENAC
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
 *
 */

/**
 *
 * fixed wing horizontal adaptive control
 *
 */

#ifndef FW_H_CTL_A_H
#define FW_H_CTL_A_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

extern float h_ctl_roll_sum_err;
extern float h_ctl_pitch_sum_err;
extern float h_ctl_roll_igain;
extern float h_ctl_pitch_igain;
extern float h_ctl_roll_Kffa;
extern float h_ctl_roll_Kffd;
extern float h_ctl_pitch_Kffa;
extern float h_ctl_pitch_Kffd;
extern float h_ctl_pitch_of_roll;

#define H_CTL_ROLL_SUM_ERR_MAX 100.
#define H_CTL_PITCH_SUM_ERR_MAX 100.

#define fw_h_ctl_a_SetRollIGain(_gain) { \
  h_ctl_roll_sum_err = 0; \
  h_ctl_roll_igain = _gain; \
  }

#define fw_h_ctl_a_SetPitchIGain(_gain) { \
  h_ctl_pitch_sum_err = 0; \
  h_ctl_pitch_igain = _gain; \
  }

/* inner roll loop parameters */
extern float h_ctl_ref_roll_angle;
extern float h_ctl_ref_roll_rate;
extern float h_ctl_ref_roll_accel;

/* inner pitch loop parameters */
extern float h_ctl_ref_pitch_angle;
extern float h_ctl_ref_pitch_rate;
extern float h_ctl_ref_pitch_accel;

extern bool_t use_airspeed_ratio;

#endif /* FW_H_CTL_A_H */
