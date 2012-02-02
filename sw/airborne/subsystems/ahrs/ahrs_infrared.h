/*
 * Copyright (C) 2011 The Paparazzi Team
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

#ifndef AHRS_INFRARED_H
#define AHRS_INFRARED_H

#include "subsystems/ahrs.h"
#include "std.h"

// TODO: harmonize infrared neutrals with ins_neutrals
// or get rid of ins neutrals
// this ins only needed for sim right now
extern float ins_roll_neutral;
extern float ins_pitch_neutral;

extern void ahrs_update_infrared(void);

// TODO copy ahrs to state instead of estimator
extern void ahrs_update_fw_estimator(void);

#define AhrsEvent(_available_callback) {        \
  }

#endif /* AHRS_INFRARED_H */
