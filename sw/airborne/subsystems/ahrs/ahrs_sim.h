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

#ifndef AHRS_SIM_H
#define AHRS_SIM_H

#include "subsystems/ahrs.h"
#include "std.h"

extern bool_t ahrs_sim_available;

#ifdef AHRS_UPDATE_FW_ESTIMATOR
#include "estimator.h"
// TODO copy ahrs to state instead of estimator
void ahrs_update_fw_estimator(void);
extern float ins_roll_neutral;
extern float ins_pitch_neutral;
#endif

extern void update_ahrs_from_sim(void);

#define AhrsEvent(_available_callback) {        \
    if (ahrs_sim_available) {                   \
      update_ahrs_from_sim();               \
      _available_callback();                    \
      ahrs_sim_available = FALSE;               \
    }                                           \
  }



#endif /* AHRS_SIM_H */
