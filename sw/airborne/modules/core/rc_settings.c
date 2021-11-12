/*
 *
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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


#include <inttypes.h>
#include "generated/radio.h"
#include "modules/core/rc_settings.h"
#include "autopilot.h"
#include "firmwares/fixedwing/nav.h"
#include "modules/intermcu/inter_mcu.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

uint8_t rc_settings_mode = 0;
float slider_1_val, slider_2_val;

#define ParamValInt16(param_init_val, param_travel, cur_pulse, init_pulse) \
  (param_init_val + (int16_t)(((float)(cur_pulse - init_pulse)) * param_travel / (float)MAX_PPRZ))

#define ParamValFloat(param_init_val, param_travel, cur_pulse, init_pulse) \
  (param_init_val + ((float)(cur_pulse - init_pulse)) * param_travel / (float)MAX_PPRZ)

#define RcChannel(x) (imcu_get_radio(x))

/** Includes generated code from tuning_rc.xml */
#include "generated/settings.h"


void rc_settings(bool mode_changed __attribute__((unused)))
{
  RCSettings(mode_changed);
}
