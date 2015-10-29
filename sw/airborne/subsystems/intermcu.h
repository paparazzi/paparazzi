/*
 * Copyright (C) 2015 The Paparazzi Team
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

#ifndef INTERMCU_ROTORCRAFT_H
#define INTERMCU_ROTORCRAFT_H


#include <stdint.h>

#include "subsystems/commands.h"


void intermcu_init(void);
void intermcu_on_rc_frame(void);
void intermcu_periodic(void);
void InterMcuEvent(void (*frame_handler)(void));

#include "subsystems/commands.h"
#include RADIO_CONTROL_TYPE_H


/** Data structure shared by fbw and ap processes */
struct fbw_state {
  pprz_t channels[RADIO_CONTROL_NB_CHANNEL];
  uint8_t ppm_cpt;
  uint8_t status;   ///<
  uint8_t nb_err;
  uint16_t vsupply; ///< 1e-1 V
  int32_t current;  ///< milliAmps
  float energy;     ///< mAh
};

struct ap_state {
  pprz_t commands[COMMANDS_NB];
  pprz_t command_roll_trim;
  pprz_t command_pitch_trim;
  pprz_t command_yaw_trim;
};

extern struct ap_state  from_ap;


///////////////////////////////////////////////////////////////////////////////
// InterMCU Watchdog

#define INTERMCU_LOST_CNT 25  /* 50ms with a 512Hz timer */

#define INTERMCU_OK          0
#define INTERMCU_LOST        1

struct InterMCU {
  uint8_t status;
  uint8_t time_since_last_frame;
};

extern struct InterMCU inter_mcu;


#endif
