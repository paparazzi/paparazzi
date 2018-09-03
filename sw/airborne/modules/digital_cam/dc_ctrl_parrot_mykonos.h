/*
 * Copyright (C) 2018 OpenUAS
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


/** @file modules/digital_cam/dc_ctrl_parrot_mykonos.h
 *  @brief Digital video/photo recorder control for Parrot Mykonos Platform,
 *  For others that is: control the camera of the Disco and if one manages to
 *  add pimpctl onto a Bebop or Bebop2, should work also on those
 *
 * Provides the control of the Camera start and stop of video recording,
 * Taking photos and switch video streaming on and off
 * This module starts the camera in standby mode and triggers starting
 * of recording or take a picture.
 * Minimum time between two pictures is 1 second.
 *
 */

#ifndef DC_CTRL_PARROT_MYKONOS_H
#define DC_CTRL_PARROT_MYKONOS_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

// Include Standard Camera Control Interface
// Note: Standard DC nneds to be unified and enhance for multicam support
#include "dc.h"

#include BOARD_CONFIG

enum dc_ctrl_parrot_mykonos_status {
  DC_CTRL_PARROT_MYKONOS_NONE,
  DC_CTRL_PARROT_MYKONOS_RECORD_START,
  DC_CTRL_PARROT_MYKONOS_RECORD_STOP,
  DC_CTRL_PARROT_MYKONOS_SHOOT,
  DC_CTRL_PARROT_MYKONOS_STREAM_START,
  DC_CTRL_PARROT_MYKONOS_STREAM_STOP,
  DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_START,
  DC_CTRL_PARROT_MYKONOS_AUTOSHOOT_STOP
};

struct Dc_Ctrl_Parrot_Mykonos {
  enum dc_ctrl_parrot_mykonos_status status;
  uint32_t timer;
  int16_t photo_nr;
  uint32_t autoshoot;
  struct EnuCoor_f last_shot_pos;
  uint32_t log_delay;
};

extern struct Dc_Ctrl_Parrot_Mykonos dc_ctrl_parrot_mykonos;

extern void dc_ctrl_parrot_mykonos_init(void);
extern void dc_ctrl_parrot_mykonos_periodic(void);
extern void dc_ctrl_parrot_mykonos_autoshoot(void);
extern void dc_ctrl_parrot_mykonos_autoshoot_start(void);
extern void dc_ctrl_parrot_mykonos_command(enum dc_ctrl_parrot_mykonos_status cmd);

// macro for setting handler
#define dc_ctrl_parrot_mykonos_SendCmd(cmd) dc_ctrl_parrot_mykonos_command(cmd)

#endif // DC_CTRL_PARROT_MYKONOS_H
