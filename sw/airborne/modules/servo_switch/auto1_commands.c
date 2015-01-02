/*
 * Copyright (C) 2014 OpenUAS
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

#include "auto1_commands.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/autopilot.h"
#include "inter_mcu.h"

void periodic_auto1_commands(void)
{
  // Copy Radio commands in AUTO1
  if (pprz_mode == PPRZ_MODE_AUTO1) {
#ifdef COMMAND_HATCH
#ifndef RADIO_HATCH
#error auto1_commands COMMAND_HATCH needs RADIO_HATCH channel
#endif
    ap_state->commands[COMMAND_HATCH] = fbw_state->channels[RADIO_HATCH];
#endif
#ifdef COMMAND_BRAKE
#ifndef RADIO_BRAKE
#error auto1_commands COMMAND_BRAKE needs RADIO_BRAKE channel
#endif
    ap_state->commands[COMMAND_BRAKE] = fbw_state->channels[RADIO_BRAKE];
#endif
#ifdef COMMAND_FLAPS
#ifndef RADIO_FLAPS
#error auto1_commands COMMAND_FLAPS needs RADIO_FLAPS channel
#endif
    ap_state->commands[COMMAND_FLAPS] = fbw_state->channels[RADIO_FLAPS];
#endif
  }
}
