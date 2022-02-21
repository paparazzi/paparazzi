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
#include "autopilot.h"
#include "modules/core/commands.h"

void periodic_auto1_commands(void)
{
  // Copy Radio commands in AUTO1
  if (autopilot_get_mode() == AP_MODE_AUTO1) {
#ifdef COMMAND_GEAR
#ifndef RADIO_GEAR
#error auto1_commands COMMAND_GEAR needs RADIO_GEAR channel
#endif
    command_set(COMMAND_GEAR, radio_control_get(RADIO_GEAR));
#endif
#ifdef COMMAND_FLAP
#ifndef RADIO_FLAP
#error auto1_commands COMMAND_FLAP needs RADIO_FLAP channel
#endif
    command_set(COMMAND_FLAP, radio_control_get(RADIO_FLAP));
#endif
#ifdef COMMAND_AUX1
#ifndef RADIO_AUX1
#error auto1_commands COMMAND_AUX1 needs RADIO_AUX1 channel
#endif
    command_set(COMMAND_AUX1, radio_control_get(RADIO_AUX1));
#endif
#ifdef COMMAND_AUX2
#ifndef RADIO_AUX2
#error auto1_commands COMMAND_AUX2 needs RADIO_AUX2 channel
#endif
    command_set(COMMAND_AUX2, radio_control_get(RADIO_AUX2));
#endif
#ifdef COMMAND_AUX3
#ifndef RADIO_AUX3
#error auto1_commands COMMAND_AUX1 needs RADIO_AUX3 channel
#endif
    command_set(COMMAND_AUX3, radio_control_get(RADIO_AUX3));
#endif
#ifdef COMMAND_AUX4
#ifndef RADIO_AUX4
#error auto1_commands COMMAND_AUX4 needs RADIO_AUX4 channel
#endif
    command_set(COMMAND_AUX4, radio_control_get(RADIO_AUX4));
#endif
#ifdef COMMAND_AUX5
#ifndef RADIO_AUX5
#error auto1_commands COMMAND_AUX5 needs RADIO_AUX5 channel
#endif
    command_set(COMMAND_AUX5, radio_control_get(RADIO_AUX5));
#endif
#ifdef COMMAND_AUX6
#ifndef RADIO_AUX6
#error auto1_commands COMMAND_AUX6 needs RADIO_AUX6 channel
#endif
    command_set(COMMAND_AUX6, radio_control_get(RADIO_AUX6));
#endif
#ifdef COMMAND_AUX7
#ifndef RADIO_AUX7
#error auto1_commands COMMAND_AUX7 needs RADIO_AUX7 channel
#endif
    command_set(COMMAND_AUX7, radio_control_get(RADIO_AUX7));
#endif
#ifdef COMMAND_BRAKE
#ifndef RADIO_BRAKE
#error auto1_commands COMMAND_BRAKE needs RADIO_BRAKE channel
#endif
    command_set(COMMAND_BRAKE, radio_control_get(RADIO_BRAKE));
#endif
#ifdef COMMAND_HATCH
#ifndef RADIO_HATCH
#error auto1_commands COMMAND_HATCH needs RADIO_HATCH channel
#endif
    command_set(COMMAND_HATCH, radio_control_get(RADIO_HATCH));
#endif
  }
}
