/*
 * Copyright (c) 2006 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/** \file modules/core/commands.c
 *  \brief Hardware independent data structures for commands handling
 *
 */

#include "modules/core/commands.h"

pprz_t commands[COMMANDS_NB];
const pprz_t commands_failsafe[COMMANDS_NB] = COMMANDS_FAILSAFE;

pprz_t command_roll_trim = 0;
pprz_t command_pitch_trim = 0;
pprz_t command_yaw_trim = 0;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_commands(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_COMMANDS(trans, dev, AC_ID , COMMANDS_NB, commands);
}
#endif

void commands_init(void)
{
  SetCommands(commands_failsafe);


#ifdef COMMAND_ROLL_TRIM
  command_roll_trim = COMMAND_ROLL_TRIM;
#endif
#ifdef COMMAND_PITCH_TRIM
  command_pitch_trim = COMMAND_PITCH_TRIM;
#endif
#ifdef COMMAND_YAW_TRIM
  command_yaw_trim = COMMAND_YAW_TRIM;
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_COMMANDS, send_commands);
#endif
}


