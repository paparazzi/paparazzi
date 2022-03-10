/*
 * Copyright (C) 2014 Christophe De Wagter
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

#include "gas_engine_idle_trim.h"

int gas_engine_idle_trim_left = 0;
int gas_engine_idle_trim_right = 0;

#include "modules/core/commands.h"


void periodic_gas_engine_idle_trim(void)
{
  command_set(COMMAND_IDLE1, radio_control_get(RADIO_GAIN1));
  command_set(COMMAND_IDLE2, radio_control_get(RADIO_GAIN2));
}

