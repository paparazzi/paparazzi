/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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

/**
 * @file subsystems/navigation/traffic_info.c
 *
 * Information relative to the other aircrafts.
 *
 */

#include <inttypes.h>
#include "subsystems/navigation/traffic_info.h"
#include "generated/airframe.h"

uint8_t acs_idx;
uint8_t the_acs_id[NB_ACS_ID];
struct ac_info_ the_acs[NB_ACS];

void traffic_info_init(void)
{
  the_acs_id[0] = 0;  // ground station
  the_acs_id[AC_ID] = 1;
  the_acs[the_acs_id[AC_ID]].ac_id = AC_ID;
  acs_idx = 2;
}

struct ac_info_ *get_ac_info(uint8_t id)
{
  return &the_acs[the_acs_id[id]];
}
