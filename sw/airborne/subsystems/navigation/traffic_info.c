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

#include "subsystems/navigation/traffic_info.h"

//#include "subsystems/navigation/common_nav.h"
#include "generated/airframe.h"     // AC_ID
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

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

struct ac_info_ *get_ac_info(uint8_t _id)
{
  return &the_acs[the_acs_id[_id]];
}

// 0 is reserved for ground station (_id=0)
// 1 is reserved for this AC (_id=AC_ID)
void SetAcInfo(uint8_t _id, float _utm_x /*m*/, float _utm_y /*m*/, float _course/*rad(CW)*/, float _alt/*m*/,
               float _gspeed/*m/s*/, float _climb, uint32_t _itow)
{
  if (acs_idx < NB_ACS) {
    if (_id > 0 && the_acs_id[_id] == 0) {
      the_acs_id[_id] = acs_idx++;
      the_acs[the_acs_id[_id]].ac_id = _id;
    }
    the_acs[the_acs_id[_id]].east = _utm_x;// -  nav_utm_east0;
    the_acs[the_acs_id[_id]].north = _utm_y;// - nav_utm_north0;
    the_acs[the_acs_id[_id]].course = _course;
    the_acs[the_acs_id[_id]].alt = _alt;// +- NAV_MSL0;
    the_acs[the_acs_id[_id]].gspeed = _gspeed;
    the_acs[the_acs_id[_id]].climb = _climb;
    the_acs[the_acs_id[_id]].itow = _itow;
  }
}

// 0 is reserved for ground station (_id=0)
// 1 is reserved for this AC (_id=AC_ID)
void SetAcInfoLLA(uint8_t _id, int32_t lat/*1e7deg*/, int32_t lon/*1e7deg*/, int32_t alt/*mm*/,
                  int16_t course/*decideg*/, uint16_t gspeed/*cm/s*/, int16_t climb/*cm/s*/,
                  uint32_t itow/*ms*/)
{
  if (acs_idx < NB_ACS) {
    if (_id > 0 && the_acs_id[_id] == 0) {
      the_acs_id[_id] = acs_idx++;
      the_acs[the_acs_id[_id]].ac_id = _id;
    }

    struct LlaCoor_i lla_i = {.lat = lat, .lon = lon, .alt = alt};
    struct LlaCoor_f lla_f;
    LLA_FLOAT_OF_BFP(lla_f, lla_i);

    struct UtmCoor_f utm_f;
    utm_of_lla_f(&utm_f, &lla_f);

    the_acs[the_acs_id[_id]].east = utm_f.east;
    the_acs[the_acs_id[_id]].north = utm_f.north;
    the_acs[the_acs_id[_id]].alt = utm_f.alt;
    the_acs[the_acs_id[_id]].course = RadOfDeg((float)course / 10.);
    the_acs[the_acs_id[_id]].gspeed = (float)gspeed * 100;
    the_acs[the_acs_id[_id]].climb = (float)climb * 100;
    the_acs[the_acs_id[_id]].itow = itow;
  }
}
