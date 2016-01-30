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
 * @file subsystems/navigation/traffic_info.h
 *
 * Information relative to the other aircrafts.
 *
 */

#ifndef TI_H
#define TI_H

#define NB_ACS_ID 256
#define NB_ACS 24

#include <inttypes.h>

struct ac_info_ {
  uint8_t ac_id;
  float east;   /* m relative to nav_utm_east0 */
  float north;  /* m relative to nav_utm_north0 */
  float course; /* rad (CW) */
  float alt;    /* m */
  float gspeed; /* m/s */
  float climb;  /* m/s */
  uint32_t itow;/* ms */
};

extern uint8_t acs_idx;
extern uint8_t the_acs_id[NB_ACS_ID];
extern struct ac_info_ the_acs[NB_ACS];

extern void traffic_info_init(void);
struct ac_info_ *get_ac_info(uint8_t id);

void SetAcInfo(uint8_t _id, float _utm_x /*m*/, float _utm_y /*m*/, float _course/*rad(CW)*/, float _alt/*m*/,
               float _gspeed/*m/s*/, float _climb, uint32_t _itow/*ms*/);
void SetAcInfoLLA(uint8_t _id, int32_t lat/*1e7deg*/, int32_t lon/*1e7deg*/, int32_t alt/*mm*/,
                  int16_t course/*decideg*/, uint16_t gspeed/*cm/s*/, int16_t climb/*cm/s*/,
                  uint32_t itow/*ms*/);

#endif
