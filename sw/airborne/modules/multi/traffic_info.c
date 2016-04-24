/*
 * Copyright (C) Pascal Brisset, Antoine Drouin (2008), Kirk Scheper (2016)
 *
 * This file is part of paparazzi
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
 */
/**
 * @file "modules/traffic_info/traffic_info.c"
 * @author Kirk Scheper
 * Information relative to the other aircrafts.
 * Keeps track of other aircraft in airspace
 */

#include "modules/multi/traffic_info.h"

#include "generated/airframe.h"     // AC_ID
#include "generated/flight_plan.h"  // NAV_MSL0

#include "subsystems/datalink/datalink.h"
#include "pprzlink/dl_protocol.h"
#include "pprzlink/messages.h"

#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_utm.h"

#include "subsystems/gps.h"

uint8_t ti_acs_idx;
uint8_t ti_acs_id[NB_ACS_ID];
struct ac_info_ ti_acs[NB_ACS];

void traffic_info_init(void)
{
  memset(ti_acs_id, 0, NB_ACS_ID);

  ti_acs_id[0] = 0;  // ground station
  ti_acs_id[AC_ID] = 1;
  ti_acs[ti_acs_id[AC_ID]].ac_id = AC_ID;
  ti_acs_idx = 2;
}

int parse_acinfo_dl()
{
  uint8_t sender_id = SenderIdOfPprzMsg(dl_buffer);
  uint8_t msg_id = IdOfPprzMsg(dl_buffer);

  if (sender_id > 0) {
    switch (msg_id) {
      case DL_GPS_SMALL: {
        uint32_t multiplex_speed = DL_GPS_SMALL_multiplex_speed(dl_buffer);

        // decode compressed values
        int16_t course = (int16_t)((multiplex_speed >> 21) & 0x7FF); // bits 31-21 course in decideg
        if (course & 0x400) {
          course |= 0xF800;  // fix for twos complements
        }
        course *= 2; // scale course by resolution
        int16_t gspeed = (int16_t)((multiplex_speed >> 10) & 0x7FF); // bits 20-10 ground speed cm/s
        if (gspeed & 0x400) {
          gspeed |= 0xF800;  // fix for twos complements
        }
        int16_t climb = (int16_t)(multiplex_speed & 0x3FF); // bits 9-0 z climb speed in cm/s
        if (climb & 0x200) {
          climb |= 0xFC00;  // fix for twos complements
        }

        set_ac_info_lla(sender_id,
                        DL_GPS_SMALL_lla_lat(dl_buffer),
                        DL_GPS_SMALL_lla_lon(dl_buffer),
                        (int32_t)DL_GPS_SMALL_alt(dl_buffer) * 10,
                        course,
                        gspeed,
                        climb,
                        gps_tow_from_sys_ticks(sys_time.nb_tick));
      }
      break;

      case DL_GPS: {
        set_ac_info(sender_id,
                    DL_GPS_utm_east(dl_buffer),
                    DL_GPS_utm_north(dl_buffer),
                    DL_GPS_alt(dl_buffer),
                    DL_GPS_utm_zone(dl_buffer),
                    DL_GPS_course(dl_buffer),
                    DL_GPS_speed(dl_buffer),
                    DL_GPS_climb(dl_buffer),
                    DL_GPS_itow(dl_buffer));
      }
      break;
      case DL_GPS_LLA: {
        set_ac_info_lla(sender_id,
                        DL_GPS_LLA_lat(dl_buffer),
                        DL_GPS_LLA_lon(dl_buffer),
                        DL_GPS_LLA_alt(dl_buffer),
                        DL_GPS_LLA_course(dl_buffer),
                        DL_GPS_LLA_speed(dl_buffer),
                        DL_GPS_LLA_climb(dl_buffer),
                        DL_GPS_LLA_itow(dl_buffer));
      }
      break;
      default:
        return 0;
    }

  } else {
    switch (msg_id) {
      case DL_ACINFO: {
        set_ac_info(DL_ACINFO_ac_id(dl_buffer),
                    DL_ACINFO_utm_east(dl_buffer),
                    DL_ACINFO_utm_north(dl_buffer),
                    DL_ACINFO_alt(dl_buffer) * 10,
                    DL_ACINFO_utm_zone(dl_buffer),
                    DL_ACINFO_course(dl_buffer),
                    DL_ACINFO_speed(dl_buffer),
                    DL_ACINFO_climb(dl_buffer),
                    DL_ACINFO_itow(dl_buffer));
      }
      break;
      default:
        return 0;
    }
  }

  return 1;
}

struct ac_info_ *get_ac_info(uint8_t _id)
{
  return &ti_acs[ti_acs_id[_id]];
}

void set_ac_info(uint8_t id, uint32_t utm_east, uint32_t utm_north, uint32_t alt, uint8_t utm_zone, uint16_t course,
                 uint16_t gspeed, uint16_t climb, uint32_t itow)
{
  if (ti_acs_idx < NB_ACS) {
    if (id > 0 && ti_acs_id[id] == 0) {    // new aircraft id
      ti_acs_id[id] = ti_acs_idx++;
      ti_acs[ti_acs_id[id]].ac_id = id;
    }

    uint16_t my_zone = UtmZoneOfLlaLonDeg(gps.lla_pos.lon);
    if (utm_zone == my_zone) {
      ti_acs[ti_acs_id[id]].utm.east = utm_east;
      ti_acs[ti_acs_id[id]].utm.north = utm_north;
    } else { // store other uav in utm extended zone
      struct UtmCoor_i utm = {.east = utm_east, .north = utm_north, .alt = alt, .zone = utm_zone};
      struct LlaCoor_i lla;
      lla_of_utm_i(&lla, &utm);
      utm.zone = my_zone;
      utm_of_lla_i(&utm, &lla);

      ti_acs[ti_acs_id[id]].utm.east = utm.east;
      ti_acs[ti_acs_id[id]].utm.north = utm.north;
    }

    ti_acs[ti_acs_id[id]].utm.alt = alt;
    ti_acs[ti_acs_id[id]].utm.zone = utm_zone;
    ti_acs[ti_acs_id[id]].course = course;
    ti_acs[ti_acs_id[id]].gspeed = gspeed;
    ti_acs[ti_acs_id[id]].climb = climb;
    ti_acs[ti_acs_id[id]].itow = itow;
  }
}

void set_ac_info_lla(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                     int16_t course, uint16_t gspeed, int16_t climb, uint32_t itow)
{

  if (ti_acs_idx < NB_ACS) {
    if (id > 0 && ti_acs_id[id] == 0) {
      ti_acs_id[id] = ti_acs_idx++;
      ti_acs[ti_acs_id[id]].ac_id = id;
    }

    struct LlaCoor_i lla = {.lat = lat, .lon = lon, .alt = alt};
    struct UtmCoor_i utm;
    utm.zone = UtmZoneOfLlaLonDeg(gps.lla_pos.lon);   // use my zone as reference, i.e zone extend

    utm_of_lla_i(&utm, &lla);

    UTM_COPY(ti_acs[ti_acs_id[id]].utm, utm);
    ti_acs[ti_acs_id[id]].course = course;
    ti_acs[ti_acs_id[id]].gspeed = gspeed;
    ti_acs[ti_acs_id[id]].climb = climb;
    ti_acs[ti_acs_id[id]].itow = itow;
  }
}
