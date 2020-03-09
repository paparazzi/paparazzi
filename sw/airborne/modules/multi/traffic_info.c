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
 * @file "modules/multi/traffic_info.c"
 * @author Kirk Scheper
 * Information relative to the other aircrafts.
 * Keeps track of other aircraft in airspace
 */

#include "modules/multi/traffic_info.h"

#include "generated/airframe.h"     // AC_ID
#include "generated/flight_plan.h"  // NAV_MSL0

#include "subsystems/datalink/datalink.h"
#include "pprzlink/dl_protocol.h"   // datalink messages
#include "pprzlink/messages.h"	    // telemetry messages

#include "state.h"
#include "math/pprz_geodetic_utm.h"
#include "math/pprz_geodetic_wgs84.h"

/* number of ac being tracked */
uint8_t ti_acs_idx;
/* index of ac in the list of traffic info aircraft (ti_acs) */
uint8_t ti_acs_id[NB_ACS_ID];
/* container for available traffic info */
struct acInfo ti_acs[NB_ACS];

/* Geoid height (msl) over ellipsoid [mm] */
int32_t geoid_height;

void traffic_info_init(void)
{
  memset(ti_acs_id, 0, NB_ACS_ID);

  ti_acs_id[0] = 0;  // ground station
  ti_acs_id[AC_ID] = 1;
  ti_acs[ti_acs_id[AC_ID]].ac_id = AC_ID;
  ti_acs_idx = 2;

  geoid_height = NAV_MSL0;
}

/**
 * Update estimate of the geoid height
 * Requires an available hsml and/or lla measurement, if not available value isn't updated
 */
static void update_geoid_height(void) {
  if(bit_is_set(gps.valid_fields, GPS_VALID_HMSL_BIT) && bit_is_set(gps.valid_fields, GPS_VALID_POS_LLA_BIT))
  {
    geoid_height = gps.lla_pos.alt - gps.hmsl;
  } else if(bit_is_set(gps.valid_fields, GPS_VALID_POS_LLA_BIT))
  {
    geoid_height = wgs84_ellipsoid_to_geoid_i(gps.lla_pos.lat, gps.lla_pos.lon);
  } /* default is just keep last available height */
}

bool parse_acinfo_dl(uint8_t *buf)
{
  uint8_t sender_id = SenderIdOfPprzMsg(buf);
  uint8_t msg_id = IdOfPprzMsg(buf);

  /* handle telemetry message */
#if PPRZLINK_DEFAULT_VER == 2
  if (pprzlink_get_msg_class_id(buf) == DL_telemetry_CLASS_ID) {
#else
  if (sender_id > 0) {
#endif
    switch (msg_id) {
      case DL_GPS_SMALL: {
        uint32_t multiplex_speed = DL_GPS_SMALL_multiplex_speed(buf);

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
                        DL_GPS_SMALL_lat(buf),
                        DL_GPS_SMALL_lon(buf),
                        (int32_t)DL_GPS_SMALL_alt(buf) * 10,
                        course,
                        gspeed,
                        climb,
                        gps_tow_from_sys_ticks(sys_time.nb_tick));
      }
      break;
      case DL_GPS: {
        set_ac_info_utm(sender_id,
                    DL_GPS_utm_east(buf),
                    DL_GPS_utm_north(buf),
                    DL_GPS_alt(buf),
                    DL_GPS_utm_zone(buf),
                    DL_GPS_course(buf),
                    DL_GPS_speed(buf),
                    DL_GPS_climb(buf),
                    DL_GPS_itow(buf));
      }
      break;
      case DL_GPS_LLA: {
        set_ac_info_lla(sender_id,
                        DL_GPS_LLA_lat(buf),
                        DL_GPS_LLA_lon(buf),
                        DL_GPS_LLA_alt(buf),
                        DL_GPS_LLA_course(buf),
                        DL_GPS_LLA_speed(buf),
                        DL_GPS_LLA_climb(buf),
                        DL_GPS_LLA_itow(buf));
      }
      break;
      default:
        return FALSE;
    }
  /* handle datalink message */
  } else {
    switch (msg_id) {
      case DL_ACINFO: {
        set_ac_info_utm(DL_ACINFO_ac_id(buf),
                        DL_ACINFO_utm_east(buf),
                        DL_ACINFO_utm_north(buf),
                        DL_ACINFO_alt(buf) * 10,
                        DL_ACINFO_utm_zone(buf),
                        DL_ACINFO_course(buf),
                        DL_ACINFO_speed(buf),
                        DL_ACINFO_climb(buf),
                        DL_ACINFO_itow(buf));
      }
      break;
      case DL_ACINFO_LLA: {
        set_ac_info_lla(DL_ACINFO_LLA_ac_id(buf),
                  DL_ACINFO_LLA_lat(buf),
                  DL_ACINFO_LLA_lon(buf),
                  DL_ACINFO_LLA_alt(buf) * 10,
                  DL_ACINFO_LLA_course(buf),
                  DL_ACINFO_LLA_speed(buf),
                  DL_ACINFO_LLA_climb(buf),
                  DL_ACINFO_LLA_itow(buf));
      }
      break;
      default:
        return FALSE;
    }
  }
  return TRUE;
}


void set_ac_info_utm(uint8_t id, uint32_t utm_east, uint32_t utm_north, uint32_t alt, uint8_t utm_zone, uint16_t course,
                 uint16_t gspeed, uint16_t climb, uint32_t itow)
{
  if (ti_acs_idx < NB_ACS) {
    if (id > 0 && ti_acs_id[id] == 0) {    // new aircraft id
      ti_acs_id[id] = ti_acs_idx++;
      ti_acs[ti_acs_id[id]].ac_id = id;
    }

    ti_acs[ti_acs_id[id]].status = 0;

    uint16_t my_zone = state.utm_origin_f.zone;
    if (utm_zone == my_zone) {
      ti_acs[ti_acs_id[id]].utm_pos_i.east = utm_east;
      ti_acs[ti_acs_id[id]].utm_pos_i.north = utm_north;
      ti_acs[ti_acs_id[id]].utm_pos_i.alt = alt;
      ti_acs[ti_acs_id[id]].utm_pos_i.zone = utm_zone;
      SetBit(ti_acs[ti_acs_id[id]].status, AC_INFO_POS_UTM_I);

    } else { // store other uav in utm extended zone
      struct UtmCoor_i utm = {.east = utm_east, .north = utm_north, .alt = alt, .zone = utm_zone};
      struct LlaCoor_i lla;
      lla_of_utm_i(&lla, &utm);
      LLA_COPY(ti_acs[ti_acs_id[id]].lla_pos_i, lla);
      SetBit(ti_acs[ti_acs_id[id]].status, AC_INFO_POS_LLA_I);

      utm.zone = my_zone;
      utm_of_lla_i(&utm, &lla);

      UTM_COPY(ti_acs[ti_acs_id[id]].utm_pos_i, utm);
      SetBit(ti_acs[ti_acs_id[id]].status, AC_INFO_POS_UTM_I);
    }

    ti_acs[ti_acs_id[id]].course = RadOfDeciDeg(course);
    ti_acs[ti_acs_id[id]].gspeed = MOfCm(gspeed);
    ti_acs[ti_acs_id[id]].climb = MOfCm(climb);
    SetBit(ti_acs[ti_acs_id[id]].status, AC_INFO_VEL_LOCAL_F);

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

    ti_acs[ti_acs_id[id]].status = 0;

    struct LlaCoor_i lla = {.lat = lat, .lon = lon, .alt = alt};
    LLA_COPY(ti_acs[ti_acs_id[id]].lla_pos_i, lla);
    SetBit(ti_acs[ti_acs_id[id]].status, AC_INFO_POS_LLA_I);

    ti_acs[ti_acs_id[id]].course = RadOfDeciDeg(course);
    ti_acs[ti_acs_id[id]].gspeed = MOfCm(gspeed);
    ti_acs[ti_acs_id[id]].climb = MOfCm(climb);
    SetBit(ti_acs[ti_acs_id[id]].status, AC_INFO_VEL_LOCAL_F);

    ti_acs[ti_acs_id[id]].itow = itow;
  }
}

/* Conversion funcitons for computing ac position in requested reference frame from
 * any other available reference frame
 */

/* compute UTM position of aircraft with ac_id (int) */
void acInfoCalcPositionUtm_i(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    return;
  }

  /* LLA_i -> UTM_i is more accurate than from UTM_f */
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_i.zone = state.utm_origin_f.zone;
    utm_of_lla_i(&ti_acs[ac_nr].utm_pos_i, &ti_acs[ac_nr].lla_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_i.alt -= geoid_height;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    UTM_BFP_OF_REAL(ti_acs[ac_nr].utm_pos_i, ti_acs[ac_nr].utm_pos_f);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_i.zone = state.utm_origin_f.zone;
    utm_of_lla_f(&ti_acs[ac_nr].utm_pos_f, &ti_acs[ac_nr].lla_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_f.alt -= geoid_height/1000.;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F);
    UTM_BFP_OF_REAL(ti_acs[ac_nr].utm_pos_i, ti_acs[ac_nr].utm_pos_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I);
}

/* compute LLA position of aircraft with ac_id (int) */
void acInfoCalcPositionLla_i(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    LLA_BFP_OF_REAL(ti_acs[ac_nr].lla_pos_i, ti_acs[ac_nr].lla_pos_f);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    lla_of_utm_i(&ti_acs[ac_nr].lla_pos_i, &ti_acs[ac_nr].utm_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_i.alt += geoid_height;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    lla_of_utm_f(&ti_acs[ac_nr].lla_pos_f, &ti_acs[ac_nr].utm_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_f.alt += geoid_height/1000.;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F);
    LLA_BFP_OF_REAL(ti_acs[ac_nr].lla_pos_i, ti_acs[ac_nr].lla_pos_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I);
}

/* compute ENU position of aircraft with ac_id (int) */
void acInfoCalcPositionEnu_i(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F))
  {
    ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f);
  }
  else if (state.ned_initialized_i)
  {
    if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
    {
      enu_of_lla_point_i(&ti_acs[ac_nr].enu_pos_i, &state.ned_origin_i, acInfoGetPositionLla_i(ac_id));
    } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
    {
      enu_of_lla_point_f(&ti_acs[ac_nr].enu_pos_f, &state.ned_origin_f, acInfoGetPositionLla_f(ac_id));
      SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
      ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f)
    }
  } else if (state.utm_initialized_f)
  {
    /* if utm origin is initialized we use the ENU = UTM - UTM_ORIGIN as in state to facilitate comparison */
    ENU_OF_UTM_DIFF(ti_acs[ac_nr].enu_pos_f, *acInfoGetPositionUtm_f(ac_id), state.utm_origin_f);
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
    ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I);
}

/* compute UTM position of aircraft with ac_id (float) */
void acInfoCalcPositionUtm_f(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    UTM_FLOAT_OF_BFP(ti_acs[ac_nr].utm_pos_f, ti_acs[ac_nr].utm_pos_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_i.zone = state.utm_origin_f.zone;
    utm_of_lla_i(&ti_acs[ac_nr].utm_pos_i, &ti_acs[ac_nr].lla_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_i.alt -= geoid_height;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I);
    UTM_FLOAT_OF_BFP(ti_acs[ac_nr].utm_pos_f, ti_acs[ac_nr].utm_pos_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    /* not very accurate with float ~5cm */
    ti_acs[ac_nr].utm_pos_f.zone = state.utm_origin_f.zone;
    utm_of_lla_f(&ti_acs[ac_nr].utm_pos_f, &ti_acs[ac_nr].lla_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_f.alt -= geoid_height/1000.;
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F);
}

/* compute LLA position of aircraft with ac_id (float) */
void acInfoCalcPositionLla_f(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    LLA_FLOAT_OF_BFP(ti_acs[ac_nr].lla_pos_f, ti_acs[ac_nr].lla_pos_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    lla_of_utm_i(&ti_acs[ac_nr].lla_pos_i, &ti_acs[ac_nr].utm_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_i.alt += geoid_height;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I);
    LLA_FLOAT_OF_BFP(ti_acs[ac_nr].lla_pos_f, ti_acs[ac_nr].lla_pos_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    lla_of_utm_f(&ti_acs[ac_nr].lla_pos_f, &ti_acs[ac_nr].utm_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_f.alt += geoid_height/1000.;
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F);
}

/* compute ENU position of aircraft with ac_id (float) */
void acInfoCalcPositionEnu_f(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I))
  {
    ENU_FLOAT_OF_BFP(ti_acs[ac_nr].enu_pos_f, ti_acs[ac_nr].enu_pos_i);
  }
  else if (state.ned_initialized_i)
  {
    if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
    {
      enu_of_lla_point_f(&ti_acs[ac_nr].enu_pos_f, &state.ned_origin_f, acInfoGetPositionLla_f(ac_id));
    } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
    {
      enu_of_lla_point_i(&ti_acs[ac_nr].enu_pos_i, &state.ned_origin_i, acInfoGetPositionLla_i(ac_id));
      SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I);
      ENU_FLOAT_OF_BFP(ti_acs[ac_nr].enu_pos_f, ti_acs[ac_nr].enu_pos_i);
    }
  } else if (state.utm_initialized_f)
  {
    /* if utm origin is initialized we use the ENU = UTM - UTM_ORIGIN as in state to facilitate comparison */
    ENU_OF_UTM_DIFF(ti_acs[ac_nr].enu_pos_f, *acInfoGetPositionUtm_f(ac_id), state.utm_origin_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
}

/* compute ENU velocity of aircraft with ac_id (int) */
void acInfoCalcVelocityEnu_i(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F))
  {
    SPEEDS_BFP_OF_REAL(ti_acs[ac_nr].enu_vel_i, ti_acs[ac_nr].enu_vel_f);
  } else {
    ti_acs[ac_nr].enu_vel_f.x = ti_acs[ac_nr].gspeed * sinf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.y = ti_acs[ac_nr].gspeed * cosf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.z = ti_acs[ac_nr].climb;
    SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F);
    SPEEDS_BFP_OF_REAL(ti_acs[ac_nr].enu_vel_i, ti_acs[ac_nr].enu_vel_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I);
}

/* compute ENU position of aircraft with ac_id (float) */
void acInfoCalcVelocityEnu_f(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I))
  {
    SPEEDS_FLOAT_OF_BFP(ti_acs[ac_nr].enu_vel_f, ti_acs[ac_nr].enu_vel_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_LOCAL_F)) {
    ti_acs[ac_nr].enu_vel_f.x = ti_acs[ac_nr].gspeed * sinf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.y = ti_acs[ac_nr].gspeed * cosf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.z = ti_acs[ac_nr].climb;
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F);
}
