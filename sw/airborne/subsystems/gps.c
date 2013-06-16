/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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

/** @file gps.c
 *  @brief Device independent GPS code
 *
 */

#include "subsystems/gps.h"

#include "led.h"

#define MSEC_PER_WEEK (1000*60*60*24*7)

struct GpsState gps;

struct GpsTimeSync gps_time_sync;

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

static void send_gps(void) {
  static uint8_t i;
  int16_t climb = -gps.ned_vel.z;
  int16_t course = (DegOfRad(gps.course)/((int32_t)1e6));
  DOWNLINK_SEND_GPS(DefaultChannel, DefaultDevice, &gps.fix,
      &gps.utm_pos.east, &gps.utm_pos.north,
      &course, &gps.hmsl, &gps.gspeed, &climb,
      &gps.week, &gps.tow, &gps.utm_pos.zone, &i);
  if ((gps.fix != GPS_FIX_3D) && (i >= gps.nb_channels)) i = 0;
  if (i >= gps.nb_channels * 2) i = 0;
  if (i < gps.nb_channels && ((gps.fix != GPS_FIX_3D) || (gps.svinfos[i].cno > 0))) {
    DOWNLINK_SEND_SVINFO(DefaultChannel, DefaultDevice, &i,
        &gps.svinfos[i].svid, &gps.svinfos[i].flags,
        &gps.svinfos[i].qi, &gps.svinfos[i].cno,
        &gps.svinfos[i].elev, &gps.svinfos[i].azim);
  }
  i++;
}

static void send_gps_int(void) {
  static uint8_t i;
  static uint8_t last_cnos[GPS_NB_CHANNELS];
  DOWNLINK_SEND_GPS_INT(DefaultChannel, DefaultDevice,
      &gps.ecef_pos.x, &gps.ecef_pos.y, &gps.ecef_pos.z,
      &gps.lla_pos.lat, &gps.lla_pos.lon, &gps.lla_pos.alt,
      &gps.hmsl,
      &gps.ecef_vel.x, &gps.ecef_vel.y, &gps.ecef_vel.z,
      &gps.pacc, &gps.sacc,
      &gps.tow,
      &gps.pdop,
      &gps.num_sv,
      &gps.fix);
  if (i == gps.nb_channels) i = 0;
  if (i < gps.nb_channels && gps.svinfos[i].cno > 0 && gps.svinfos[i].cno != last_cnos[i]) {
    DOWNLINK_SEND_SVINFO(DefaultChannel, DefaultDevice, &i,
        &gps.svinfos[i].svid, &gps.svinfos[i].flags,
        &gps.svinfos[i].qi, &gps.svinfos[i].cno,
        &gps.svinfos[i].elev, &gps.svinfos[i].azim);
    last_cnos[i] = gps.svinfos[i].cno;
  }
  i++;
}

static void send_gps_lla(void) {
  uint8_t err = 0;
  int16_t climb = -gps.ned_vel.z;
  int16_t course = (DegOfRad(gps.course)/((int32_t)1e6));
  DOWNLINK_SEND_GPS_LLA(DefaultChannel, DefaultDevice,
      &gps.lla_pos.lat, &gps.lla_pos.lon, &gps.lla_pos.alt,
      &course, &gps.gspeed, &climb,
      &gps.week, &gps.tow,
      &gps.fix, &err);
}

static void send_gps_sol(void) {
  DOWNLINK_SEND_GPS_SOL(DefaultChannel, DefaultDevice, &gps.pacc, &gps.sacc, &gps.pdop, &gps.num_sv);
}
#endif

void gps_init(void) {
  gps.fix = GPS_FIX_NONE;
  gps.cacc = 0;
#ifdef GPS_LED
  LED_OFF(GPS_LED);
#endif
#ifdef GPS_TYPE_H
  gps_impl_init();
#endif

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "GPS", send_gps);
  register_periodic_telemetry(DefaultPeriodic, "GPS_INT", send_gps_int);
  register_periodic_telemetry(DefaultPeriodic, "GPS_LLA", send_gps_lla);
  register_periodic_telemetry(DefaultPeriodic, "GPS_SOL", send_gps_sol);
#endif
}

uint32_t gps_tow_from_sys_ticks(uint32_t sys_ticks)
{
  uint32_t clock_delta;
  uint32_t time_delta;
  uint32_t itow_now;

  if (sys_ticks < gps_time_sync.t0_ticks) {
    clock_delta = (0xFFFFFFFF - sys_ticks) + gps_time_sync.t0_ticks + 1;
  } else {
    clock_delta = sys_ticks - gps_time_sync.t0_ticks;
  }

  time_delta = msec_of_sys_time_ticks(clock_delta);

  itow_now = gps_time_sync.t0_tow + time_delta;
  if (itow_now > MSEC_PER_WEEK) {
    itow_now %= MSEC_PER_WEEK;
  }

  return itow_now;
}
