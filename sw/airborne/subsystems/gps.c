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

#ifdef GPS_POWER_GPIO
#include "mcu_periph/gpio.h"

#ifndef GPS_POWER_GPIO_ON
#define GPS_POWER_GPIO_ON gpio_set
#endif
#endif

#define MSEC_PER_WEEK (1000*60*60*24*7)

struct GpsState gps;

struct GpsTimeSync gps_time_sync;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_svinfo_id(struct transport_tx *trans, struct link_device *dev,
                           uint8_t svid)
{
  if (svid < GPS_NB_CHANNELS) {
    pprz_msg_send_SVINFO(trans, dev, AC_ID, &svid,
                         &gps.svinfos[svid].svid, &gps.svinfos[svid].flags,
                         &gps.svinfos[svid].qi, &gps.svinfos[svid].cno,
                         &gps.svinfos[svid].elev, &gps.svinfos[svid].azim);
  }
}

/** send SVINFO message (regardless of state) */
static void send_svinfo(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t i = 0;
  if (i == gps.nb_channels) { i = 0; }
  send_svinfo_id(trans, dev, i);
  i++;
}

/** send SVINFO message if updated.
 * send SVINFO for all satellites while no GPS fix,
 * after 3D fix, send avialable sats only when there is new information
 */
static inline void send_svinfo_available(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t i = 0;
  static uint8_t last_cnos[GPS_NB_CHANNELS];
  if (i >= gps.nb_channels) { i = 0; }
  // send SVINFO for all satellites while no GPS fix,
  // after 3D fix, send avialable sats if they were updated
  if (gps.fix != GPS_FIX_3D) {
    send_svinfo_id(trans, dev, i);
  } else if (gps.svinfos[i].cno != last_cnos[i]) {
    send_svinfo_id(trans, dev, i);
    last_cnos[i] = gps.svinfos[i].cno;
  }
  i++;
}

static void send_gps(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t zero = 0;
  int16_t climb = -gps.ned_vel.z;
  int16_t course = (DegOfRad(gps.course) / ((int32_t)1e6));
  pprz_msg_send_GPS(trans, dev, AC_ID, &gps.fix,
                    &gps.utm_pos.east, &gps.utm_pos.north,
                    &course, &gps.hmsl, &gps.gspeed, &climb,
                    &gps.week, &gps.tow, &gps.utm_pos.zone, &zero);
  // send SVINFO for available satellites that have new data
  send_svinfo_available(trans, dev);
}

static void send_gps_int(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GPS_INT(trans, dev, AC_ID,
                        &gps.ecef_pos.x, &gps.ecef_pos.y, &gps.ecef_pos.z,
                        &gps.lla_pos.lat, &gps.lla_pos.lon, &gps.lla_pos.alt,
                        &gps.hmsl,
                        &gps.ecef_vel.x, &gps.ecef_vel.y, &gps.ecef_vel.z,
                        &gps.pacc, &gps.sacc,
                        &gps.tow,
                        &gps.pdop,
                        &gps.num_sv,
                        &gps.fix);
  // send SVINFO for available satellites that have new data
  send_svinfo_available(trans, dev);
}

static void send_gps_lla(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t err = 0;
  int16_t climb = -gps.ned_vel.z;
  int16_t course = (DegOfRad(gps.course) / ((int32_t)1e6));
  pprz_msg_send_GPS_LLA(trans, dev, AC_ID,
                        &gps.lla_pos.lat, &gps.lla_pos.lon, &gps.lla_pos.alt,
                        &course, &gps.gspeed, &climb,
                        &gps.week, &gps.tow,
                        &gps.fix, &err);
}

static void send_gps_sol(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GPS_SOL(trans, dev, AC_ID, &gps.pacc, &gps.sacc, &gps.pdop, &gps.num_sv);
}
#endif

void gps_init(void)
{
  gps.fix = GPS_FIX_NONE;
  gps.week = 0;
  gps.tow = 0;
  gps.cacc = 0;

  gps.last_3dfix_ticks = 0;
  gps.last_3dfix_time = 0;
  gps.last_msg_ticks = 0;
  gps.last_msg_time = 0;
#ifdef GPS_POWER_GPIO
  gpio_setup_output(GPS_POWER_GPIO);
  GPS_POWER_GPIO_ON(GPS_POWER_GPIO);
#endif
#ifdef GPS_LED
  LED_OFF(GPS_LED);
#endif
#ifdef GPS_TYPE_H
  gps_impl_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "GPS", send_gps);
  register_periodic_telemetry(DefaultPeriodic, "GPS_INT", send_gps_int);
  register_periodic_telemetry(DefaultPeriodic, "GPS_LLA", send_gps_lla);
  register_periodic_telemetry(DefaultPeriodic, "GPS_SOL", send_gps_sol);
  register_periodic_telemetry(DefaultPeriodic, "SVINFO", send_svinfo);
#endif
}

void gps_periodic_check(void)
{
  if (sys_time.nb_sec - gps.last_msg_time > GPS_TIMEOUT) {
    gps.fix = GPS_FIX_NONE;
  }
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
