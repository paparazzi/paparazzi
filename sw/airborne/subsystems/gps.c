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

#include "subsystems/abi.h"
#include "subsystems/gps.h"
#include "led.h"
#include "subsystems/settings.h"
#include "generated/settings.h"

#ifndef PRIMARY_GPS
#error "PRIMARY_GPS not set!"
#else
PRINT_CONFIG_VAR(PRIMARY_GPS)
#endif

#ifdef SECONDARY_GPS
PRINT_CONFIG_VAR(SECONDARY_GPS)
#endif

#define __RegisterGps(_x) _x ## _register()
#define _RegisterGps(_x) __RegisterGps(_x)
#define RegisterGps(_x) _RegisterGps(_x)

/** maximum number of GPS implementations that can register */
#ifdef SECONDARY_GPS
#define GPS_NB_IMPL 2
#else
#define GPS_NB_IMPL 1
#endif

#define PRIMARY_GPS_INSTANCE 0
#define SECONDARY_GPS_INSTANCE 1

#ifdef GPS_POWER_GPIO
#include "mcu_periph/gpio.h"

#ifndef GPS_POWER_GPIO_ON
#define GPS_POWER_GPIO_ON gpio_set
#endif
#endif

#define MSEC_PER_WEEK (1000*60*60*24*7)
#define TIME_TO_SWITCH 5000 //ten s in ms

struct GpsState gps;

struct GpsTimeSync gps_time_sync;

#ifdef SECONDARY_GPS
static uint8_t current_gps_id = 0;
#endif

uint8_t multi_gps_mode;

/* gps structs */
struct GpsInstance {
  ImplGpsInit init;
  ImplGpsEvent event;
  uint8_t id;
};

struct GpsInstance GpsInstances[GPS_NB_IMPL];

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
  if (gps.fix < GPS_FIX_3D) {
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
  struct UtmCoor_i utm = utm_int_from_gps(&gps, 0);
  pprz_msg_send_GPS(trans, dev, AC_ID, &gps.fix,
                    &utm.east, &utm.north,
                    &course, &gps.hmsl, &gps.gspeed, &climb,
                    &gps.week, &gps.tow, &utm.zone, &zero);
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
                        &gps.fix,
                        &gps.comp_id);
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
                        &gps.hmsl, &course, &gps.gspeed, &climb,
                        &gps.week, &gps.tow,
                        &gps.fix, &err);
}

static void send_gps_sol(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GPS_SOL(trans, dev, AC_ID, &gps.pacc, &gps.sacc, &gps.pdop, &gps.num_sv);
}
#endif

void gps_periodic_check(void)
{
  if (sys_time.nb_sec - gps.last_msg_time > GPS_TIMEOUT) {
    gps.fix = GPS_FIX_NONE;
  }
}

#ifdef SECONDARY_GPS
static uint8_t gps_multi_switch(struct GpsState *gps_s) {
  static uint32_t time_since_last_gps_switch = 0;

  if (multi_gps_mode == GPS_MODE_PRIMARY){
    return GpsInstances[PRIMARY_GPS_INSTANCE].id;
  } else if (multi_gps_mode == GPS_MODE_SECONDARY){
    return GpsInstances[SECONDARY_GPS_INSTANCE].id;
  } else{
    if (gps_s->fix > gps.fix){
      return gps_s->comp_id;
    } else if (gps.fix > gps_s->fix){
      return gps.comp_id;
    } else{
      if (get_sys_time_msec() - time_since_last_gps_switch > TIME_TO_SWITCH) {
        if (gps_s->num_sv > gps.num_sv) {
          current_gps_id = gps_s->comp_id;
          time_since_last_gps_switch = get_sys_time_msec();
        } else if (gps.num_sv > gps_s->num_sv) {
          current_gps_id = gps.comp_id;
          time_since_last_gps_switch = get_sys_time_msec();
        }
      }
    }
  }
  return current_gps_id;
}
#endif /*SECONDARY_GPS*/

static abi_event gps_ev;
static void gps_cb(uint8_t sender_id,
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  if (sender_id == GPS_MULTI_ID) {
    return;
  }
  uint32_t now_ts = get_sys_time_usec();
#ifdef SECONDARY_GPS
  current_gps_id = gps_multi_switch(gps_s);
  if (gps_s->comp_id == current_gps_id) {
    gps = *gps_s;
    AbiSendMsgGPS(GPS_MULTI_ID, now_ts, gps_s);
  }
#else
  gps = *gps_s;
  AbiSendMsgGPS(GPS_MULTI_ID, now_ts, gps_s);
#endif
  if (gps.tow != gps_time_sync.t0_tow)
  {
    gps_time_sync.t0_ticks = sys_time.nb_tick;
    gps_time_sync.t0_tow = gps.tow;
  }
}

/*
 * handle gps switching and updating gps instances
 */
void GpsEvent(void) {
  // run each gps event
  for (int i = 0 ; i < GPS_NB_IMPL ; i++) {
    if (GpsInstances[i].event != NULL) {
      GpsInstances[i].event();
    }
  }
}

/*
 * register gps structs for callback
 */
void gps_register_impl(ImplGpsInit init, ImplGpsEvent event, uint8_t id)
{
  int i;
  for (i=0; i < GPS_NB_IMPL; i++) {
    if (GpsInstances[i].init == NULL) {
      GpsInstances[i].init = init;
      GpsInstances[i].event = event;
      GpsInstances[i].id = id;
      break;
    }
  }

}

void gps_init(void)
{
  multi_gps_mode = MULTI_GPS_MODE;

  gps.valid_fields = 0;
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

  RegisterGps(PRIMARY_GPS);
#ifdef SECONDARY_GPS
  RegisterGps(SECONDARY_GPS);
#endif

  for (int i=0; i < GPS_NB_IMPL; i++) {
    if (GpsInstances[i].init != NULL) {
      GpsInstances[i].init();
    }
  }

  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS, send_gps);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_INT, send_gps_int);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_LLA, send_gps_lla);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_SOL, send_gps_sol);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SVINFO, send_svinfo);
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

/**
 * Default parser for GPS injected data
 */
void WEAK gps_inject_data(uint8_t packet_id __attribute__((unused)), uint8_t length __attribute__((unused)), uint8_t *data __attribute__((unused))){

}

/**
 * Convenience function to get utm position from GPS state
 */
struct UtmCoor_f utm_float_from_gps(struct GpsState *gps_s, uint8_t zone)
{
  struct UtmCoor_f utm;

  if (bit_is_set(gps_s->valid_fields, GPS_VALID_POS_UTM_BIT)) {
    // A real UTM position is available, use the correct zone
    utm.zone = gps_s->utm_pos.zone;
    utm.east = gps_s->utm_pos.east / 100.0f;
    utm.north = gps_s->utm_pos.north / 100.0f;
    utm.alt = gps_s->utm_pos.alt / 1000.f;
  }
  else {
    struct UtmCoor_i utm_i;
    utm_i.zone = zone;
    utm_of_lla_i(&utm_i, &gps_s->lla_pos);
    UTM_FLOAT_OF_BFP(utm, utm_i);
  }

  return utm;
}

struct UtmCoor_i utm_int_from_gps(struct GpsState *gps_s, uint8_t zone)
{
  struct UtmCoor_i utm;
  utm.zone = zone;

  if (bit_is_set(gps_s->valid_fields, GPS_VALID_POS_UTM_BIT)) {
    // A real UTM position is available, use the correct zone
    utm.zone = gps_s->utm_pos.zone;
    utm.east = gps_s->utm_pos.east;
    utm.north = gps_s->utm_pos.north;
    utm.alt = gps_s->utm_pos.alt;
  }
  else {
    /* Recompute UTM coordinates in this zone */
    utm_of_lla_i(&utm, &gps_s->lla_pos);
  }

  return utm;
}
