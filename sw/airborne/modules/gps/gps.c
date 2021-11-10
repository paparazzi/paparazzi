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

/**
 * @file gps.c
 * @brief Device independent GPS code.
 * This provides some general GPS functions and handles the selection of the
 * currently active GPS (if multiple ones are used).
 *
 * Each GPS implementation sends a GPS message via ABI for each new measurement,
 * which can be received by any other part (either from all or only one specific GPS).
 *
 * To make it easy to switch to the currently best (or simply the preferred) GPS at runtime,
 * the #multi_gps_mode can be set to #GPS_MODE_PRIMARY, #GPS_MODE_SECONDARY or #GPS_MODE_AUTO.
 * This re-sends the GPS message of the "selected" GPS with #GPS_MULTI_ID as sender id.
 * In the (default) GPS_MODE_AUTO mode, the GPS with the best fix is selected.
 *
 * The global #gps struct is also updated from the "selected" GPS
 * and used to send the normal GPS telemetry messages.
 */

#include "modules/core/abi.h"
#include "modules/gps/gps.h"
#include "led.h"
#include "modules/core/settings.h"
#include "generated/settings.h"
#include "math/pprz_geodetic_wgs84.h"
#include "math/pprz_geodetic.h"

#ifndef PRIMARY_GPS
#error "PRIMARY_GPS not set!"
#else
PRINT_CONFIG_VAR(PRIMARY_GPS)
#endif

#ifdef SECONDARY_GPS
PRINT_CONFIG_VAR(SECONDARY_GPS)
#endif

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
struct GpsRelposNED gps_relposned;
struct RtcmMan rtcm_man;

#ifdef SECONDARY_GPS
static uint8_t current_gps_id = GpsId(PRIMARY_GPS);
#endif

uint8_t multi_gps_mode;


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
  if (i >= gps.nb_channels) { i = 0; }
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
#if PPRZLINK_DEFAULT_VER == 2 && GPS_POS_BROADCAST
  // broadcast GPS message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = PPRZLINK_MSG_BROADCAST;
  msg.component_id = 0;
  pprzlink_msg_send_GPS(&msg,
#else
  pprz_msg_send_GPS(trans, dev, AC_ID,
#endif
                    &gps.fix,
                    &utm.east, &utm.north,
                    &course, &gps.hmsl, &gps.gspeed, &climb,
                    &gps.week, &gps.tow, &utm.zone, &zero);
  // send SVINFO for available satellites that have new data
  send_svinfo_available(trans, dev);
}

static void send_gps_rtk(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GPS_RTK(trans, dev, AC_ID,
                        &gps_relposned.iTOW,
                        &gps_relposned.refStationId,
                        &gps_relposned.relPosN, &gps_relposned.relPosE, &gps_relposned.relPosD,
                        &gps_relposned.relPosHPN, &gps_relposned.relPosHPE, &gps_relposned.relPosHPD,
                        &gps_relposned.accN, &gps_relposned.accE, &gps_relposned.accD,
                        &gps_relposned.carrSoln,
                        &gps_relposned.relPosValid,
                        &gps_relposned.diffSoln,
                        &gps_relposned.gnssFixOK);
}

static void send_gps_rxmrtcm(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GPS_RXMRTCM(trans, dev, AC_ID,
                            &rtcm_man.Cnt105,
                            &rtcm_man.Cnt177,
                            &rtcm_man.Cnt187,
                            &rtcm_man.Crc105,
                            &rtcm_man.Crc177,
                            &rtcm_man.Crc187);
}

static void send_gps_int(struct transport_tx *trans, struct link_device *dev)
{
#if PPRZLINK_DEFAULT_VER == 2 && GPS_POS_BROADCAST
  // broadcast GPS message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = PPRZLINK_MSG_BROADCAST;
  msg.component_id = 0;
  pprzlink_msg_send_GPS_INT(&msg,
#else
  pprz_msg_send_GPS_INT(trans, dev, AC_ID,
#endif
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
#if PPRZLINK_DEFAULT_VER == 2 && GPS_POS_BROADCAST
  // broadcast GPS message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = PPRZLINK_MSG_BROADCAST;
  msg.component_id = 0;
  pprzlink_msg_send_GPS_LLA(&msg,
#else
  pprz_msg_send_GPS_LLA(trans, dev, AC_ID,
#endif
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


#ifdef SECONDARY_GPS
static uint8_t gps_multi_switch(struct GpsState *gps_s)
{
  static uint32_t time_since_last_gps_switch = 0;

  if (multi_gps_mode == GPS_MODE_PRIMARY) {
    return GpsId(PRIMARY_GPS);
  } else if (multi_gps_mode == GPS_MODE_SECONDARY) {
    return GpsId(SECONDARY_GPS);
  } else {
    if (gps_s->fix > gps.fix) {
      return gps_s->comp_id;
    } else if (gps.fix > gps_s->fix) {
      return gps.comp_id;
    } else {
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


void gps_periodic_check(struct GpsState *gps_s)
{
  if (sys_time.nb_sec - gps_s->last_msg_time > GPS_TIMEOUT) {
    gps_s->fix = GPS_FIX_NONE;
  }

#ifdef SECONDARY_GPS
  current_gps_id = gps_multi_switch(gps_s);
  if (gps_s->comp_id == current_gps_id) {
    gps = *gps_s;
  }
#else
  gps = *gps_s;
#endif
}


static abi_event gps_ev;
static void gps_cb(uint8_t sender_id,
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  /* ignore callback from own AbiSendMsgGPS */
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
  if (gps.tow != gps_time_sync.t0_tow) {
    gps_time_sync.t0_ticks = sys_time.nb_tick;
    gps_time_sync.t0_tow = gps.tow;
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
  gps.hacc = 0;
  gps.vacc = 0;

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

  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS, send_gps);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_INT, send_gps_int);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_LLA, send_gps_lla);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_SOL, send_gps_sol);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SVINFO, send_svinfo);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_RTK, send_gps_rtk);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_RXMRTCM, send_gps_rxmrtcm);
#endif

  // Initializing counter variables to count the number of Rtcm msgs in the input stream(for each msg type)
  rtcm_man.Cnt105 = 0;
  rtcm_man.Cnt177 = 0;
  rtcm_man.Cnt187 = 0;
  // Initializing counter variables to count the number of messages that failed Crc Check
  rtcm_man.Crc105 = 0;
  rtcm_man.Crc177 = 0;
  rtcm_man.Crc187 = 0;
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
void WEAK gps_inject_data(uint8_t packet_id __attribute__((unused)), uint8_t length __attribute__((unused)),
                          uint8_t *data __attribute__((unused)))
{

}

/**
 * Convenience functions to get utm position from GPS state
 */
#include "state.h"
struct UtmCoor_f utm_float_from_gps(struct GpsState *gps_s, uint8_t zone)
{
  struct UtmCoor_f utm = {.east = 0., .north = 0., .alt = 0., .zone = zone};

  if (bit_is_set(gps_s->valid_fields, GPS_VALID_POS_UTM_BIT)) {
    /* A real UTM position is available, use the correct zone */
    UTM_FLOAT_OF_BFP(utm, gps_s->utm_pos);
  } else if (bit_is_set(gps_s->valid_fields, GPS_VALID_POS_LLA_BIT)) {
    /* Recompute UTM coordinates in this zone */
    struct UtmCoor_i utm_i;
    utm_i.zone = zone;
    utm_of_lla_i(&utm_i, &gps_s->lla_pos);
    UTM_FLOAT_OF_BFP(utm, utm_i);

    /* set utm.alt in hsml */
    if (bit_is_set(gps_s->valid_fields, GPS_VALID_HMSL_BIT)) {
      utm.alt = gps_s->hmsl / 1000.;
    } else {
      utm.alt = wgs84_ellipsoid_to_geoid_i(gps_s->lla_pos.lat, gps_s->lla_pos.lon) / 1000.;
    }
  }

  return utm;
}

struct UtmCoor_i utm_int_from_gps(struct GpsState *gps_s, uint8_t zone)
{
  struct UtmCoor_i utm = {.east = 0, .north = 0, .alt = 0, .zone = zone};

  if (bit_is_set(gps_s->valid_fields, GPS_VALID_POS_UTM_BIT)) {
    // A real UTM position is available, use the correct zone
    UTM_COPY(utm, gps_s->utm_pos);
  } else if (bit_is_set(gps_s->valid_fields, GPS_VALID_POS_LLA_BIT)) {
    /* Recompute UTM coordinates in zone */
    utm_of_lla_i(&utm, &gps_s->lla_pos);

    /* set utm.alt in hsml */
    if (bit_is_set(gps_s->valid_fields, GPS_VALID_HMSL_BIT)) {
      utm.alt = gps_s->hmsl;
    } else {
      utm.alt = wgs84_ellipsoid_to_geoid_i(gps_s->lla_pos.lat, gps_s->lla_pos.lon);
    }
  }

  return utm;
}

/**
 * GPS week number roll-over workaround application note
 */

// known day_of_year for each month:
// Major index 0 is for non-leap years, and 1 is for leap years
// Minor index is for month number 1 .. 12, 0 at index 0 is number of days before January
static const uint16_t month_days[2][13] = {
  { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 },
  { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 }
};

// Count the days since start of 1980
// Counts year * 356 days + leap days + month lengths + days in month
// The leap days counting needs the "+ 1" because GPS year 0 (i.e. 1980) was a leap year
uint16_t gps_day_number(uint16_t year, uint8_t month, uint8_t day)
{
  uint16_t gps_years = year - 1980;
  uint16_t leap_year = (gps_years % 4 == 0) ? 1 : 0;
  uint16_t day_of_year = month_days[leap_year][month - 1] + day;
  if (gps_years == 0)
    return day_of_year;
  return gps_years * 365 + ((gps_years - 1) / 4) + 1 + day_of_year;
}

uint16_t gps_week_number(uint16_t year, uint8_t month, uint8_t day)
{
  return gps_day_number(year, month, day) / 7;
}

