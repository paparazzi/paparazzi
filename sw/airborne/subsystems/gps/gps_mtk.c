/*
 * Copyright (C) 2011 The Paparazzi Team
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

/** @file gps_mtk.c
 * @brief Mediatek MT3329 specific code
 *
 * supports:
 *   DIYDrones V1.4 protocol (AXN1.30_2278)
 *   DIYDrones V1.6 protocol (AXN1.30_2389)
 *
 * documentation is partly incorrect, see mtk.xml for what seems
 * to be "real"
 *
 */

#include "gps_mtk.h"
#include "subsystems/abi.h"
#include "led.h"

#include "mcu_periph/sys_time.h"
#include "pprzlink/pprzlink_device.h"

#ifndef MTK_GPS_LINK
#error "MTK_GPS_LINK not set"
#endif

#define MTK_DIY_OUTPUT_RATE MTK_DIY_OUTPUT_4HZ
#define OUTPUT_RATE     4

/* parser status */
#define UNINIT        0
#define GOT_SYNC1_14  1
#define GOT_SYNC2_14  2
#define GOT_CLASS_14  3
#define GOT_SYNC1_16  4
#define GOT_SYNC2_16  5
#define GOT_ID        6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8

/* last error type */
#define GPS_MTK_ERR_NONE         0
#define GPS_MTK_ERR_OVERRUN      1
#define GPS_MTK_ERR_MSG_TOO_LONG 2
#define GPS_MTK_ERR_CHECKSUM     3
#define GPS_MTK_ERR_UNEXPECTED   4
#define GPS_MTK_ERR_OUT_OF_SYNC  5

/* mediatek gps fix mask */
#define MTK_DIY_FIX_3D      3
#define MTK_DIY_FIX_2D      2
#define MTK_DIY_FIX_NONE    1


/* defines for UTC-GPS time conversion */
#define SECS_MINUTE (60)
#define SECS_HOUR   (60*60)
#define SECS_DAY    (60*60*24)
#define SECS_WEEK   (60*60*24*7)

#define isleap(x) ((((x)%400)==0) || (!(((x)%100)==0) && (((x)%4)==0)))

const int8_t DAYS_MONTH[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

struct GpsMtk gps_mtk;

#ifdef GPS_CONFIGURE
#define MTK_DIY_SET_BINARY  "$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_DIY_SET_NMEA    "$PGCMD,16,1,1,1,1,1*6B\r\n"

#define MTK_DIY_OUTPUT_1HZ  "$PMTK220,1000*1F\r\n"
#define MTK_DIY_OUTPUT_2HZ  "$PMTK220,500*2B\r\n"
#define MTK_DIY_OUTPUT_4HZ  "$PMTK220,250*29\r\n"
#define MTK_DIY_OTUPUT_5HZ  "$PMTK220,200*2C\r\n"
#define MTK_DIY_OUTPUT_10HZ "$PMTK220,100*2F\r\n"

#define MTK_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define MTK_DIY_SBAS_ON     "$PMTK313,1*2E\r\n"
#define MTK_DIY_SBAS_OFF    "$PMTK313,0*2F\r\n"

#define MTK_DIY_WAAS_ON     "$PSRF151,1*3F\r\n"
#define MTK_DIY_WAAS_OFF    "$PSRF151,0*3E\r\n"

bool gps_configuring;
static uint8_t gps_status_config;
#endif

void gps_mtk_read_message(void);
void gps_mtk_parse(uint8_t c);
void gps_mtk_msg(void);

void gps_mtk_init(void)
{
  gps_mtk.status = UNINIT;
  gps_mtk.msg_available = false;
  gps_mtk.error_cnt = 0;
  gps_mtk.error_last = GPS_MTK_ERR_NONE;
#ifdef GPS_CONFIGURE
  gps_status_config = 0;
  gps_configuring = true;
#endif
}

void gps_mtk_event(void)
{
  struct link_device *dev = &((MTK_GPS_LINK).device);

  while (dev->char_available(dev->periph)) {
    gps_mtk_parse(dev->get_byte(dev->periph));
    if (gps_mtk.msg_available) {
      gps_mtk_msg();
    }
    GpsConfigure();
  }
}

void gps_mtk_msg(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();

  gps_mtk.state.last_msg_ticks = sys_time.nb_sec_rem;
  gps_mtk.state.last_msg_time = sys_time.nb_sec;
  gps_mtk_read_message();
  if (gps_mtk.msg_class == MTK_DIY14_ID &&
      gps_mtk.msg_id == MTK_DIY14_NAV_ID) {
    if (gps_mtk.state.fix == GPS_FIX_3D) {
      gps_mtk.state.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps_mtk.state.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_MTK_ID, now_ts, &gps_mtk.state);
  }
  if (gps_mtk.msg_class == MTK_DIY16_ID &&
      gps_mtk.msg_id == MTK_DIY16_NAV_ID) {
    if (gps_mtk.state.fix == GPS_FIX_3D) {
      gps_mtk.state.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps_mtk.state.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_MTK_ID, now_ts, &gps_mtk.state);
  }
  gps_mtk.msg_available = false;
}

static void gps_mtk_time2itow(uint32_t  gps_date, uint32_t  gps_time,
                              uint16_t *gps_week, uint32_t *gps_itow)
{
  /* convert UTC date/time to GPS week/itow, we have no idea about GPS
     leap seconds for now */
  uint16_t gps_msecond = gps_time % 1000;
  uint8_t  gps_second  = (gps_time / 1000) % 100;
  uint8_t  gps_minute  = (gps_time / 100000) % 100;
  uint8_t  gps_hour    = (gps_time / 10000000) % 100;
  uint16_t gps_year    = 2000 + (gps_date % 100);
  uint8_t  gps_month   = (gps_date / 100) % 100;
  uint8_t  gps_day     = (gps_date / 10000) % 100;
  int32_t  i, days;

  *gps_week = 0;
  *gps_itow = 0;

  /* sanity checks */
  if (gps_month > 12) { return; }
  if (gps_day > (DAYS_MONTH[gps_month] +
                 ((gps_month == 1) ? isleap(gps_year) : 0))) { return; }
  if (gps_hour > 23) { return; }
  if (gps_minute > 59) { return; }
  if (gps_second > 59) { return; }

  /* days since 6-JAN-1980 */
  days = -6;
  for (i = 1980; i < gps_year; i++) { days += (365 + isleap(i)); }

  /* add days in gps_year */
  for (i = 0; i < gps_month - 1; i++) {
    days += DAYS_MONTH[i] + ((i == 1) ? isleap(gps_year) : 0);
  }
  days += gps_day;

  /* convert */
  *gps_week = (uint16_t)(days / 7);
  *gps_itow = ((days % 7) * SECS_DAY +
               gps_hour * SECS_HOUR +
               gps_minute * SECS_MINUTE +
               gps_second) * 1000 +
              gps_msecond;
}

void gps_mtk_read_message(void)
{
  if (gps_mtk.msg_class == MTK_DIY14_ID) {
    if (gps_mtk.msg_id == MTK_DIY14_NAV_ID) {
      /* get hardware clock ticks */
      gps_mtk.state.lla_pos.lat = MTK_DIY14_NAV_LAT(gps_mtk.msg_buf) * 10;
      gps_mtk.state.lla_pos.lon = MTK_DIY14_NAV_LON(gps_mtk.msg_buf) * 10;
      SetBit(gps_mtk.state.valid_fields, GPS_VALID_POS_LLA_BIT);
      // FIXME: with MTK you do not receive vertical speed
      if (sys_time.nb_sec - gps_mtk.state.last_3dfix_time < 2) {
        gps_mtk.state.ned_vel.z  = ((gps_mtk.state.hmsl -
                           MTK_DIY14_NAV_HEIGHT(gps_mtk.msg_buf) * 10) * OUTPUT_RATE) / 10;
      } else { gps_mtk.state.ned_vel.z = 0; }
      gps_mtk.state.hmsl        = MTK_DIY14_NAV_HEIGHT(gps_mtk.msg_buf) * 10;
      SetBit(gps_mtk.state.valid_fields, GPS_VALID_HMSL_BIT);
      // FIXME: with MTK you do not receive ellipsoid altitude
      gps_mtk.state.lla_pos.alt = gps_mtk.state.hmsl;
      gps_mtk.state.gspeed      = MTK_DIY14_NAV_GSpeed(gps_mtk.msg_buf);
      // FIXME: with MTK you do not receive speed 3D
      gps_mtk.state.speed_3d    = gps_mtk.state.gspeed;
      gps_mtk.state.course      = (RadOfDeg(MTK_DIY14_NAV_Heading(gps_mtk.msg_buf))) * 10;
      SetBit(gps_mtk.state.valid_fields, GPS_VALID_COURSE_BIT);
      gps_mtk.state.num_sv      = MTK_DIY14_NAV_numSV(gps_mtk.msg_buf);
      switch (MTK_DIY14_NAV_GPSfix(gps_mtk.msg_buf)) {
        case MTK_DIY_FIX_3D:
          gps_mtk.state.fix = GPS_FIX_3D;
          break;
        case MTK_DIY_FIX_2D:
          gps_mtk.state.fix = GPS_FIX_2D;
          break;
        default:
          gps_mtk.state.fix = GPS_FIX_NONE;
      }
      gps_mtk.state.tow         = MTK_DIY14_NAV_ITOW(gps_mtk.msg_buf);;
      // FIXME: with MTK DIY 1.4 you do not receive GPS week
      gps_mtk.state.week        = 0;
#ifdef GPS_LED
      if (gps_mtk.state.fix == GPS_FIX_3D) {
        LED_ON(GPS_LED);
      } else {
        LED_TOGGLE(GPS_LED);
      }
#endif
    }
  }

  if (gps_mtk.msg_class == MTK_DIY16_ID) {
    if (gps_mtk.msg_id == MTK_DIY16_NAV_ID) {
      uint32_t gps_date, gps_time;
      gps_date = MTK_DIY16_NAV_UTC_DATE(gps_mtk.msg_buf);
      gps_time = MTK_DIY16_NAV_UTC_TIME(gps_mtk.msg_buf);
      gps_mtk_time2itow(gps_date, gps_time, &gps_mtk.state.week, &gps_mtk.state.tow);
#ifdef GPS_TIMESTAMP
      /* get hardware clock ticks */
      SysTimeTimerStart(gps_mtk.state.t0);
      gps_mtk.state.t0_tow      = gps_mtk.state.tow;
      gps_mtk.state.t0_tow_frac = 0;
#endif
      gps_mtk.state.lla_pos.lat = MTK_DIY16_NAV_LAT(gps_mtk.msg_buf) * 10;
      gps_mtk.state.lla_pos.lon = MTK_DIY16_NAV_LON(gps_mtk.msg_buf) * 10;
      // FIXME: with MTK you do not receive vertical speed
      if (sys_time.nb_sec - gps_mtk.state.last_3dfix_time < 2) {
        gps_mtk.state.ned_vel.z  = ((gps_mtk.state.hmsl -
                           MTK_DIY16_NAV_HEIGHT(gps_mtk.msg_buf) * 10) * OUTPUT_RATE) / 10;
      } else { gps_mtk.state.ned_vel.z = 0; }
      gps_mtk.state.hmsl        = MTK_DIY16_NAV_HEIGHT(gps_mtk.msg_buf) * 10;
      SetBit(gps_mtk.state.valid_fields, GPS_VALID_HMSL_BIT);
      // FIXME: with MTK you do not receive ellipsoid altitude
      gps_mtk.state.lla_pos.alt = gps_mtk.state.hmsl;
      gps_mtk.state.gspeed      = MTK_DIY16_NAV_GSpeed(gps_mtk.msg_buf);
      // FIXME: with MTK you do not receive speed 3D
      gps_mtk.state.speed_3d    = gps_mtk.state.gspeed;
      gps_mtk.state.course      = (RadOfDeg(MTK_DIY16_NAV_Heading(gps_mtk.msg_buf) * 10000)) * 10;
      SetBit(gps_mtk.state.valid_fields, GPS_VALID_COURSE_BIT);
      gps_mtk.state.num_sv      = MTK_DIY16_NAV_numSV(gps_mtk.msg_buf);
      switch (MTK_DIY16_NAV_GPSfix(gps_mtk.msg_buf)) {
        case MTK_DIY_FIX_3D:
          gps_mtk.state.fix = GPS_FIX_3D;
          break;
        case MTK_DIY_FIX_2D:
          gps_mtk.state.fix = GPS_FIX_2D;
          break;
        default:
          gps_mtk.state.fix = GPS_FIX_NONE;
      }
      /* HDOP? */
#ifdef GPS_LED
      if (gps_mtk.state.fix == GPS_FIX_3D) {
        LED_ON(GPS_LED);
      } else {
        LED_TOGGLE(GPS_LED);
      }
#endif
    }
  }
}

/* byte parsing */
void gps_mtk_parse(uint8_t c)
{
  if (gps_mtk.status < GOT_PAYLOAD) {
    gps_mtk.ck_a += c;
    gps_mtk.ck_b += gps_mtk.ck_a;
  }
  switch (gps_mtk.status) {
    case UNINIT:
      if (c == MTK_DIY14_SYNC1) {
        gps_mtk.status = GOT_SYNC1_14;
      }
      if (c == MTK_DIY16_ID) {
        gps_mtk.msg_class = c;
      }
      gps_mtk.status = GOT_SYNC1_16;
      break;
      /* MTK_DIY_VER_14 */
    case GOT_SYNC1_14:
      if (c != MTK_DIY14_SYNC2) {
        gps_mtk.error_last = GPS_MTK_ERR_OUT_OF_SYNC;
        goto error;
      }
      if (gps_mtk.msg_available) {
        /* Previous message has not yet been parsed: discard this one */
        gps_mtk.error_last = GPS_MTK_ERR_OVERRUN;
        goto error;
      }
      gps_mtk.ck_a = 0;
      gps_mtk.ck_b = 0;
      gps_mtk.status++;
      gps_mtk.len = MTK_DIY14_NAV_LENGTH;
      break;
    case GOT_SYNC2_14:
      if (c != MTK_DIY14_ID) {
        gps_mtk.error_last = GPS_MTK_ERR_OUT_OF_SYNC;
        goto error;
      }
      gps_mtk.msg_class = c;
      gps_mtk.msg_idx = 0;
      gps_mtk.status++;
      break;
    case GOT_CLASS_14:
      if (c != MTK_DIY14_NAV_ID) {
        gps_mtk.error_last = GPS_MTK_ERR_OUT_OF_SYNC;
        goto error;
      }
      gps_mtk.msg_id = c;
      gps_mtk.status = GOT_ID;
      break;
      /* MTK_DIY_VER_16 */
    case GOT_SYNC1_16:
      if (c != MTK_DIY16_NAV_ID) {
        gps_mtk.error_last = GPS_MTK_ERR_OUT_OF_SYNC;
        goto error;
      }
      if (gps_mtk.msg_available) {
        /* Previous message has not yet been parsed: discard this one */
        gps_mtk.error_last = GPS_MTK_ERR_OVERRUN;
        goto error;
      }
      gps_mtk.msg_id = c;
      gps_mtk.ck_a = 0;
      gps_mtk.ck_b = 0;
      gps_mtk.status++;
      break;
    case GOT_SYNC2_16:
      gps_mtk.len = c;
      gps_mtk.msg_idx = 0;
      gps_mtk.status = GOT_ID;
      break;
    case GOT_ID:
      gps_mtk.msg_buf[gps_mtk.msg_idx] = c;
      gps_mtk.msg_idx++;
      if (gps_mtk.msg_idx >= gps_mtk.len) {
        gps_mtk.status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != gps_mtk.ck_a) {
        gps_mtk.error_last = GPS_MTK_ERR_CHECKSUM;
        goto error;
      }
      gps_mtk.status++;
      break;
    case GOT_CHECKSUM1:
      if (c != gps_mtk.ck_b) {
        gps_mtk.error_last = GPS_MTK_ERR_CHECKSUM;
        goto error;
      }
      gps_mtk.msg_available = true;
      goto restart;
      break;
    default:
      gps_mtk.error_last = GPS_MTK_ERR_UNEXPECTED;
      goto error;
  }
  return;
error:
  gps_mtk.error_cnt++;
restart:
  gps_mtk.status = UNINIT;
  return;
}


/*
 * register callbacks & structs
 */
void gps_mtk_register(void)
{
  gps_register_impl(gps_mtk_init, gps_mtk_event, GPS_MTK_ID);
}

/*
 *
 *
 * GPS dynamic configuration
 *
 *
 */
#ifdef GPS_CONFIGURE

#include "pprzlink/pprzlink_device.h"

static void MtkSend_CFG(char *dat)
{
  struct link_device *dev = &((MTK_GPS_LINK).device);
  while (*dat != 0) { dev->put_byte(dev->periph, 0, *dat++); }
}

void gps_configure_uart(void)
{
}

#ifdef USER_GPS_CONFIGURE
#include USER_GPS_CONFIGURE
#else
static bool user_gps_configure(bool cpt)
{
  switch (cpt) {
    case 0:
      MtkSend_CFG(MTK_DIY_SET_BINARY);
      break;
    case 1:
      MtkSend_CFG(MTK_DIY_OUTPUT_RATE);
      return false;
    default:
      break;
  }
  return true; /* Continue, except for the last case */
}
#endif // ! USER_GPS_CONFIGURE

void gps_configure(void)
{
  static uint32_t count = 0;
  /* start configuring after having received 50 bytes */
  if (count++ > 50) {
    gps_configuring = user_gps_configure(gps_status_config++);
  }
}

#endif /* GPS_CONFIGURE */
