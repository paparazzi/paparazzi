/*
 *
 * Copyright (C) 2009-2011 The Paparazzi Team
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
 *
 */

/**
 * @file mcu_periph/sys_time.c
 * @brief Architecture independent timing functions.
 *
 */

#include "mcu_periph/sys_time.h"
#include "mcu.h"
#include "pprzlink/dl_protocol.h"

// check the number of timers against max tid_t value
#include "limits.h"
#if (SYS_TIME_NB_TIMER >= SCHAR_MAX)
  WARNING("Too many sys timers (SYS_TIME_NB_TIMER >= SCHAR_MAX). Consider increasing the size of tid_d type (currently int8_t")
#endif

PRINT_CONFIG_VAR(SYS_TIME_FREQUENCY)

struct sys_time sys_time;

/** Time without GPS sync in ms before the ground time is used */
#ifndef SYS_TIME_GPS_SYNC_TIMEOUT
#define SYS_TIME_GPS_SYNC_TIMEOUT 10000
#endif

/** Period of the RTC updates in ms */
#ifndef SYS_TIME_RTC_SYNC_PERIOD
#define SYS_TIME_RTC_SYNC_PERIOD 5000
#endif

/** Minimum difference in ms before the ground time is applied again,
 * avoids steps (also backwards) from the round trip jitter of the link */
#ifndef SYS_TIME_GROUND_SYNC_THRESHOLD
#define SYS_TIME_GROUND_SYNC_THRESHOLD 200
#endif

#define SYS_TIME_GPS_LEAP_MS ((uint64_t)SYS_TIME_GPS_LEAP_SECONDS * 1000)

/** Milliseconds since startup without the ~49.7 days wrap of get_sys_time_msec()
 * nb_sec gives the number of wraps, get_sys_time_msec() the exact time */
static uint64_t uptime_msec(void)
{
  uint32_t ms = get_sys_time_msec();
  int64_t diff = (int64_t)sys_time.nb_sec * 1000 - ms;
  uint64_t wraps = (uint64_t)((diff + (1LL << 31)) >> 32);
  return (wraps << 32) + ms;
}

uint32_t get_sys_time_tow(void)
{
  return (sys_time.tow_offset + uptime_msec() % SYS_TIME_MSEC_PER_WEEK) % SYS_TIME_MSEC_PER_WEEK;
}

uint64_t get_sys_time_unix_msec(void)
{
  // 64-bit is not read/written atomically on 32-bit MCUs,
  // fine as long as the syncs and the users run in the same thread
  if (!sys_time.unix_synced) {
    return 0;
  }
  return sys_time.unix_offset + uptime_msec();
}

uint32_t get_sys_time_unix(void)
{
  return (uint32_t)(get_sys_time_unix_msec() / 1000);
}

/** uptime_msec() at the moment of the timestamp in usec
 * the usec stamp wraps after ~71 min, so only use its age with respect to now */
static uint64_t msec_of_stamp(uint32_t stamp, uint64_t *now)
{
  *now = uptime_msec();
  uint32_t age_ms = (get_sys_time_usec() - stamp) / 1000;
  return *now - age_ms;
}

static void set_tow(uint32_t tow, uint64_t msec)
{
  sys_time.tow_offset = (tow % SYS_TIME_MSEC_PER_WEEK + SYS_TIME_MSEC_PER_WEEK - msec % SYS_TIME_MSEC_PER_WEEK)
                        % SYS_TIME_MSEC_PER_WEEK;
}

static void set_unix(uint64_t unix_ms, uint64_t msec, uint64_t now)
{
  sys_time.unix_offset = unix_ms - msec;
  sys_time.unix_synced = true;

  // Keep the RTC in sync
  if (sys_time.rtc_sync == 0 || (uint32_t)now - sys_time.rtc_sync >= SYS_TIME_RTC_SYNC_PERIOD) {
    sys_time.rtc_sync = (uint32_t)now;
    uint64_t unix_now = sys_time.unix_offset + now;
    sys_time_rtc_set((uint32_t)(unix_now / 1000), (uint16_t)(unix_now % 1000));
  }
}

void sys_time_sync_gps(uint16_t week, uint32_t tow, uint32_t stamp)
{
  uint64_t now;
  uint64_t msec = msec_of_stamp(stamp, &now);

  set_tow(tow, msec);
  sys_time.gps_tow_sync = (uint32_t)now;
  sys_time.gps_tow_synced = true;

  if (week != 0) {
    uint64_t gps_ms = (uint64_t)week * SYS_TIME_MSEC_PER_WEEK + tow;
    set_unix(gps_ms + SYS_TIME_UNIX_GPS_EPOCH_MS - SYS_TIME_GPS_LEAP_MS, msec, now);
    sys_time.gps_unix_sync = (uint32_t)now;
    sys_time.gps_unix_synced = true;
  }
}

void sys_time_sync_unix(uint32_t unix_time, uint16_t unix_ms, uint32_t stamp)
{
  uint64_t unix_msec = (uint64_t)unix_time * 1000 + unix_ms;
  if (unix_msec < SYS_TIME_UNIX_GPS_EPOCH_MS) {
    return; // invalid time
  }

  uint64_t now;
  uint64_t msec = msec_of_stamp(stamp, &now);

  // GPS has priority
  bool use_unix = !sys_time.gps_unix_synced || (uint32_t)now - sys_time.gps_unix_sync > SYS_TIME_GPS_SYNC_TIMEOUT;
  bool use_tow = !sys_time.gps_tow_synced || (uint32_t)now - sys_time.gps_tow_sync > SYS_TIME_GPS_SYNC_TIMEOUT;
  if (!use_unix && !use_tow) {
    return;
  }

  // Only step when the difference with the last applied ground time is significant
  uint64_t offset = unix_msec - msec;
  int64_t diff = (int64_t)(offset - sys_time.ground_offset);
  if (sys_time.ground_synced && diff < SYS_TIME_GROUND_SYNC_THRESHOLD && diff > -SYS_TIME_GROUND_SYNC_THRESHOLD) {
    return;
  }
  sys_time.ground_offset = offset;
  sys_time.ground_synced = true;

  if (use_unix) {
    set_unix(unix_msec, msec, now);
  }
  if (use_tow) {
    uint64_t gps_ms = unix_msec - SYS_TIME_UNIX_GPS_EPOCH_MS + SYS_TIME_GPS_LEAP_MS;
    set_tow((uint32_t)(gps_ms % SYS_TIME_MSEC_PER_WEEK), msec);
  }
}

tid_t sys_time_register_timer(float duration, sys_time_cb cb)
{
  uint32_t start_time = sys_time.nb_tick;
  for (tid_t i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (!sys_time.timer[i].in_use) {
      sys_time.timer[i].cb         = cb;
      sys_time.timer[i].elapsed    = false;
      sys_time.timer[i].end_time   = start_time + sys_time_ticks_of_sec(duration);
      sys_time.timer[i].duration   = sys_time_ticks_of_sec(duration);
      sys_time.timer[i].in_use     = true;
      return i;
    }
  }
  return -1;
}

tid_t sys_time_register_timer_offset(tid_t timer, float offset, sys_time_cb cb)
{
  if (timer >=0 && timer < SYS_TIME_NB_TIMER) {
    for (tid_t i = 0; i < SYS_TIME_NB_TIMER; i++) {
      if (!sys_time.timer[i].in_use && (timer < i)) { // timer should be already registered
        sys_time.timer[i].cb         = cb;
        sys_time.timer[i].elapsed    = false;
        sys_time.timer[i].end_time   = sys_time.timer[timer].end_time + sys_time_ticks_of_sec(offset); // add offset to end time
        sys_time.timer[i].duration   = sys_time.timer[timer].duration; // copy duration
        sys_time.timer[i].in_use     = true;
        return i;
      }
    }
  }
  return -1;
}

void sys_time_cancel_timer(tid_t id)
{
  sys_time.timer[id].in_use     = false;
  sys_time.timer[id].cb         = NULL;
  sys_time.timer[id].elapsed    = false;
  sys_time.timer[id].end_time   = 0;
  sys_time.timer[id].duration   = 0;
}

// FIXME: race condition ??
void sys_time_update_timer(tid_t id, float duration)
{
  sys_time.timer[id].end_time -= (sys_time.timer[id].duration - sys_time_ticks_of_sec(duration));
  sys_time.timer[id].duration = sys_time_ticks_of_sec(duration);
}

void sys_time_parse_TIME_SYNC(uint8_t *buf)
{
  sys_time_sync_unix(pprzlink_get_DL_TIME_SYNC_unix_time(buf), pprzlink_get_DL_TIME_SYNC_unix_ms(buf),
                     get_sys_time_usec());
}

#if USE_SHELL
#include "modules/core/shell.h"
#include "printf.h"
#include "string.h"

static void cmd_sys_time(shell_stream_t *sh, int argc, const char * const argv[])
{
  (void) argv;
  if (argc > 0) {
    chprintf(sh, "Usage: sys_time\r\n");
    return;
  }

  chprintf(sh, "Current sys_time\r\n");
  chprintf(sh, "  nb_sec: %lu\r\n", sys_time.nb_sec);
  chprintf(sh, "  nb_sec_rem: %lu\r\n", sys_time.nb_sec_rem);
  chprintf(sh, "  nb_tick: %lu\r\n", sys_time.nb_tick);
  chprintf(sh, "  tow: %lu\r\n", get_sys_time_tow());
  chprintf(sh, "  unix: %lu\r\n", get_sys_time_unix());

  uint32_t rtc_ms = sys_time_rtc_millisecond();
  uint32_t rtc_h = rtc_ms / 3600000;
  uint32_t rtc_m = (rtc_ms % 3600000) / 60000;
  float rtc_s = (rtc_ms % 60000) / 1000.0;
  chprintf(sh, "  rtc_time: %02u:%02u:%02.03f\r\n", rtc_h, rtc_m, rtc_s);

  chprintf(sh, "Registered timers\r\n");
  for (tid_t i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use) {
      chprintf(sh, "id: %d, duration: %lu, end_time: %lu, cb: 0x%lx\r\n", i,
          sys_time.timer[i].duration, sys_time.timer[i].end_time, sys_time.timer[i].cb);
    }
  }
}
#endif

void sys_time_init(void)
{
  sys_time.nb_sec     = 0;
  sys_time.nb_sec_rem = 0;
  sys_time.nb_tick    = 0;
  sys_time.tow_offset = 0;
  sys_time.unix_offset = 0;
  sys_time.ground_offset = 0;
  sys_time.gps_tow_sync = 0;
  sys_time.gps_unix_sync = 0;
  sys_time.rtc_sync = 0;
  sys_time.gps_tow_synced = false;
  sys_time.gps_unix_synced = false;
  sys_time.unix_synced = false;
  sys_time.ground_synced = false;

  sys_time.ticks_per_sec = SYS_TIME_FREQUENCY;
  sys_time.resolution = 1.0 / sys_time.ticks_per_sec;

  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    sys_time.timer[i].in_use     = false;
    sys_time.timer[i].cb         = NULL;
    sys_time.timer[i].elapsed    = false;
    sys_time.timer[i].end_time   = 0;
    sys_time.timer[i].duration   = 0;
  }

  sys_time_arch_init();

#if USE_SHELL
  shell_add_entry("sys_time", cmd_sys_time);
#endif
}
