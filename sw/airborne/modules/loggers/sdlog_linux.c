/*
 * Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/loggers/sdlog_linux.c
 * @brief SD logger for Linux based autopilots (Bebop, ...) and NPS.
 *
 * Same interface and file layout as the ChibiOS SD logger, so the same
 * modules (flight recorder, high speed IMU logging, ...) can log, and the
 * files can be converted with sd2log:
 *   <SDLOG_DIR>/PPRZ/pprzlog_0001.LOG
 *   <SDLOG_DIR>/FLIGHT_RECORDER/fr_0001.LOG
 */

#include "modules/loggers/sdlog_chibios.h"
#include "modules/loggers/sdlog_chibios/usbStorage.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

/** Directory where the logs are stored, the default in simulation is
 * $PAPARAZZI_HOME/var/logs/sdlog/<AIRCRAFT> */
#ifndef SDLOG_DIR
#if SITL
#define SDLOG_DIR NULL
#else
#define SDLOG_DIR "/data/ftp/internal_000/sdlog"
#endif
#endif

/** Flush period in seconds */
#ifndef SDLOG_AUTO_FLUSH_PERIOD
#if SITL
#define SDLOG_AUTO_FLUSH_PERIOD 1
#else
#define SDLOG_AUTO_FLUSH_PERIOD 10
#endif
#endif

#define SDLOG_MAX_FILES 4
#define SDLOG_MAX_MSG_LEN 512

static const char PPRZ_LOG_NAME[] = "pprzlog_";
static const char PPRZ_LOG_DIR[] = "PPRZ";

/** Write the configuration of the firmware next to the logs (PPRZ/<date>_<md5>_<aircraft>.conf.gz),
 * so sd2log can decode them without the var/conf of the computer that built the firmware */
#ifndef SDLOG_CONF
#define SDLOG_CONF TRUE
#endif
#if SDLOG_CONF
#include "generated/aircraft_conf.h"
#endif
/** Result of writing the configuration, reported to the GCS */
static SdioError sdlog_conf_error = SDLOG_OK;
#if FLIGHTRECORDER_SDLOG
static const char FLIGHTRECORDER_LOG_NAME[] = "fr_";
static const char FR_LOG_DIR[] = "FLIGHT_RECORDER";
FileDes flightRecorderLogFile = -1;
#endif

FileDes pprzLogFile = -1;
struct chibios_sdlog chibios_sdlog;

/** Not available on Linux, only there for the settings */
float usb_storage_status = 0;
void usbStorage_enable_usb_storage(float e __attribute__((unused))) { usb_storage_status = 0; }

static FILE *sdlog_files[SDLOG_MAX_FILES];
static size_t sdlog_bytes = 0;
static float sdlog_last_flush = 0;
static enum {
  SDLOG_STOPPED = 0,
  SDLOG_RUNNING,
  SDLOG_ERROR
} sdlog_status = SDLOG_STOPPED;
static SdioError sdlog_error = SDLOG_OK;
static char sdlog_filenames[68];

/** Message under construction, one per thread so threads can't mix messages
 * (the fd argument of the device functions is only used with PPRZLINK_ENABLE_FD) */
struct sdlog_msg {
  uint16_t len;
  uint8_t buf[SDLOG_MAX_MSG_LEN];
};
static __thread struct sdlog_msg sdlog_msg;

static FILE *sdlog_file(FileDes fd)
{
  if (fd < 0 || fd >= SDLOG_MAX_FILES) {
    return NULL;
  }
  return sdlog_files[fd];
}

static void sdlog_write(FileDes fd, const uint8_t *data, size_t len)
{
  FILE *f = sdlog_file(fd);
  if (f == NULL) {
    return;
  }
  if (fwrite(data, 1, len, f) != len) {
    sdlog_error = SDLOG_FSFULL;
    sdlog_status = SDLOG_ERROR;
    return;
  }
  sdlog_bytes += len;

  float now = get_sys_time_float();
  if (now - sdlog_last_flush > SDLOG_AUTO_FLUSH_PERIOD || now < sdlog_last_flush) {
    sdlog_last_flush = now;
    sdLogFlushAllLogs();
  }
}

/* sdLog API */
SdioError sdLogWriteLog(const FileDes fd, const char *fmt, ...)
{
  char buf[SDLOG_MAX_MSG_LEN];
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (len < 0) {
    return SDLOG_INTERNAL_ERROR;
  }
  if ((size_t)len >= sizeof(buf)) {
    len = sizeof(buf) - 1;
  }
  sdlog_write(fd, (uint8_t *)buf, len);
  return sdlog_error;
}

SdioError sdLogWriteRaw(const FileDes fd, const uint8_t *buffer, const size_t len)
{
  sdlog_write(fd, buffer, len);
  return sdlog_error;
}

SdioError sdLogWriteByte(const FileDes fd, const uint8_t value)
{
  sdlog_write(fd, &value, 1);
  return sdlog_error;
}

SdioError sdLogFlushLog(const FileDes fd)
{
  FILE *f = sdlog_file(fd);
  if (f != NULL) {
    fflush(f);
  }
  return sdlog_error;
}

SdioError sdLogFlushAllLogs(void)
{
  for (int i = 0; i < SDLOG_MAX_FILES; i++) {
    if (sdlog_files[i] != NULL) {
      fflush(sdlog_files[i]);
    }
  }
  return sdlog_error;
}

size_t sdLogGetNbBytesWrittenToStorage(void)
{
  return sdlog_bytes;
}

SdioError sdLogGetStorageStatus(void)
{
  return sdlog_error;
}

/* Generic device API */
static int sdlog_check_free_space(struct chibios_sdlog *p, long *fd __attribute__((unused)), uint16_t len)
{
  if (sdlog_file(*(p->file)) == NULL || len > SDLOG_MAX_MSG_LEN) {
    return 0;
  }
  sdlog_msg.len = 0;
  return 1;
}

static void sdlog_put_buffer(struct chibios_sdlog *p __attribute__((unused)), long fd __attribute__((unused)),
                             const uint8_t *data, uint16_t len)
{
  if (sdlog_msg.len + len > SDLOG_MAX_MSG_LEN) {
    return;
  }
  memcpy(&sdlog_msg.buf[sdlog_msg.len], data, len);
  sdlog_msg.len += len;
}

static void sdlog_put_byte(struct chibios_sdlog *p, long fd, uint8_t byte)
{
  sdlog_put_buffer(p, fd, &byte, 1);
}

static void sdlog_send(struct chibios_sdlog *p, long fd __attribute__((unused)))
{
  sdlog_write(*(p->file), sdlog_msg.buf, sdlog_msg.len);
  sdlog_msg.len = 0;
}

static int null_function(struct chibios_sdlog *p __attribute__((unused))) { return 0; }
static uint8_t null_byte_function(struct chibios_sdlog *p __attribute__((unused))) { return 0; }

void chibios_sdlog_init(struct chibios_sdlog *sdlog, FileDes *file)
{
  sdlog->file = file;
  sdlog->device.periph = (void *)(sdlog);
  sdlog->device.check_free_space = (check_free_space_t) sdlog_check_free_space;
  sdlog->device.put_byte = (put_byte_t) sdlog_put_byte;
  sdlog->device.put_buffer = (put_buffer_t) sdlog_put_buffer;
  sdlog->device.send_message = (send_message_t) sdlog_send;
  sdlog->device.char_available = (char_available_t) null_function; // write only
  sdlog->device.get_byte = (get_byte_t) null_byte_function; // write only
}

/* Files */

/** Create a directory and its parents */
static int sdlog_mkdir(const char *path)
{
  char tmp[256];
  snprintf(tmp, sizeof(tmp), "%s", path);
  for (char *c = tmp + 1; *c; c++) {
    if (*c == '/') {
      *c = '\0';
      if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
        return -1;
      }
      *c = '/';
    }
  }
  if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
    return -1;
  }
  return 0;
}

/** Remove the empty logs and get the next free log number */
static int sdlog_next_number(const char *dir, const char *prefix)
{
  int next = 1;
  size_t plen = strlen(prefix);
  DIR *d = opendir(dir);
  if (d == NULL) {
    return next;
  }
  struct dirent *e;
  while ((e = readdir(d)) != NULL) {
    int num;
    char ext[8];
    if (strncmp(e->d_name, prefix, plen) != 0 ||
        sscanf(e->d_name + plen, "%d.%7s", &num, ext) != 2 || strcmp(ext, "LOG") != 0) {
      continue;
    }
    char path[512];
    struct stat st;
    snprintf(path, sizeof(path), "%s/%s", dir, e->d_name);
    if (stat(path, &st) == 0 && st.st_size == 0) {
      remove(path);
    } else if (num >= next) {
      next = num + 1;
    }
  }
  closedir(d);
  return next;
}

/** Open the next log file in base/dir, returns the descriptor or -1 */
static FileDes sdlog_open(const char *base, const char *dir, const char *prefix, char *name, size_t name_len)
{
  FileDes fd = -1;
  for (int i = 0; i < SDLOG_MAX_FILES; i++) {
    if (sdlog_files[i] == NULL) {
      fd = i;
      break;
    }
  }
  if (fd < 0) {
    sdlog_error = SDLOG_FDFULL;
    return -1;
  }

  char path[256];
  // a truncated path would silently log into another folder
  if (snprintf(path, sizeof(path), "%s/%s", base, dir) >= (int)sizeof(path) || sdlog_mkdir(path) != 0) {
    sdlog_error = SDLOG_NOCARD;
    return -1;
  }

  int num = sdlog_next_number(path, prefix);
  if (num > 9999) {
    sdlog_error = SDLOG_LOGNUM_ERROR;
    return -1;
  }
  snprintf(name, name_len, "%s%04d.LOG", prefix, num);

  char filename[512];
  snprintf(filename, sizeof(filename), "%s/%s", path, name);
  sdlog_files[fd] = fopen(filename, "wb");
  if (sdlog_files[fd] == NULL) {
    sdlog_error = SDLOG_FATFS_ERROR;
    return -1;
  }
  printf("[sdlog] Logging to %s\n", filename);
  return fd;
}

#if SDLOG_CONF
/** Write the configuration in base/dir if it is not there yet (under a temporary name first) */
static SdioError sdlog_write_conf(const char *base, const char *dir)
{
  char path[512], tmp[520];
  struct stat st;
  snprintf(path, sizeof(path), "%s/%s/%s", base, dir, AIRCRAFT_CONF_NAME);
  if (stat(path, &st) == 0) {
    return SDLOG_OK;
  }
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  FILE *f = fopen(tmp, "wb");
  if (f == NULL) {
    return SDLOG_FATFS_ERROR;
  }
  bool ok = fwrite(aircraft_conf_gz, 1, sizeof(aircraft_conf_gz), f) == sizeof(aircraft_conf_gz);
  ok = (fclose(f) == 0) && ok;
  if (!ok || rename(tmp, path) != 0) {
    remove(tmp);
    return ok ? SDLOG_FATFS_ERROR : SDLOG_FSFULL;
  }
  return SDLOG_OK;
}
#endif

#if PREFLIGHT_CHECKS
#include "modules/checks/preflight_checks.h"
static struct preflight_check_t sdlog_pfc;

static void sdlog_preflight(struct preflight_result_t *result)
{
  if (sdlog_status != SDLOG_RUNNING) {
    const char *cause = (sdlog_status == SDLOG_STOPPED) ? "not started" : sdlog_error_string(sdlog_error);
#ifdef SDLOG_PREFLIGHT_ERROR
    preflight_error(result, "SD log not running: %s", cause);
#else
    preflight_warning(result, "SD log not running: %s", cause);
#endif
  } else {
    preflight_success(result, "SDLogger running");
  }
}
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
/**
 * Tell the GCS in words how the log start went: the log files, or why the logger does not run
 * (no SD card, card full, ...), and when the configuration could not be written. Also every time the
 * logger goes into error later. Sent with the LOGGER_STATUS message, from the main thread.
 */
static void sdlog_report_status(struct transport_tx *trans, struct link_device *dev)
{
  static int reported = SDLOG_STOPPED;
  int status = sdlog_status;
  // only on a change: the start (stopped to running or error), and every time it goes into error
  if (status == reported || status == SDLOG_STOPPED || (status == SDLOG_RUNNING && reported == SDLOG_ERROR)) {
    reported = status;
    return;
  }
  reported = status;
  char msg[120];
  int len;
  if (status == SDLOG_ERROR) {
    len = snprintf(msg, sizeof(msg), "SD log error: %s", sdlog_error_string(sdlog_error));
  } else if (sdlog_conf_error != SDLOG_OK) {
    len = snprintf(msg, sizeof(msg), "SD log started: %s, configuration not written: %s", sdlog_filenames,
                sdlog_error_string(sdlog_conf_error));
  } else {
    len = snprintf(msg, sizeof(msg), "SD log started: %s", sdlog_filenames);
  }
  if (len > 0) {
    len = Min(len, (int)sizeof(msg) - 1);
    pprz_msg_send_INFO_MSG(trans, dev, AC_ID, len, msg);
  }
}

static void send_sdlog_status(struct transport_tx *trans, struct link_device *dev)
{
  sdlog_report_status(trans, dev);
  uint8_t status = (uint8_t) sdlog_status;
  uint8_t err = (uint8_t) sdlog_error;
  uint32_t used = (uint32_t) sdlog_bytes;
  char *filenames = sdlog_filenames;
  if (strlen(filenames) == 0) {
    filenames = "none";
  }
  pprz_msg_send_LOGGER_STATUS(trans, dev, AC_ID, &status, &err, &used, strlen(filenames), filenames);
}
#endif

static void sdlog_atexit(void)
{
  sdlog_chibios_finish(true);
}

void sdlog_chibios_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LOGGER_STATUS, send_sdlog_status);
#endif
#if PREFLIGHT_CHECKS
  preflight_check_register(&sdlog_pfc, sdlog_preflight);
#endif

  chibios_sdlog_init(&chibios_sdlog, &pprzLogFile);

  char base[256];
  const char *dir = SDLOG_DIR;
  if (dir == NULL) {
    const char *home = getenv("PAPARAZZI_HOME");
    snprintf(base, sizeof(base), "%s/var/logs/sdlog/%s", home ? home : ".", AIRFRAME_NAME);
  } else {
    snprintf(base, sizeof(base), "%s", dir);
  }

  char name[32];
  sdlog_error = SDLOG_OK;
  pprzLogFile = sdlog_open(base, PPRZ_LOG_DIR, PPRZ_LOG_NAME, name, sizeof(name));
  snprintf(sdlog_filenames, sizeof(sdlog_filenames), "%s", pprzLogFile != -1 ? name : "");
#if SDLOG_CONF
  if (pprzLogFile != -1) {
    sdlog_conf_error = sdlog_write_conf(base, PPRZ_LOG_DIR);
  }
#endif
#if FLIGHTRECORDER_SDLOG
  flightRecorderLogFile = sdlog_open(base, FR_LOG_DIR, FLIGHTRECORDER_LOG_NAME, name, sizeof(name));
  if (flightRecorderLogFile != -1) {
    size_t l = strlen(sdlog_filenames);
    snprintf(sdlog_filenames + l, sizeof(sdlog_filenames) - l, "%s%s", l > 0 ? "," : "", name);
  }
#endif

  if (sdlog_error == SDLOG_OK) {
    sdlog_status = SDLOG_RUNNING;
  } else {
    printf("[sdlog] Could not open the log files in %s (error %d)\n", base, sdlog_error);
    sdlog_status = SDLOG_ERROR;
  }
  sdlog_last_flush = get_sys_time_float();
  atexit(sdlog_atexit);
}

void sdlog_chibios_finish(const bool flush)
{
  pprzLogFile = -1;
#if FLIGHTRECORDER_SDLOG
  flightRecorderLogFile = -1;
#endif
  for (int i = 0; i < SDLOG_MAX_FILES; i++) {
    if (sdlog_files[i] != NULL) {
      if (flush) {
        fflush(sdlog_files[i]);
      }
      fclose(sdlog_files[i]);
      sdlog_files[i] = NULL;
    }
  }
  sdlog_status = SDLOG_STOPPED;
}

void logger_log_msg_up(uint8_t *buf)
{
  uint8_t ac_id = pprzlink_get_DL_INFO_MSG_UP_ac_id(buf);
  if (ac_id != AC_ID && ac_id != 0xFF) {
    return;
  }
  uint8_t fd = pprzlink_get_DL_INFO_MSG_UP_fd(buf);
  if (pprzLogFile != -1 && (fd == DEST_INFO_MSG_ALL || fd == DEST_INFO_MSG_PPRZLOG)) {
    uint8_t len = pprzlink_get_INFO_MSG_UP_msg_length(buf);
    uint8_t *msg = (uint8_t *) pprzlink_get_DL_INFO_MSG_UP_msg(buf);
    sdLogWriteRaw(pprzLogFile, msg, len);
    sdLogWriteByte(pprzLogFile, '\n');
  }
}
