/*
 * Copyright (C) 2013-2015 Gautier Hattenberger, Alexandre Bustico
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

/*
 * @file modules/loggers/sdlog_chibios.c
 * @brief sdlog process with battery monitoring
 *
 */

#include <ch.h>
#include <hal.h>
#include "modules/loggers/sdlog_chibios/sdLog.h"
#include "modules/loggers/sdlog_chibios/usbStorage.h"
#include "modules/loggers/sdlog_chibios/printf.h"
#include "modules/loggers/sdlog_chibios.h"
#include "modules/tlsf/tlsf_malloc.h"
#include "mcu_periph/adc.h"
#include "mcu.h"
#include "led.h"

#if HAL_USE_RTC
#include <hal_rtc.h>
#include <time.h>
#include "modules/gps/gps.h"
#endif

// Delay before starting SD log
#ifndef SDLOG_START_DELAY
#define SDLOG_START_DELAY 30
#endif

// Auto-flush period (in seconds)
#ifndef SDLOG_AUTO_FLUSH_PERIOD
#define SDLOG_AUTO_FLUSH_PERIOD 10
#endif

// Contiguous storage memory (in Mo)
#ifndef SDLOG_CONTIGUOUS_STORAGE_MEM
#define SDLOG_CONTIGUOUS_STORAGE_MEM 50
#endif

#if (!defined USE_ADC_WATCHDOG) || (USE_ADC_WATCHDOG == 0)
#error sdlog_chibios need USE_ADC_WATCHDOG in order to properly close files when power is unplugged
#endif

#define DefaultAdcOfVoltage(voltage) ((uint32_t) (voltage/(DefaultVoltageOfAdc(1))))
static const uint16_t V_ALERT = DefaultAdcOfVoltage(5.5f);
static const char PPRZ_LOG_NAME[] = "pprzlog_";
static const char PPRZ_LOG_DIR[] = "PPRZ";

/*
 * Start log thread
 */
static IN_DMA_SECTION(THD_WORKING_AREA(wa_thd_startlog, 4096));
static __attribute__((noreturn)) void thd_startlog(void *arg);

/*
 * Bat survey thread
 */
static THD_WORKING_AREA(wa_thd_bat_survey, 1024);
static __attribute__((noreturn)) void thd_bat_survey(void *arg);
static void  powerOutageIsr(void);
event_source_t powerOutageSource;
event_listener_t powerOutageListener;

bool sdOk = false;

FileDes pprzLogFile = -1;

struct chibios_sdlog chibios_sdlog;

#if FLIGHTRECORDER_SDLOG
static const char FLIGHTRECORDER_LOG_NAME[] = "fr_";
static const char FR_LOG_DIR[] = "FLIGHT_RECORDER";
FileDes flightRecorderLogFile = -1;
#endif

/** sdlog status
 */
static enum {
  SDLOG_STOPPED = 0,
  SDLOG_RUNNING,
  SDLOG_ERROR
} chibios_sdlog_status;

/** sdlog filenames
 */
static char chibios_sdlog_filenames[68];
static char NO_FILE_NAME[] = "none";

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_sdlog_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = (uint8_t) chibios_sdlog_status;
  uint8_t errno = (uint8_t) sdLogGetStorageStatus();
  uint32_t used = (uint32_t) sdLogGetNbBytesWrittenToStorage();
  uint8_t size = strlen(chibios_sdlog_filenames);
  char *filenames = chibios_sdlog_filenames;
  if (size == 0) {
    // when no file opened
    filenames = NO_FILE_NAME;
    size = strlen(filenames);
  }
  pprz_msg_send_LOGGER_STATUS(trans, dev, AC_ID, &status, &errno, &used, size, filenames);
}
#endif


// Functions for the generic device API
static int sdlog_check_free_space(struct chibios_sdlog *p __attribute__((unused)), long *fd, uint16_t len)
{
  SdLogBuffer *sdb;
  SdioError status = sdLogAllocSDB(&sdb, len);
  if (status != SDLOG_OK) {
    return 0;
  } else {
    *fd = (long) sdb;
    return 1;
  }
}

static void sdlog_transmit(struct chibios_sdlog *p __attribute__((unused)), long fd, uint8_t byte)
{
  SdLogBuffer *sdb = (SdLogBuffer *) fd;
  uint8_t *data = (uint8_t *) sdLogGetBufferFromSDB(sdb);
  *data = byte;
  sdLogSeekBufferFromSDB(sdb, 1);
}

static void sdlog_transmit_buffer(struct chibios_sdlog *p __attribute__((unused)), long fd, uint8_t *data, uint16_t len)
{
  SdLogBuffer *sdb = (SdLogBuffer *) fd;
  memcpy(sdLogGetBufferFromSDB(sdb), data, len);
  sdLogSeekBufferFromSDB(sdb, len);
}

static void sdlog_send(struct chibios_sdlog *p, long fd)
{
  SdLogBuffer *sdb = (SdLogBuffer *) fd;
  sdLogWriteSDB(*(p->file), sdb);
}

static int null_function(struct chibios_sdlog *p __attribute__((unused))) { return 0; }
static uint8_t null_byte_function(struct chibios_sdlog *p __attribute__((unused))) { return 0; }

void chibios_sdlog_init(struct chibios_sdlog *sdlog, FileDes *file)
{
  // Store file descriptor
  sdlog->file = file;
  // Configure generic device
  sdlog->device.periph = (void *)(sdlog);
  sdlog->device.check_free_space = (check_free_space_t) sdlog_check_free_space;
  sdlog->device.put_byte = (put_byte_t) sdlog_transmit;
  sdlog->device.put_buffer = (put_buffer_t) sdlog_transmit_buffer;
  sdlog->device.send_message = (send_message_t) sdlog_send;
  sdlog->device.char_available = (char_available_t) null_function; // write only
  sdlog->device.get_byte = (get_byte_t) null_byte_function; // write only

}

void sdlog_chibios_init(void)
{
  chibios_sdlog_status = SDLOG_STOPPED;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LOGGER_STATUS, send_sdlog_status);
#endif

  // Start polling on USB
  usbStorageStartPolling();

  // Start log thread
  chThdCreateStatic(wa_thd_startlog, sizeof(wa_thd_startlog),
                    NORMALPRIO + 2, thd_startlog, NULL);
}


void sdlog_chibios_finish(const bool flush)
{
  if (pprzLogFile != -1) {
    // disable all required periph to save energy and maximize chance to flush files
    // to mass storage and avoid infamous dirty bit on filesystem
    mcu_periph_energy_save();

    // if a FF_FS_REENTRANT is true, we can umount fs without closing
    // file, fatfs lock will assure that umount is done after a write,
    // and umount will close all open files cleanly. Thats the fatest
    // way to umount cleanly filesystem.
    //
    // if FF_FS_REENTRANT is false,
    // we have to flush and close files before unmounting filesystem
#if FF_FS_REENTRANT == 0
    sdLogCloseAllLogs(flush);
#else
    (void) flush;
#endif

    sdLogFinish();
    pprzLogFile = 0;
#if FLIGHTRECORDER_SDLOG
    flightRecorderLogFile = 0;
#endif
  }
  chibios_sdlog_status = SDLOG_STOPPED;
}

static void thd_startlog(void *arg)
{
  (void) arg;
  chRegSetThreadName("start log");
  char tmpFilename[32];

  // Wait before starting the log if needed
  chThdSleepSeconds(SDLOG_START_DELAY);
  // Check if we are already in USB Storage mode
  if (usbStorageIsItRunning()) {
    chThdSleepSeconds(20000);  // stuck here for hours FIXME stop the thread ?
  }

  // Init sdlog struct
  chibios_sdlog_init(&chibios_sdlog, &pprzLogFile);

  // Check for init errors
  sdOk = true;

  if (sdLogInit(NULL) != SDLOG_OK) {
    sdOk = false;
  } else {
    removeEmptyLogs(PPRZ_LOG_DIR, PPRZ_LOG_NAME, 50);
    if (sdLogOpenLog(&pprzLogFile, PPRZ_LOG_DIR,
		     PPRZ_LOG_NAME, SDLOG_AUTO_FLUSH_PERIOD, LOG_APPEND_TAG_AT_CLOSE_DISABLED,
		     SDLOG_CONTIGUOUS_STORAGE_MEM, LOG_PREALLOCATION_DISABLED, tmpFilename, sizeof(tmpFilename)) != SDLOG_OK) {
      sdOk = false;
    }
    chsnprintf(chibios_sdlog_filenames, sizeof(chibios_sdlog_filenames), "%s", tmpFilename);
#if FLIGHTRECORDER_SDLOG
    removeEmptyLogs(FR_LOG_DIR, FLIGHTRECORDER_LOG_NAME, 50);
    if (sdLogOpenLog(&flightRecorderLogFile, FR_LOG_DIR, FLIGHTRECORDER_LOG_NAME,
		     SDLOG_AUTO_FLUSH_PERIOD, LOG_APPEND_TAG_AT_CLOSE_DISABLED,
		      SDLOG_CONTIGUOUS_STORAGE_MEM, LOG_PREALLOCATION_DISABLED, tmpFilename, sizeof(tmpFilename)) != SDLOG_OK) {
      sdOk = false;
    }
    chsnprintf(chibios_sdlog_filenames, sizeof(chibios_sdlog_filenames), "%s,%s", chibios_sdlog_filenames, tmpFilename);
#endif
  }

  if (sdOk) {
    // Create Battery Survey Thread with event
    chEvtObjectInit(&powerOutageSource);
    chThdCreateStatic(wa_thd_bat_survey, sizeof(wa_thd_bat_survey),
                      NORMALPRIO + 2, thd_bat_survey, NULL);

    chibios_sdlog_status = SDLOG_RUNNING;
  } else {
    chibios_sdlog_status = SDLOG_ERROR;
  }

  while (true) {
#ifdef SDLOG_LED
    LED_TOGGLE(SDLOG_LED);
#endif
    // Blink faster if init has errors
    chThdSleepMilliseconds(sdOk == true ? 1000 : 200);
    if (sdLogGetStorageStatus() != SDLOG_OK) {
      chibios_sdlog_status = SDLOG_ERROR;
      sdOk = false;
    } else {
      chibios_sdlog_status = SDLOG_RUNNING;
      sdOk = true;
    }

#if HAL_USE_RTC && USE_GPS
    static uint32_t timestamp = 0;
    // FIXME this could be done somewhere else, like in sys_time
    // we sync gps time to rtc every 5 seconds
    if (chVTGetSystemTime() - timestamp > TIME_S2I(5)) {
      timestamp = chVTGetSystemTime();
      if (gps.tow != 0) {
        // Unix timestamp of the GPS epoch 1980-01-06 00:00:00 UTC
        const uint32_t unixToGpsEpoch = 315964800;
        struct tm time_tm;
        time_t univTime = ((gps.week * 7 * 24 * 3600) + (gps.tow / 1000)) + unixToGpsEpoch;
        gmtime_r(&univTime, &time_tm);
        // Chibios date struct
        RTCDateTime date;
        rtcConvertStructTmToDateTime(&time_tm, 0, &date);
        rtcSetTime(&RTCD1, &date);
      }
    }
#endif

  }
}


static void thd_bat_survey(void *arg)
{
  (void)arg;
  chRegSetThreadName("battery survey");
  chEvtRegister(&powerOutageSource, &powerOutageListener, 1);
  chThdSleepMilliseconds(2000);

  register_adc_watchdog(&SDLOG_BAT_ADC, SDLOG_BAT_CHAN, V_ALERT, &powerOutageIsr);

  chEvtWaitOne(EVENT_MASK(1));
  // in case of powerloss, we should go fast and avoid to flush ram buffer
  sdlog_chibios_finish(false);
  chThdExit(0);

  // Only put to deep sleep in case there is no power on the USB
  if(palReadPad(SDLOG_USB_VBUS_PORT, SDLOG_USB_VBUS_PIN) == PAL_LOW)
    mcu_deep_sleep();
  chThdSleep(TIME_INFINITE);
  while (true); // never goes here, only to avoid compiler  warning: 'noreturn' function does return
}


/*
  powerOutageIsr is called within a lock zone from an isr, so no lock/unlock is needed
*/
static void powerOutageIsr(void)
{
  chEvtBroadcastI(&powerOutageSource);
}

