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
#include "modules/loggers/sdlog_chibios.h"
#include "mcu_periph/adc.h"
#include "led.h"

#if HAL_USE_RTC
#include <rtc.h>
#include <time.h>
#include "subsystems/gps.h"
#endif

// Delay before starting SD log
#ifndef SDLOG_START_DELAY
#define SDLOG_START_DELAY 30
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
static THD_WORKING_AREA(wa_thd_startlog, 2048);
static __attribute__((noreturn)) void thd_startlog(void *arg);

/*
 * Bat survey thread
 */
static THD_WORKING_AREA(wa_thd_bat_survey, 1024);
static __attribute__((noreturn)) void thd_bat_survey(void *arg);
static void  powerOutageIsr (void);
static void systemDeepSleep (void);
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


// Functions for the generic device API
static int sdlog_check_free_space(struct chibios_sdlog* p __attribute__((unused)), long *fd __attribute__((unused)), uint16_t len __attribute__((unused)))
{
  return 1;
}

static void sdlog_transmit(struct chibios_sdlog* p, long fd __attribute__((unused)), uint8_t byte)
{
  sdLogWriteByte(*p->file, byte);
}

static void sdlog_transmit_buffer(struct chibios_sdlog* p, long fd __attribute__((unused)), uint8_t *data, uint16_t len)
{
  sdLogWriteRaw(*p->file, data, len);
}

static void sdlog_send(struct chibios_sdlog* p __attribute__((unused)), long fd __attribute__((unused))) { }

static int null_function(struct chibios_sdlog *p __attribute__((unused))) { return 0; }

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
  sdlog->device.get_byte = (get_byte_t) null_function; // write only

}

void sdlog_chibios_init(void)
{
  // Start polling on USB
  usbStorageStartPolling();

  // Start log thread
  chThdCreateStatic(wa_thd_startlog, sizeof(wa_thd_startlog),
      NORMALPRIO+2, thd_startlog, NULL);
}


void sdlog_chibios_finish(bool flush)
{
  if (pprzLogFile != -1) {
    sdLogCloseAllLogs(flush);
    sdLogFinish ();
    pprzLogFile = 0;
#if FLIGHTRECORDER_SDLOG
    flightRecorderLogFile = 0;
#endif
  }
}

static void thd_startlog(void *arg)
{
  (void) arg;
  chRegSetThreadName("start log");

  // Wait before starting the log if needed
  chThdSleepSeconds (SDLOG_START_DELAY);
  // Check if we are already in USB Storage mode
  if (usbStorageIsItRunning ())
    chThdSleepSeconds (20000); // stuck here for hours FIXME stop the thread ?

  // Init sdlog struct
  chibios_sdlog_init(&chibios_sdlog, &pprzLogFile);

  // Check for init errors
  sdOk = true;

  if (sdLogInit (NULL) != SDLOG_OK) {
    sdOk = false;
  }
  else {
    removeEmptyLogs (PPRZ_LOG_DIR, PPRZ_LOG_NAME, 50);
    if (sdLogOpenLog (&pprzLogFile, PPRZ_LOG_DIR, PPRZ_LOG_NAME, true) != SDLOG_OK)
      sdOk = false;

#if FLIGHTRECORDER_SDLOG
    removeEmptyLogs (FR_LOG_DIR, FLIGHTRECORDER_LOG_NAME, 50);
    if (sdLogOpenLog (&flightRecorderLogFile, FR_LOG_DIR, FLIGHTRECORDER_LOG_NAME, false) != SDLOG_OK)
      sdOk = false;
#endif
  }

  if (sdOk) {
    // Create Battery Survey Thread with event
    chEvtObjectInit (&powerOutageSource);
    chThdCreateStatic (wa_thd_bat_survey, sizeof(wa_thd_bat_survey),
        NORMALPRIO+2, thd_bat_survey, NULL);
  }

  while (true) {
#ifdef LED_SDLOG
    LED_TOGGLE(LED_SDLOG);
#endif
    // Blink faster if init has errors
    chThdSleepMilliseconds (sdOk == true ? 1000 : 200);
    static uint32_t timestamp = 0;

#if HAL_USE_RTC
    // FIXME this could be done somewhere else, like in sys_time
    // we sync gps time to rtc every 5 seconds
    if (chVTGetSystemTime() - timestamp > 5000) {
      timestamp = chVTGetSystemTime();
      if (gps.tow != 0) {
        // Unix timestamp of the GPS epoch 1980-01-06 00:00:00 UTC
        const uint32_t unixToGpsEpoch = 315964800;
        struct tm time_tm;
        time_t univTime = ((gps.week * 7 * 24 * 3600) + (gps.tow/1000)) + unixToGpsEpoch;
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
  chRegSetThreadName ("battery survey");
  chEvtRegister(&powerOutageSource, &powerOutageListener, 1);
  chThdSleepMilliseconds (2000);

  // FIXME: &ADCD1 and channel AD1_4 should not be hardcoded like this
  register_adc_watchdog(&ADCD1, AD1_4_CHANNEL, V_ALERT, &powerOutageIsr);

  chEvtWaitOne(EVENT_MASK(1));
  sdlog_chibios_finish (true);
  chThdExit(0);
  systemDeepSleep();
  chThdSleepMilliseconds (TIME_INFINITE);
  while (1); // never goes here, only to avoid compiler  warning: 'noreturn' function does return
}


/*
  powerOutageIsr is called within a lock zone from an isr, so no lock/unlock is needed
 */
static void  powerOutageIsr (void)
{
  chEvtBroadcastI(&powerOutageSource);
}


static void systemDeepSleep (void)
{
  chSysLock();
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  PWR->CR |= (PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
  __WFE();
  chSysUnlock();
}
