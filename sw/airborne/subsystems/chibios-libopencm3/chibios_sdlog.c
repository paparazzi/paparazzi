/*
 * Copyright (C) 2013 Gautier Hattenberger, Alexandre Bustico
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
 * @file subsystems/chibios-libopencm3/chibios_sdlog.c
 * @brief sdlog process with battery monitoring
 *
 */

#include <ch.h>
#include <hal.h>
#include "subsystems/chibios-libopencm3/sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
#include "mcu_periph/adc.h"

#define DefaultAdcOfVoltage(voltage) ((uint32_t) (voltage/(DefaultVoltageOfAdc(1))))
static const uint16_t V_ALERT = DefaultAdcOfVoltage(5.0f);
static const char PPRZ_LOG_NAME[] = "pprzlog_";
static const char PPRZ_LOG_DIR[] = "PPRZ";

static msg_t batterySurveyThd(void *arg);
static void  launchBatterySurveyThread (void);
static void  powerOutageIsr (void);
static void systemDeepSleep (void);
EventSource powerOutageSource;
EventListener powerOutageListener;


FileDes pprzLogFile = -1;

struct chibios_sdlog chibios_sdlog;

#if FLIGHTRECORDER_SDLOG
static const char FLIGHTRECORDER_LOG_NAME[] = "fr_";
static const char FR_LOG_DIR[] = "FLIGHT_RECORDER";
FileDes flightRecorderLogFile = -1;
#endif

static WORKING_AREA(waThdBatterySurvey, 4096);
static void launchBatterySurveyThread (void)
{

  chThdCreateStatic (waThdBatterySurvey, sizeof(waThdBatterySurvey),
      NORMALPRIO+2, batterySurveyThd, NULL);

}

// Functions for the generic device API
static int sdlog_check_free_space(struct chibios_sdlog* p __attribute__((unused)), uint8_t len __attribute__((unused)))
{
  return TRUE;
}

static void sdlog_transmit(struct chibios_sdlog* p, uint8_t byte)
{
  sdLogWriteByte(*p->file, byte);
}

static void sdlog_send(struct chibios_sdlog* p __attribute__((unused))) { }

static int null_function(struct chibios_sdlog *p __attribute__((unused))) { return 0; }

void chibios_sdlog_init(struct chibios_sdlog *sdlog, FileDes *file)
{
  // Store file descriptor
  sdlog->file = file;
  // Configure generic device
  sdlog->device.periph = (void *)(sdlog);
  sdlog->device.check_free_space = (check_free_space_t) sdlog_check_free_space;
  sdlog->device.put_byte = (put_byte_t) sdlog_transmit;
  sdlog->device.send_message = (send_message_t) sdlog_send;
  sdlog->device.char_available = (char_available_t) null_function; // write only
  sdlog->device.get_byte = (get_byte_t) null_function; // write only

}

bool_t chibios_logInit(void)
{
  nvicSetSystemHandlerPriority(HANDLER_PENDSV,
             CORTEX_PRIORITY_MASK(15));

  // Init sdlog struct
  chibios_sdlog_init(&chibios_sdlog, &pprzLogFile);

  if (sdLogInit (NULL) != SDLOG_OK)
    goto error;

  if (sdLogOpenLog (&pprzLogFile, PPRZ_LOG_DIR, PPRZ_LOG_NAME, TRUE) != SDLOG_OK)
    goto error;

#if FLIGHTRECORDER_SDLOG
  if (sdLogOpenLog (&flightRecorderLogFile, FR_LOG_DIR, FLIGHTRECORDER_LOG_NAME, FALSE) != SDLOG_OK)
    goto error;
#endif

  chEvtInit (&powerOutageSource);

  launchBatterySurveyThread ();

  return TRUE;

error:
  return FALSE;
}


void chibios_logFinish(bool_t flush)
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


static msg_t batterySurveyThd(void *arg)
{
  (void)arg;
  chRegSetThreadName ("battery survey");
  chEvtRegister(&powerOutageSource, &powerOutageListener, 1);
  chThdSleepMilliseconds (2000);

  register_adc_watchdog((uint32_t) ADC1, 4,
      V_ALERT, 0xfff, &powerOutageIsr);

  chEvtWaitOne(EVENT_MASK(1));
  chibios_logFinish (false);
  chThdExit(0);
  systemDeepSleep();
  return 0;
}


/*
  powerOutageIsr is called from an isr not managed by chibios, so it is not possible
  to directly call chibios api from this isr.
  We have to use a 2 stage system and trigger a pendSV isr that is directly managed
  by chibios

  ° if in the future we have more than one pprz(ocm3) isr which have to call
    chibios api, we will need to route the pendSV isr to the right isr routine

  ° all theses problems will vanish when we will use a chibios hal
 */
static void  powerOutageIsr (void)
{
  // trigger PendSVVector isr
  SCB_ICSR = ICSR_PENDSVSET;
}

CH_IRQ_HANDLER(PendSVVector) {
  CH_IRQ_PROLOGUE();
  chSysLockFromIsr();
  chEvtBroadcastI(&powerOutageSource);
  chSysUnlockFromIsr();
  CH_IRQ_EPILOGUE();
}

static void systemDeepSleep (void)
{
  chSysLock();
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  PWR->CR |= (PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
  __WFE();
}
