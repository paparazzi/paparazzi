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
EventSource powerOutageSource;
EventListener powerOutageListener;


FIL pprzLogFile = {0};

#if LOG_PROCESS_STATE
static const char PROCESS_LOG_NAME[] = "processLog_";
FIL processLogFile = {0};
#endif

static WORKING_AREA(waThdBatterySurvey, 4096);
static void launchBatterySurveyThread (void)
{

  chThdCreateStatic (waThdBatterySurvey, sizeof(waThdBatterySurvey),
      NORMALPRIO+2, batterySurveyThd, NULL);

}


bool_t chibios_logInit(const bool_t binaryFile)
{
  nvicSetSystemHandlerPriority(HANDLER_PENDSV,
             CORTEX_PRIORITY_MASK(15));

  if (sdLogInit (NULL) != SDLOG_OK)
    goto error;

  if (sdLogOpenLog (&pprzLogFile, PPRZ_LOG_DIR, PPRZ_LOG_NAME) != SDLOG_OK)
    goto error;

#if LOG_PROCESS_STATE
  if (sdLogOpenLog (&processLogFile, PROCESS_LOG_NAME) != SDLOG_OK)
    goto error;
#endif

  if  (sdLoglaunchThread (binaryFile) != SDLOG_OK)
    goto error;

  chEvtInit (&powerOutageSource);

  launchBatterySurveyThread ();

  return TRUE;

error:
  return FALSE;
}


void chibios_logFinish(void)
{
  if (pprzLogFile.fs != NULL) {
    sdLogStopThread ();
    sdLogCloseLog (&pprzLogFile);
#if LOG_PROCESS_STATE
    sdLogCloseLog (&processLogFile);
#endif
    sdLogFinish ();
    pprzLogFile.fs = NULL;
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
  chibios_logFinish ();
  chThdExit(0);
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
