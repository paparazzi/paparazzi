/*
 * Copyright (C) 2014-2015 Gautier Hattenberger, Alexandre Bustico
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
 * @file modules/loggers/sdlog_chibios/usbStorage.c
 *
 */

#include <ch.h>
#include <hal.h>
#include "usb_msd.h"
#include "usbStorage.h"
#include "modules/loggers/sdlog_chibios.h"
#include <stdio.h>
#include <string.h>
#include "main_chibios.h"
#include "mcu.h"
#include "mcu_periph/sdio.h"
#include "led.h"

static void thdUsbStorage(void *arg);
static thread_t *usbStorageThreadPtr = NULL;
/* USB mass storage driver */
static bool isRunning = false;


/* Turns on a LED when there is I/O activity on the USB port */
static void usbActivity(bool active __attribute__((unused)))
{
#ifdef SDLOG_USB_LED
  if (active) {
    LED_ON(SDLOG_USB_LED);
  } else {
    LED_OFF(SDLOG_USB_LED);
  }
#endif
}

/* USB mass storage configuration */
static USBMassStorageConfig msdConfig = {
  &USBD1,
  (BaseBlockDevice *) &SDCD1,
  USB_MS_DATA_EP,
  &usbActivity,
  "Pprz_sd",
  "AutoPilot",
  "0.2"
};


static THD_WORKING_AREA(waThdUsbStorage, 1024);
void usbStorageStartPolling(void)
{
  usbStorageThreadPtr = chThdCreateStatic(waThdUsbStorage, sizeof(waThdUsbStorage),
                                          NORMALPRIO + 2, thdUsbStorage, NULL);

}


void usbStorageWaitForDeconnexion(void)
{
  if (usbStorageThreadPtr != NULL) {
    chThdWait(usbStorageThreadPtr);
  }
  usbStorageThreadPtr = NULL;
}

void usbStorageStop(void)
{
  if (usbStorageThreadPtr != NULL) {
    chThdTerminate(usbStorageThreadPtr);
  }
}




static void thdUsbStorage(void *arg)
{
  (void) arg;

  chRegSetThreadName("UsbStorage:polling");
  uint antiBounce = 5;
  event_listener_t connected;

  while (!chThdShouldTerminateX() && antiBounce) {
    const bool usbConnected = palReadPad(SDLOG_USB_VBUS_PORT, SDLOG_USB_VBUS_PIN);
    if (usbConnected) {
      antiBounce--;
    } else {
      antiBounce = 5;
    }

    chThdSleepMilliseconds(20);
  }
  isRunning = true;
  chRegSetThreadName("UsbStorage:connected");

  /* Stop the logs*/
  // it's not a powerloss, wa have time to flush the ram buffer
  sdlog_chibios_finish(true);


  /* connect sdcard sdc interface sdio */
  if (sdio_connect() == false) {
    chThdExit(MSG_TIMEOUT);
  }

  /* initialize the USB mass storage driver */
  init_msd_driver(NULL, &msdConfig);

  /* wait for a real usb storage connexion before shutting down autopilot */
  msd_register_evt_connected(&connected, EVENT_MASK(1));
  chEvtWaitOne(EVENT_MASK(1));

  /* stop autopilot */
  pprz_terminate_autopilot_threads();

  /* wait until usb-storage is unmount and usb cable is unplugged*/
  while (!chThdShouldTerminateX() && palReadPad(SDLOG_USB_VBUS_PORT, SDLOG_USB_VBUS_PIN)) {
    chThdSleepMilliseconds(10);
  }

  deinit_msd_driver();

  chThdSleepMilliseconds(500);

  mcu_reset();
}

bool usbStorageIsItRunning(void)
{
  return isRunning;
}

