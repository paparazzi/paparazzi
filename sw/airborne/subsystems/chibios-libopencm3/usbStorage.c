/*
 * Copyright (C) 2014 Gautier Hattenberger, Alexandre Bustico
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
 * @file subsystems/chibios-libopencm3/usbStorage.c
 *
 */

#include "ch.h"
#include "hal.h"
#include "usb_msd.h"
#include "usbStorage.h"
#include "chibios_sdlog.h"
#include "chibios_init.h"
#include <stdio.h>
#include <string.h>
#include <sdio.h>

static uint8_t  nibbleToHex (uint8_t nibble);
static void	populateSerialNumberDescriptorData (void);
static msg_t    thdUsbStorage(void *arg);
static Thread*	usbStorageThreadPtr=NULL;
/* USB mass storage driver */
static USBMassStorageDriver UMSD1;
static bool isRunning = false;

/* endpoint index */
#define USB_MS_DATA_EP 1
// cortex_m4 specific
#define MCU_RESTART() {*((unsigned long *)0x0E000ED0C) = 0x05FA0004;}

/* USB device descriptor */
static const uint8_t deviceDescriptorData[] =
{
  USB_DESC_DEVICE
    (
     0x0200, /* supported USB version (2.0)                     */
     0x00,   /* device class (none, specified in interface)     */
     0x00,   /* device sub-class (none, specified in interface) */
     0x00,   /* device protocol (none, specified in interface)  */
     64,     /* max packet size of control end-point            */
     0x0483, /* vendor ID (STMicroelectronics!)                 */
     0x5740, /* product ID (STM32F407)                          */
     0x0100, /* device release number                           */
     1,      /* index of manufacturer string descriptor         */
     2,      /* index of product string descriptor              */
     3,      /* index of serial number string descriptor        */
     1       /* number of possible configurations               */
    )
};
static const USBDescriptor deviceDescriptor =
{
  sizeof(deviceDescriptorData),
  deviceDescriptorData
};

/* configuration descriptor */
static const uint8_t configurationDescriptorData[] =
{
  /* configuration descriptor */
  USB_DESC_CONFIGURATION
    (
     32,   /* total length                                             */
     1,    /* number of interfaces                                     */
     1,    /* value that selects this configuration                    */
     0,    /* index of string descriptor describing this configuration */
     0xC0, /* attributes (self-powered)                                */
     50    /* max power (100 mA)                                       */
    ),

  /* interface descriptor */
  USB_DESC_INTERFACE
    (
     0,    /* interface number                                     */
     0,    /* value used to select alternative setting             */
     2,    /* number of end-points used by this interface          */
     0x08, /* interface class (Mass Storage)                       */
     0x06, /* interface sub-class (SCSI Transparent Storage)       */
     0x50, /* interface protocol (Bulk Only)                       */
     0     /* index of string descriptor describing this interface */
    ),

  /* end-point descriptor */
  USB_DESC_ENDPOINT
    (
     USB_MS_DATA_EP | 0x80, /* address (end point index | OUT direction)      */
     USB_EP_MODE_TYPE_BULK, /* attributes (bulk)                              */
     64,                    /* max packet size                                */
     0x05                   /* polling interval (ignored for bulk end-points) */
    ),

  /* end-point descriptor */
  USB_DESC_ENDPOINT
    (
     USB_MS_DATA_EP | 0x00, /* address (end point index | IN direction)       */
     USB_EP_MODE_TYPE_BULK, /* attributes (bulk)                              */
     64,                    /* max packet size                                */
     0x05                   /* polling interval (ignored for bulk end-points) */
    )
};
static const USBDescriptor configurationDescriptor =
{
  sizeof(configurationDescriptorData),
  configurationDescriptorData
};

/* Language descriptor */
static const uint8_t languageDescriptorData[] =
{
  USB_DESC_BYTE(4),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  USB_DESC_WORD(0x0409) /* U.S. english */
};
static const USBDescriptor languageDescriptor =
{
  sizeof(languageDescriptorData),
  languageDescriptorData
};

/* Vendor descriptor */
static const uint8_t vendorDescriptorData[] =
{
  USB_DESC_BYTE(20),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  'P', 0, 'a', 0, 'p', 0, 'a', 0, 'r', 0, 'a', 0, 'z', 0, 'z', 0, 'i', 0
};
static const USBDescriptor vendorDescriptor =
{
  sizeof(vendorDescriptorData),
  vendorDescriptorData
};

/* Product descriptor */
static const uint8_t productDescriptorData[] =
{
  USB_DESC_BYTE(20),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  'A', 0, 'u', 0, 't', 0, 'o', 0, 'p', 0, 'i', 0, 'l', 0, 'o', 0, 't', 0
};
static const USBDescriptor productDescriptor =
{
  sizeof(productDescriptorData),
  productDescriptorData
};

/* Serial number descriptor from stm32 uniq id */
/* uniq id is 12 bytes which gives 24 char which require 50 bytes for usb description */
/* this array is the only one in ram (others in flash), and should be populated before */
/* usb initialisation */
uint8_t serialNumberDescriptorData[50] =
{
  USB_DESC_BYTE(50),
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
  0
};
static const USBDescriptor serialNumberDescriptor =
{
  sizeof(serialNumberDescriptorData),
  serialNumberDescriptorData
};

/* Handles GET_DESCRIPTOR requests from the USB host */
static const USBDescriptor* getDescriptor(USBDriver* usbp, uint8_t type, uint8_t index, uint16_t lang)
{
  (void)usbp;
  (void)lang;

  switch (type)
  {
    case USB_DESCRIPTOR_DEVICE:
      return &deviceDescriptor;

    case USB_DESCRIPTOR_CONFIGURATION:
      return &configurationDescriptor;

    case USB_DESCRIPTOR_STRING:
      switch (index)
      {
        case 0: return &languageDescriptor;
        case 1: return &vendorDescriptor;
        case 2: return &productDescriptor;
        case 3: return &serialNumberDescriptor;
      }
  }

  return 0;
}

/* Handles global events of the USB driver */
static void usbEvent(USBDriver* usbp, usbevent_t event)
{
  (void) usbp;

  switch (event)
  {
    case USB_EVENT_CONFIGURED:
      chSysLockFromIsr();
      msdConfigureHookI(&UMSD1);
      chSysUnlockFromIsr();
      break;

    case USB_EVENT_RESET:
    case USB_EVENT_ADDRESS:
    case USB_EVENT_SUSPEND:
    case USB_EVENT_WAKEUP:
    case USB_EVENT_STALLED:
    default:
      break;
  }
}

/* Configuration of the USB driver */
const USBConfig usbConfig =
{
  usbEvent,
  getDescriptor,
  msdRequestsHook,
  0
};

/* Turns on a LED when there is I/O activity on the USB port */
static void usbActivity(bool active)
{
  if (active)
    palSetPad(GPIOC, GPIOC_LED4);
  else
    palClearPad(GPIOC, GPIOC_LED4);
}

/* USB mass storage configuration */
static USBMassStorageConfig msdConfig =
{
  &USBD1,
  (BaseBlockDevice*)&SDCD1,
  USB_MS_DATA_EP,
  &usbActivity,
  "DPprz_sd",
  "DApogee",
  "0.1"
};


static WORKING_AREA(waThdUsbStorage, 1024);
void usbStorageStartPolling (void)
{
  populateSerialNumberDescriptorData ();
  usbStorageThreadPtr = chThdCreateStatic (waThdUsbStorage, sizeof(waThdUsbStorage),
      NORMALPRIO, thdUsbStorage, NULL);

}


void usbStorageWaitForDeconnexion (void)
{
  if (usbStorageThreadPtr != NULL)
    chThdWait (usbStorageThreadPtr);
  usbStorageThreadPtr = NULL;
}

void usbStorageStop (void)
{
  if (usbStorageThreadPtr != NULL) {
    chThdTerminate (usbStorageThreadPtr);
  }
}




static msg_t     thdUsbStorage(void *arg)
{
  (void) arg; // unused
  chRegSetThreadName("UsbStorage:polling");
  uint antiBounce=5;
  EventListener connected;

  // Should use EXTI interrupt instead of active polling,
  // but in the chibios_opencm3 implementation, since EXTI is
  // used via libopencm3, ISR are routed on pprz/opencm3 and cannot
  // be used concurrently by chibios api
  // Should be fixed when using chibios-rt branch
  while (!chThdShouldTerminate() && antiBounce) {
    const bool usbConnected = palReadPad (GPIOA, GPIOA_OTG_FS_VBUS);
    if (usbConnected)
      antiBounce--;
    else
      antiBounce=5;

    chThdSleepMilliseconds(20);
  }
  isRunning = true;
  chRegSetThreadName("UsbStorage:connected");

  /* Stop the logs*/
  chibios_logFinish (true);


  /* connect sdcard sdc interface sdio */
  if (sdioConnect () == false)
    chThdExit (RDY_TIMEOUT);

  /* initialize the USB mass storage driver */
  msdInit(&UMSD1);

  /* start the USB mass storage service */
  msdStart(&UMSD1, &msdConfig);

  /* start the USB driver */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(1000);
  usbStart(&USBD1, &usbConfig);
  usbConnectBus(&USBD1);

  /* wait for a real usb storage connexion before shutting down autopilot */
  chEvtRegisterMask(&UMSD1.evt_connected, &connected, EVENT_MASK(1));
  chEvtWaitOne(EVENT_MASK(1));

  /* stop autopilot */
  if (pprzThdPtr != NULL) {
    chThdTerminate (pprzThdPtr);
    chThdWait (pprzThdPtr);
    pprzThdPtr = NULL;
  }

  /* wait until usb-storage is unmount and usb cable is unplugged*/
  while (!chThdShouldTerminate() && palReadPad (GPIOA, GPIOA_OTG_FS_VBUS)) {
    chThdSleepMilliseconds(10);
  }


  /* then close open descriptors and reboot autopilot */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(500);
  msdStop(&UMSD1);
  sdioDisconnect ();

  MCU_RESTART();
  return RDY_OK;
}

bool usbStorageIsItRunning (void)
{
  return isRunning;
}

static uint8_t nibbleToHex (uint8_t nibble)
{
  nibble &= 0x0F;
  return nibble < 10 ? nibble + '0' : nibble + 'A' - 10;
}


static void	populateSerialNumberDescriptorData (void)
{
  const uint8_t *UniqProcessorId = (uint8_t *) 0x1FFF7A10;
  const uint8_t  UniqProcessorIdLen = 12;

  uint8_t *baseDdPtr = &serialNumberDescriptorData[2];
  for(uint32_t i=0; i<UniqProcessorIdLen; i++) {
    *baseDdPtr++ = nibbleToHex(UniqProcessorId[i] >> 4);
    *baseDdPtr++ = 0;
    *baseDdPtr++ = nibbleToHex(UniqProcessorId[i]);
    *baseDdPtr++ = 0;
  }
}

