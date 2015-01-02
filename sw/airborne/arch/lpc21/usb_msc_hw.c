/*
  LPCUSB, an USB device driver for LPC microcontrollers
  Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)
  adapted to pprz    Martin Mueller (martinmm@pfump.org)

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. The name of the author may not be used to endorse or promote products
     derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
    Connects a microSD card to the SPI port of the Paparazzi Tiny. Keep cables
    short, microSD card can be directly soldered to Molex cable. For now only
    non SDHC SD cards (<= 2GB) are supported. martinmm@pfump.org

    microSD         TinyV2 SPI J3
    8 nc
    7 DO            5 MISO
    6 GND           1 GND
    5 CLK           7 SCK
    4 Vcc           2 +3V3
    3 DI            4 MOSI
    2 CS            3 SSEL
    1 nc

    Looking onto the gold plated connector side of the microSD card:

    ---------------
    I 8
    I 7
    I 6
    I 5
    I 4
    I 3
    I 2
    I 1
    ------    --
          \  I  \
           --    --

*/

#include <string.h>
#include "std.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include BOARD_CONFIG

#include "lpcusb/usbapi.h"
#include "msc_bot.h"
#include "blockdev.h"
#include "usb_msc_hw.h"

#if USE_USB_SERIAL
#if PCLK < 18000000
#error PCLK needs to be higher than 18MHz for USB to work properly
#endif
#endif

#define MAX_PACKET_SIZE 64

#define LE_WORD(x)              ((x)&0xFF),((x)>>8)

static U8 abClassReqData[4];

static const U8 abDescriptors[] = {

// device descriptor
  0x12,
  DESC_DEVICE,
  LE_WORD(0x0200),    // bcdUSB
  0x00,         // bDeviceClass
  0x00,         // bDeviceSubClass
  0x00,         // bDeviceProtocol
  MAX_PACKET_SIZE0,   // bMaxPacketSize
  LE_WORD(0x7070),    // idVendor
  LE_WORD(0x1236),    // idProduct
  LE_WORD(0x0100),    // bcdDevice
  0x01,         // iManufacturer
  0x02,         // iProduct
  0x03,         // iSerialNumber
  0x01,         // bNumConfigurations

// configuration descriptor
  0x09,
  DESC_CONFIGURATION,
  LE_WORD(32),      // wTotalLength
  0x01,         // bNumInterfaces
  0x01,         // bConfigurationValue
  0x00,         // iConfiguration
  0xC0,         // bmAttributes
  0x32,         // bMaxPower

// interface
  0x09,
  DESC_INTERFACE,
  0x00,         // bInterfaceNumber
  0x00,         // bAlternateSetting
  0x02,         // bNumEndPoints
  0x08,         // bInterfaceClass = mass storage
  0x06,         // bInterfaceSubClass = transparent SCSI
  0x50,         // bInterfaceProtocol = BOT
  0x00,         // iInterface
// EP
  0x07,
  DESC_ENDPOINT,
  MSC_BULK_IN_EP,     // bEndpointAddress
  0x02,         // bmAttributes = bulk
  LE_WORD(MAX_PACKET_SIZE),// wMaxPacketSize
  0x00,         // bInterval
// EP
  0x07,
  DESC_ENDPOINT,
  MSC_BULK_OUT_EP,    // bEndpointAddress
  0x02,         // bmAttributes = bulk
  LE_WORD(MAX_PACKET_SIZE),// wMaxPacketSize
  0x00,         // bInterval

// string descriptors
  0x04,
  DESC_STRING,
  LE_WORD(0x0409),

  0x0E,
  DESC_STRING,
  'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

  0x14,
  DESC_STRING,
  'S', 0, 'D', 0, '-', 0, 'R', 0, 'e', 0, 'a', 0, 'd', 0, 'e', 0, 'r', 0,

  0x12,
  DESC_STRING,
  '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0,

// terminating zero
  0
};

#ifdef USE_USB_MSC

/*************************************************************************
  HandleClassRequest
  ==================
    Handle mass storage class request

**************************************************************************/
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
  if (pSetup->wIndex != 0) {
    return FALSE;
  }
  if (pSetup->wValue != 0) {
    return FALSE;
  }

  switch (pSetup->bRequest) {

      // get max LUN
    case 0xFE:
      *ppbData[0] = 0;    // No LUNs
      *piLen = 1;
      break;

      // MSC reset
    case 0xFF:
      if (pSetup->wLength > 0) {
        return FALSE;
      }
      MSCBotReset();
      break;

    default:
      return FALSE;
  }
  return TRUE;
}


/*************************************************************************
  main
  ====
**************************************************************************/
int main_mass_storage(void)
{
  unsigned cpsr;

  // disable global interrupts, do it polling
  cpsr = disableIRQ();

  // initialise the SD card
  BlockDevInit();

  // initialise stack
  USBInit();

  // enable bulk-in interrupts on NAKs
  // these are required to get the BOT protocol going again after a STALL
  USBHwNakIntEnable(INACK_BI);

  // register descriptors
  USBRegisterDescriptors(abDescriptors);

  // register class request handler
  USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

  // register endpoint handlers
  USBHwRegisterEPIntHandler(MSC_BULK_IN_EP, MSCBotBulkIn);
  USBHwRegisterEPIntHandler(MSC_BULK_OUT_EP, MSCBotBulkOut);

  // connect to bus
  USBHwConnect(TRUE);

  // call USB interrupt handler continuously
  while (1) {
    USBHwISR();
  }

  // possibly restore global interrupts (never happens)
  restoreIRQ(cpsr);

  return 0;
}

#endif /* USE_USB_MSC */

