/*
	LPCUSB Bootloader, an USB programming utility for LPC microcontrollers
	Copyright (C) 2006 Martin Mueller (martinmm@pfump.org)

	based on LPCUSB, an USB device driver for LPC microcontrollers
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

	This software is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This software is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <string.h>

#include "type.h"
#include "usbdebug.h"

#include "lpc21iap.h"
#include "console.h"
#include "usbapi.h"
#include "startup.h"

#ifndef BOOTLOADER_VERSION
#define BOOTLOADER_VERSION          0x0103
#endif

#define BAUD_RATE                   115200
#define CCLK                        60000000
#define RAM_ADDR                    0x40004000


#define LE_WORD(x)                  (x&0xFF),(x>>8)

typedef void (*IAP)(unsigned int [], unsigned int[]);
typedef void (*GO)(void);

static unsigned int rescmd[8];
static unsigned int cmd[5];
static unsigned int write_ram_adr;
static unsigned int read_mem_adr;
static int write_ram_len;
static int read_mem_len;
static char unlocked;
static char bEcho;

static unsigned int start_app;
static unsigned int start_adr;

static const U8 abDescriptors[] = {

// device descriptor
    0x12,
    DESC_DEVICE,
    LE_WORD(0x0110),            // bcdUSB
    0x00,                       // bDeviceClass
    0x00,                       // bDeviceSubClass
    0x00,                       // bDeviceProtocol
    MAX_PACKET_SIZE0,           // bMaxPacketSize
    LE_WORD(VENDOR_ID),         // idVendor
    LE_WORD(DEVICE_ID),         // idProduct
    LE_WORD(0x0100),            // bcdDevice
    0x01,                       // iManufacturer
    0x02,                       // iProduct
    0x03,                       // iSerialNumber
    0x01,                       // bNumConfigurations

// configuration descriptor
    0x09,
    DESC_CONFIGURATION,
    LE_WORD(0x12),              // wTotalLength
    0x01,                       // bNumInterfaces
    0x01,                       // bConfigurationValue
    0x00,                       // iConfiguration
    0xC0,                       // bmAttributes
    0x32,                       // bMaxPower

// interface
    0x09,
    DESC_INTERFACE,
    0x00,                       // bInterfaceNumber
    0x00,                       // bAlternateSetting
    0x00,                       // bNumEndPoints
    0xFF,                       // bInterfaceClass
    0xFF,                       // bInterfaceSubClass
    0xFF,                       // bInterfaceProtocol
    0x00,                       // iInterface

// string descriptors
    0x04,
    DESC_STRING,
    LE_WORD(0x0409),

    0x0E,
    DESC_STRING,
    'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

    0x16,
    DESC_STRING,
    'B', 0, 'o', 0, 'o', 0, 't', 0, 'l', 0, 'o', 0, 'a', 0, 'd', 0, 'e', 0, 'r', 0,

#ifdef PROC_TINYJ
    0x26,
    DESC_STRING,
    'T', 0, 'I', 0, 'N', 0, 'Y', 0, 'J', 0, ' ', 0,
    '0', 0, '1', 0, '2', 0, '3', 0,
    '4', 0, '5', 0, '6', 0, '7', 0,
    '8', 0, '9', 0, 'A', 0, 'B', 0,
#else
#ifdef PROC_TINY
    0x24,
    DESC_STRING,
    'T', 0, 'I', 0, 'N', 0, 'Y', 0, ' ', 0,
    '0', 0, '1', 0, '2', 0, '3', 0,
    '4', 0, '5', 0, '6', 0, '7', 0,
    '8', 0, '9', 0, 'A', 0, 'B', 0,
#else
#ifdef PROC_AP
    0x20,
    DESC_STRING,
    'A', 0, 'P', 0, ' ', 0,
    'F', 0, 'E', 0, 'D', 0, 'C', 0,
    'B', 0, 'A', 0, '9', 0, '8', 0,
    '7', 0, '6', 0, '5', 0, '4', 0,
#else
#ifdef PROC_FBW
    0x22,
    DESC_STRING,
    'F', 0, 'B', 0, 'W', 0, ' ', 0,
    'F', 0, 'E', 0, 'D', 0, 'C', 0,
    'B', 0, 'A', 0, '9', 0, '8', 0,
    '7', 0, '6', 0, '5', 0, '4', 0,
#else
    0x2A,
    DESC_STRING,
    'G', 0, 'E', 0, 'N', 0, 'E', 0, 'R', 0,'I', 0, 'C', 0, ' ', 0,
    '1', 0, '1', 0, '2', 0, '2', 0,
    '4', 0, '4', 0, '5', 0, '5', 0,
    '6', 0, '6', 0, '7', 0, '7', 0,
#endif
#endif
#endif
#endif

// terminating zero
    0
};


/*************************************************************************
    isp_entry
    =========
**************************************************************************/
void isp_entry(unsigned int command[5], unsigned int result[3])
{
    IAP iap_entry;

    memcpy(cmd, command, 20);

    iap_entry = (IAP) IAP_LOCATION;

    // everything should be fine
    result[0] = CMD_SUCCESS;

    switch (command[0]) {

    case ISP_UNLOCK:
        if (command[1] == UNLOCK_CODE) {
            unlocked = 1;
        }
        else {
            result[0] = INVALID_CODE;
        }
        break;

    case ISP_SET_BAUD_RATE:
        // we don't care
        break;

    case ISP_ECHO:
        if (command[1] > 1) {
            result[0] = PARAM_ERROR;
        }
        else {
            bEcho = command[1];
        }
        break;

    case ISP_WRITE_TO_RAM:
        if (command[1] & 3) result[0] = ADDR_ERROR;
        // TODO ADDR_NOT_MAPPED
        if (command[2] & 3) result[0] = COUNT_ERROR;
//        if (*(CODE_PROTECT_ADDR) == CODE_PROTECT_DATA) {
//            result[0] = CODE_READ_PROTECTION_ENABLED;
//        }
        if (result[0] == CMD_SUCCESS)
        {
            write_ram_adr = command[1];
            write_ram_len = command[2];
        }
        break;

    case ISP_READ_MEMORY:
        if (command[1] & 3) result[0] = ADDR_ERROR;
        // TODO ADDR_NOT_MAPPED
        if (command[2] & 3) result[0] = COUNT_ERROR;
//        if (*(CODE_PROTECT_ADDR) == CODE_PROTECT_DATA) {
//            result[0] = CODE_READ_PROTECTION_ENABLED;
//        }
        if (result[0] == CMD_SUCCESS)
        {
            read_mem_adr = command[1];
            read_mem_len = command[2];
        }
        break;

    case ISP_GO:
        if (command[1] & 3) result[0] = ADDR_ERROR;
        // TODO ADDR_NOT_MAPPED
        if (!unlocked) result[0] = CMD_LOCKED;
        if ((command[2] != ARM_CODE) &&
            (command[2] != THUMB_CODE)) {
            result[0] = PARAM_ERROR;
        }
//        if (*(CODE_PROTECT_ADDR) == CODE_PROTECT_DATA) {
//            result[0] = CODE_READ_PROTECTION_ENABLED;
//        }
        if (result[0] == CMD_SUCCESS) {
            start_app = 1;
            start_adr = (command[1] | (command[2] == THUMB_CODE ? 1:0));
        }
        break;

    case ISP_PREPARE_SECTORS:
        cmd[0] = IAP_PREPARE_SECTORS;
        iap_entry(cmd, result);
        break;
    case ISP_COPY_RAM_TO_FLASH:
        if (unlocked) {
            cmd[0] = IAP_COPY_RAM_TO_FLASH;
            cmd[4] = CCLK/1000;
            iap_entry(cmd, result);
        }
        else {
            result[0] = CMD_LOCKED;
        }
DBG("%X\n",*(unsigned int*) (0x40004624+4));
        break;
    case ISP_ERASE_SECTORS:
        if (unlocked) {
            cmd[0] = IAP_ERASE_SECTORS;
            cmd[3] = CCLK/1000;
            iap_entry(cmd, result);
        }
        else {
            result[0] = CMD_LOCKED;
        }
        break;
    case ISP_BLANK_CHECK_SECTORS:
        cmd[0] = IAP_BLANK_CHECK_SECTORS;
        iap_entry(cmd, result);
        break;
    case ISP_READ_PART_ID:
        cmd[0] = IAP_READ_PART_ID;
        iap_entry(cmd, result);
        break;
    case ISP_READ_BOOT_CODE_VERSION:
        cmd[0] = IAP_READ_BOOT_CODE_VERSION;
        iap_entry(cmd, result);
        break;
    case ISP_COMPARE:
        cmd[0] = IAP_COMPARE;
        iap_entry(cmd, result);
DBG("%X\n",*(unsigned int*) (0xA624-20));
DBG("%X\n",*(unsigned int*) (0xA624-16));
DBG("%X\n",*(unsigned int*) (0xA624-12));
DBG("%X\n",*(unsigned int*) (0xA624-8));
DBG("%X\n",*(unsigned int*) (0xA624-4));
DBG(" %X\n",*(unsigned int*) 0xA624);
DBG("%X\n",*(unsigned int*) (0xA624+4));
DBG("%X\n",*(unsigned int*) (0xA624+8));
DBG("%X\n",*(unsigned int*) (0xA624+12));
DBG("%X\n",*(unsigned int*) (0xA624+16));
DBG("%X\n",*(unsigned int*) (0xA624+20));
        break;

  default:
        result[0] = INVALID_COMMAND;
        break;
  }
}


/*************************************************************************
	btl_entry
	=========
**************************************************************************/
void btl_entry(unsigned int command[5], unsigned int result[3])
{
    // everything should be fine
    result[0] = CMD_SUCCESS;

    switch (command[0]) {

    case BTL_READ_VERSION:
        result[1] = BOOTLOADER_VERSION;
        break;

    case BTL_READ_LOCATION:
        if ((unsigned int) btl_entry < 0x40000000) {
            result[1] = RUN_ROM;
        }
        else {
            result[1] = RUN_RAM;
        }
        break;

    case BTL_READ_RAM_ADDR:
        result[1] = RAM_ADDR;
        break;

    default:
        result[0] = INVALID_COMMAND;
        break;
  }
}



/*************************************************************************
    HandleVendorRequest
    ===================
**************************************************************************/
static BOOL HandleVendorRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
    IAP func_entry;

    func_entry = (IAP) IAP_LOCATION;

    if (pSetup->wIndex != 0) {
        DBG("Invalid idx %X\n", pSetup->wIndex);
        return FALSE;
    }
    if (pSetup->wValue != 0) {
        DBG("Invalid val %X\n", pSetup->wValue);
        return FALSE;
    }

    switch (pSetup->bRequest) {

    // commands
    case REQ_IAP_COMMAND:
        /* fallthrough */
    case REQ_ISP_COMMAND:
        /* fallthrough */
    case REQ_BTL_COMMAND:
        if (pSetup->bRequest == REQ_IAP_COMMAND) func_entry = (IAP) IAP_LOCATION;
        if (pSetup->bRequest == REQ_ISP_COMMAND) func_entry = isp_entry;
        if (pSetup->bRequest == REQ_BTL_COMMAND) func_entry = btl_entry;

        if (REQTYPE_GET_DIR(pSetup->bmRequestType) == REQTYPE_DIR_TO_HOST) {
            // send result
            *ppbData = (U8*) rescmd;
            *piLen = 32; // TODO remove
            if (*piLen != 32) return FALSE;
        }
        else {
            // start command
            if (*piLen != 20) return FALSE;
            memcpy(&rescmd[3], *ppbData, 20);
            func_entry(&rescmd[3], rescmd);
        }
        break;

    // data transfer
    case REQ_DATA_TRANSFER:
        if (REQTYPE_GET_DIR(pSetup->bmRequestType) == REQTYPE_DIR_TO_HOST) {
            // check if data to read
            if (*piLen > read_mem_len) return FALSE;
            *ppbData = (U8*) read_mem_adr;
            read_mem_adr += *piLen;
            read_mem_len -= *piLen;
        }
        else {
            // data is already written, adjust pointers
            if (*piLen > write_ram_len) return FALSE;
            write_ram_adr += *piLen;
            write_ram_len -= *piLen;
        }
        break;

    default:
        DBG("Unhandled vendor\n");
        return FALSE;
    }
    return TRUE;
}


/*************************************************************************
    PreHandleRequest
    ================
    Change data pointer for data out request

    Returns TRUE if the pointer was changed
**************************************************************************/
static BOOL PreHandleRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
    if ((pSetup->wLength != 0) &&
        (REQTYPE_GET_DIR(pSetup->bmRequestType) == REQTYPE_DIR_TO_DEVICE) &&
        (pSetup->bRequest == REQ_DATA_TRANSFER) &&
        (*piLen <= write_ram_len)) {
            // directly write to the location given by WRITE_TO_RAM
            *ppbData = (U8*) write_ram_adr;
            return TRUE;
    }
    return FALSE;
}


/*************************************************************************
    main
    ====
**************************************************************************/
int main(void)
{
    GO go_entry;

    // PLL and MAM
    Initialize();

    // init DBG
    ConsoleInit(CCLK / (16 * BAUD_RATE));

    DBG("Initialising USB stack\n");

    write_ram_len=0;
    read_mem_len=0;
    unlocked=0;
    bEcho=1;

    // initialise stack
    USBInit();

    // register descriptors
    USBRegisterDescriptors(abDescriptors);

    // register vendor request handler
    USBRegisterRequestHandler(REQTYPE_TYPE_VENDOR, HandleVendorRequest);

    // register pre request handler
    USBRegisterPreRequestHandler(PreHandleRequest);

    DBG("Starting USB communication\n");

    // connect to bus
    USBHwConnect(TRUE);

    start_app = 0;

    // call USB interrupt handler continuously
    while (1)
    {
        USBHwISR();

        if (start_app == 1)
        {
            volatile unsigned int count;

            // wait some time until response is sent
            for (count=0;count<10000;count++) USBHwISR();

            // disconnect from bus
            USBHwConnect(FALSE);

            // wait some ms so that called app might safely re-connect usb
            for (count=0;count<300000;count++) count=count;

            go_entry = (GO) (start_adr);
            go_entry();
        }
    }

    return 0;
}
