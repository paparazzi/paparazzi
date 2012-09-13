/*  $Id$
 *
 * lpc21iap, an USB download application for Philips LPC processors
 * Copyright (C) 2006  Martin Mueller <martinmm@pfump.org>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
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

#if defined(_WIN32) && !defined(__CYGWIN__)
    #define COMPILE_FOR_WINDOWS
#elif defined(__CYGWIN__)
    #define COMPILE_FOR_CYGWIN
#else
    #define COMPILE_FOR_LINUX
#endif

#if defined COMPILE_FOR_WINDOWS || defined COMPILE_FOR_CYGWIN
#include <windows.h>
#include <io.h>
#endif // defined COMPILE_FOR_WINDOWS || defined COMPILE_FOR_CYGWIN

#if defined COMPILE_FOR_WINDOWS
#include <conio.h>
#endif // defined COMPILE_FOR_WINDOWS

#if defined COMPILE_FOR_LINUX
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#endif // defined COMPILE_FOR_LINUX

#if defined COMPILE_FOR_LINUX || defined COMPILE_FOR_CYGWIN
#include <termios.h>
#include <unistd.h>     // for read and return value of lseek
#include <sys/time.h>   // for select_time
#endif // defined COMPILE_FOR_LINUX || defined COMPILE_FOR_CYGWIN

#include <ctype.h>      // isdigit()
#include <stdio.h>      // stdout
#include <stdarg.h>
#include <time.h>
#include <fcntl.h>
#include <usb.h>        // libusb

#include "lpcusb.h"
#include "lpc21iap.h"

#if defined COMPILE_FOR_LINUX
void Sleep(unsigned long msec);
#endif // defined COMPILE_FOR_LINUX

#define USB_MAXCHUNK    4096

#define ERR_MAXLEN      40

char cUnkownError[ERR_MAXLEN] = {
"unknown error"};

char cIspError[][ERR_MAXLEN] = {
"CMD_SUCCESS",
"INVALID_COMMAND",
"SRC_ADDR_ERROR",
"DST_ADDR_ERROR",
"SRC_ADDR_NOT_MAPPED",
"DST_ADDR_NOT_MAPPED",
"COUNT_ERROR",
"INVALID_SECTOR",
"SECTOR_NOT_BLANK",
"SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION",
"COMPARE_ERROR",
"BUSY",
"PARAM_ERROR",
"ADDR_ERROR",
"ADDR_NOT_MAPPED",
"CMD_LOCKED",
"INVALID_CODE",
"INVALID_BAUD_RATE",
"INVALID_STOP_BIT",
"CODE_READ_PROTECTION_ENABLED"};

#define LPC_MAX_ERROR  (sizeof(cIspError) / sizeof(cIspError[0]))

char * IspError(unsigned char cErr)
{
    if (cErr < LPC_MAX_ERROR)
    {
        return(cIspError[cErr]);
    }
    else
    {
        return(cUnkownError);
    }
}

int USBFindDevice(usb_dev_handle *udev)
{
    int count, found;
    struct usb_bus *bus;

    found = 0;
    usb_init();
    usb_find_busses();

    for(count = 0; count < 100 && !found; count++)
    {
        printf(".");

        if (usb_find_devices())
        {
            for (bus = usb_get_busses(); bus && (!found); bus = bus->next)
            {
                struct usb_device *dev;

                for (dev = bus->devices; dev && (!found); dev = dev->next)
                {
                    if ((dev->descriptor.idVendor  == VENDOR_ID) &&
                        (dev->descriptor.idProduct == DEVICE_ID))
                    {
                        udev = usb_open(dev);

                        if (udev)
                        {
                            // TODO check USB serial number

                            found = 1;
                        }
                    }
                }
            }
        }
        Sleep(200);
    }

    if(!found)
    {
        printf("no USB device\n");
        return(0);
    }
    return(1);
}

int USBReqISP(usb_dev_handle *udev,
              unsigned int *command,
              unsigned int *result)
{
    int cmdret;
    int resret;

    // default to unknown error
    result[0] = LPC_MAX_ERROR;

    cmdret = usb_control_msg(
        udev,       // dev_handle
        0x00 | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,   // requesttype
        REQ_ISP_COMMAND,    // request
        0,          // value
        0,          // index
        (char*) command,    // *bytes
        20,         // size
        40000);     // timeout

    if (cmdret < 0)
    {
        perror("USB error");
        return(0);
    }

    resret = usb_control_msg(
        udev,       // dev_handle
        0x80 | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,   // requesttype
        REQ_ISP_COMMAND,    // request
        0,          // value
        0,          // index
        (char*) result,    // *bytes
        12+20,      // size
        100);       // timeout
//printf("\ncmdret %d, resret %d cmd %d res %d cmdo %d \n",cmdret, resret, command[0], result[0], result[3]);
//printf("\ncmdret %d, cmd0 %d/%c, cmd1 0x%08X, cmd2 0x%08X, cmd3 0x%08X\n", cmdret, command[0], command[0], command[1], command[2], command[3]);

    if (resret < 0)
    {
        perror("USB error");
        return(0);
    }

    if (result[0] != 0)
    {
        printf("ISP error (%d:%s)\n", result[0]&0xFF, IspError(result[0]&0xFF));
        return(0);
    }
    if ((cmdret != 20) ||
        (resret != 12+20) ||
        (memcmp(command, &result[3], 20)))
    {
        printf("Unknown error\n");
        return(0);
    }

    return(1);
}

int USBReqData(usb_dev_handle *udev, unsigned char* data, int size)
{
    int datret;
    int size1, size2;
    int count;

    /* make small chunks for libusb */
    for (count = 0; count < size; count += USB_MAXCHUNK)
    {
        size1 = ((size-count)/USB_MAXCHUNK) > 0 ? USB_MAXCHUNK : size%USB_MAXCHUNK;
        size2 = size1;

        datret = usb_control_msg(
            udev,               // dev_handle
            0x00 | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,// requesttype
            REQ_DATA_TRANSFER,  // request
            0,                  // value
            0,                  // index
            (char*)(data+count),// *bytes
            size1,              // size
            1000);              // timeout

//        printf("\ndatret %d, size %d\n", datret, size);
//        printf(" %d %d\n", size1, size2);

        if (datret < 0)
        {
            perror("USB error");
            return(0);
        }

        if (size1 != size2)
        {
            printf("write data failed\n");
            return(0);
        }
    }
    return(1);
}

int USBReqBTL(usb_dev_handle *udev,
              unsigned int *command,
              unsigned int *result)
{
    int cmdret;
    int resret;

    // default to unknown error
    result[0] = LPC_MAX_ERROR;

    cmdret = usb_control_msg(
        udev,       // dev_handle
        0x00 | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,   // requesttype
        REQ_BTL_COMMAND,    // request
        0,          // value
        0,          // index
        (char*) command,    // *bytes
        20,         // size
        40000);     // timeout

    if (cmdret < 0)
    {
        perror("USB error");
        return(0);
    }

    resret = usb_control_msg(
        udev,       // dev_handle
        0x80 | USB_TYPE_VENDOR | USB_RECIP_INTERFACE,   // requesttype
        REQ_ISP_COMMAND,    // request
        0,          // value
        0,          // index
        (char*) result,    // *bytes
        12+20,      // size
        100);       // timeout
//printf("\ncmdret %d, resret %d cmd %d res %d cmdo %d \n",cmdret, resret, command[0], result[0], result[3]);
//printf("\ncmdret %d, cmd0 %d/%c, cmd1 0x%08X, cmd2 0x%08X, cmd3 0x%08X\n", cmdret, command[0], command[0], command[1], command[2], command[3]);

    if (resret < 0)
    {
        perror("USB error");
        return(0);
    }

    if (result[0] != 0)
    {
        printf("BTL error (%d:%s)\n", result[0]&0xFF, IspError(result[0]&0xFF));
        return(0);
    }
    if ((cmdret != 20) ||
        (resret != 12+20) ||
        (memcmp(command, &result[3], 20)))
    {
        printf("Unknown error\n");
        return(0);
    }

    return(1);
}

int unlock(usb_dev_handle *udev)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_UNLOCK;
    command[1] = UNLOCK_CODE;

    if (!USBReqISP(udev, command, result))
    {
        printf("unlock failed\n");
        return(0);
    }
    return(1);
}

int readBootCode(usb_dev_handle *udev, unsigned int * bootCode)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_READ_BOOT_CODE_VERSION;

    if (!USBReqISP(udev, command, result))
    {
        printf("read boot code failed\n");
        return(0);
    }

    *bootCode = result[1];

    return(1);
}

int readPartID(usb_dev_handle *udev, unsigned int * partID)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_READ_PART_ID;

    if (!USBReqISP(udev, command, result))
    {
        printf("read part ID failed\n");
        return(0);
    }

    *partID = result[1];

    return(1);
}

int prepareSectors(usb_dev_handle *udev, int startSec, int endSec)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_PREPARE_SECTORS;
    command[1] = startSec;
    command[2] = endSec;

    if (!USBReqISP(udev, command, result))
    {
        printf("prepare failed at sectors %d-%d", startSec, endSec);
        return(0);
    }
    return(1);
}

int eraseSectors(usb_dev_handle *udev, int startSec, int endSec)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_ERASE_SECTORS;
    command[1] = startSec;
    command[2] = endSec;

    if (!USBReqISP(udev, command, result))
    {
        printf("erase failed at sectors %d-%d\n", startSec, endSec);
        return(0);
    }
    return(1);
}

int blankCheckSectors(usb_dev_handle *udev, int startSec, int endSec)
{
    unsigned int command[5];
    unsigned int result[8];

    /* blank check sector 0 always fails */
    if (startSec == 0)
    {
        if (endSec == 0)
        {
            return(1);
        }
        startSec = 1;
    }

    command[0] = ISP_BLANK_CHECK_SECTORS;
    command[1] = startSec;
    command[2] = endSec;

    if (!USBReqISP(udev, command, result))
    {
        printf("blank check failed at sectors %d-%d\n", startSec, endSec);
        return(0);
    }
    return(1);
}

int writeRAM(usb_dev_handle *udev, unsigned int startAdr, unsigned int size)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_WRITE_TO_RAM;
    command[1] = startAdr;
    command[2] = size;

    if (!USBReqISP(udev, command, result))
    {
        printf("write failed at 0x%08X size 0x%08X\n", startAdr, size);
        return(0);
    }
    return(1);
}

int copyRAMFlash(usb_dev_handle *udev, unsigned int startAdr, unsigned int base, unsigned int size)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_COPY_RAM_TO_FLASH;
    command[1] = startAdr;
    command[2] = base;
    command[3] = size;

    if (!USBReqISP(udev, command, result))
    {
        printf("copy RAM to flash failed at 0x%08X base 0x%08X size 0x%08X\n", startAdr, base, size);
        return(0);
    }
    return(1);
}

int compareMem(usb_dev_handle *udev, unsigned int dst, unsigned int src, unsigned int size)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_COMPARE;
    command[1] = dst;
    command[2] = src;
    command[3] = size;

    if (!USBReqISP(udev, command, result))
    {
        printf("compare MEM failed at 0x%08X base 0x%08X size 0x%08X\n", dst, src, size);
        return(0);
    }
    return(1);
}

int goSw(usb_dev_handle *udev, unsigned int startAdr)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = ISP_GO;
    command[1] = startAdr;
    command[2] = ARM_CODE;

    if (!USBReqISP(udev, command, result))
    {
        printf("go failed at 0x%08X\n", startAdr);
        return(0);
    }

    return(1);
}

int readBootloaderVersion(usb_dev_handle *udev, unsigned int * bootVersion)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = BTL_READ_VERSION;

    if (!USBReqBTL(udev, command, result))
    {
        printf("read bootloader version failed\n");
        return(0);
    }

    *bootVersion = result[1];

    return(1);
}

int readBootloaderLocation(usb_dev_handle *udev, unsigned int * bootLocation)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = BTL_READ_LOCATION;

    if (!USBReqBTL(udev, command, result))
    {
        printf("read bootloader location failed\n");
        return(0);
    }

    *bootLocation = result[1];

    return(1);
}

int readRAMAddress(usb_dev_handle *udev, unsigned int * ramAddr)
{
    unsigned int command[5];
    unsigned int result[8];

    command[0] = BTL_READ_RAM_ADDR;

    if (!USBReqBTL(udev, command, result))
    {
        printf("read RAM load address failed\n");
        return(0);
    }

    *ramAddr = result[1];

    return(1);
}
