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


#define LPC21IAP_VER_MAJ    1
#define LPC21IAP_VER_MIN    4


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
#endif // defined COMPILE_FOR_LINUX || defined COMPILE_FOR_CYGWIN

#include <ctype.h>      // isdigit()
#include <stdio.h>      // stdout
#include <stdarg.h>
#include <fcntl.h>
#include <usb.h>        // libusb

#include "elf.h"
#include "lpcusb.h"
#include "lpc21iap.h"

#ifndef O_BINARY
#define O_BINARY 0
#endif // O_BINARY

/* LPC214x maximum sector size */
#define MAX_SECT        0x1000
/* number of sectors used for bootloader */
#define BOOTLOAD_SECT   4

typedef struct
{
    unsigned int id;
    char name[50];
} LPC_DEVICE;

typedef struct
{
    unsigned int start;
    void * next;
    unsigned char data[MAX_SECT];
} tIntermediateBuffer;

static int SectorTable_214x[] = { 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
                                  32768, 32768, 32768, 32768, 32768, 32768, 32768, 32768,
                                  32768, 32768, 32768, 32768, 32768, 32768,
                                  4096, 4096, 4096, 4096, 4096 };

static LPC_DEVICE LPCtypes[] =
{
  { 0x0402FF01, "LPC2141, 32k Flash, 8k RAM" },
  { 0x0402FF11, "LPC2142, 64k Flash, 16k RAM" },
  { 0x0402FF12, "LPC2144, 128k Flash, 16k RAM" },
  { 0x0402FF23, "LPC2146, 256k Flash, 32k+8k RAM" },
  { 0x0402FF25, "LPC2148, 512k Flash, 32k+8k RAM" },
  { 0, "unknown" },
};


#if defined COMPILE_FOR_LINUX
void Sleep(unsigned long msec)
{
    usleep(msec*1000); //convert to microseconds
}
#endif // defined COMPILE_FOR_LINUX

/* gives the sector number in which the given address is in */
int getSec(unsigned int adrFlash)
{
    int count;
    int temp = 0;

    for (count=0;
         count < sizeof(SectorTable_214x)/sizeof(SectorTable_214x[0]);
         count++)
    {
        temp += SectorTable_214x[count];
        if (temp > adrFlash)
        {
            return(count);
        }
    }
    return(-1);
}

int main(int argc, char *argv[])
{
    int fdElf;
    int lenElf;
    unsigned char * binElf;
    unsigned int entryElf;
    unsigned int flagsElf;
    unsigned int ramAddr;
    unsigned int romram;
    int secElf;
    int count;
    int temp;
    unsigned int utemp;
    int startSec, endSec;
    unsigned int start, src, size, end, type, flag;
    unsigned int maxFlash, lowFlash, highFlash;
    usb_dev_handle *udev;
    struct usb_device *dev;
    int found;
    struct usb_bus *bus;
    char cserial[256];
    tIntermediateBuffer* actBuf = NULL;
    tIntermediateBuffer* startBuf = NULL;

    printf("lpc21iap v%d.%d\n", LPC21IAP_VER_MAJ, LPC21IAP_VER_MIN);

    if ((argc < 2) || (argc > 3))
    {
        printf("usage: %s file.elf [usb_serial_number]\n", argv[0]);
        exit(1);
    }

    fdElf = open(argv[1], O_RDONLY | O_BINARY);

    if(fdElf < 0)
    {
        perror("open file");
        exit(1);
    }

    lenElf = lseek(fdElf, 0, SEEK_END);

    if(lenElf <= 0)
    {
        perror("file length");
        exit(1);
    }

    lseek(fdElf, 0, SEEK_SET);

    binElf = malloc(lenElf);

    if (binElf == NULL)
    {
        perror("malloc failed");
        exit(1);
    }

    startBuf = malloc(sizeof(tIntermediateBuffer));
    if (startBuf == NULL)
    {
        perror("malloc failed");
        exit(1);
    }
    startBuf->start = 0xFFFFFFFF;
    startBuf->next = NULL;

    memset(startBuf->data, 0xFF, MAX_SECT);

    count = 0;
    while (count < lenElf)
    {
        temp = read(fdElf, binElf+count, lenElf-count);
        if (temp <= 0)
        {
            perror("could not read");
            exit(1);
        }
        count += temp;
    }

    close(fdElf);

    /* check ELF header */
    if (!ELFCheckHeader(binElf))
    {
        perror("no ELF header" );
        exit(1);
    }

    flagsElf = ELFHdrFlags(binElf);
    if (EF_ARM_HASENTRY == (flagsElf & EF_ARM_HASENTRY))
    {
        entryElf = ELFEntryAddr(binElf);
    }

    secElf = ELFNoPSections(binElf);

    found = 0;
    usb_init();
    usb_find_busses();

    for(count = 0; count < 100 && !found; count++)
    {
        printf(".");
        fflush(stdout);

        if (usb_find_devices())
        {
            for (bus = usb_get_busses(); bus && (!found); bus = bus->next)
            {
                for (dev = bus->devices; dev && (!found); dev = dev->next)
                {
                    if ((dev->descriptor.idVendor  == VENDOR_ID) &&
                        (dev->descriptor.idProduct == DEVICE_ID))
                    {
                        udev = usb_open(dev);

                        if (udev)
                        {
                            printf("\nFound USB device\n");
                            if (argc==3)
                            {
                                if (dev->descriptor.iSerialNumber)
                                {
                                    if (usb_get_string_simple(udev, dev->descriptor.iSerialNumber, cserial, sizeof(cserial)) > 0)
                                    {
                                        if (strncmp(argv[2], cserial, strlen(argv[2])) == 0)
                                        {
                                            found = 1;
                                        }
                                        else
                                        {
                                            printf("Serial Number does not match: %s\n", cserial);
                                        }
                                    }
                                    else
                                    {
                                        printf("could not get serial number\n");
                                    }
                                }
                                else
                                {
                                    printf("no serial number\n");
                                }

                                if (!found) usb_close(udev);
                            }
                            else found = 1;
                        }
                    }
                }
            }
        }
        Sleep(200);
    }

    if(!found)
    {
        printf("\nno USB device\n");
        exit(1);
    }


    if (!unlock(udev)) exit(1);

    if (!readBootCode(udev, &utemp)) exit(1);
    printf("BootROM code: %d.%d\n", (utemp >> 8) & 0xFF, utemp & 0xFF);

    if (!readPartID(udev, &utemp)) exit(1);
    for (count = 0; LPCtypes[count].id != 0; count++)
    {
        if (utemp == LPCtypes[count].id) break;
    }
    printf("Part ID: 0x%08X (%s)\n", utemp, LPCtypes[count].name);

    if (!readBootloaderVersion(udev, &utemp)) exit(1);
    printf("BootLoader version: %d.%d\n", (utemp >> 8) & 0xFF, utemp & 0xFF);

    if (!readRAMAddress(udev, &ramAddr)) exit(1);

    if (!readBootloaderLocation(udev, &romram)) exit(1);

    /* find used sectors (kill the unused in-betweens, speed optimized) */

    /* calc maximum usable flash address */
    maxFlash = 0;
    for (count=0;
         count < sizeof(SectorTable_214x)/sizeof(SectorTable_214x[0]);
         count++)
    {
            maxFlash += SectorTable_214x[count];
    }

    /* find min and max used flash address */
    lowFlash = maxFlash;
    highFlash = 0;
    for (count=0; count < secElf; count++)
    {
        start = ELFAddrPSection(binElf, count);
        src   = ELFOffsPSection(binElf, count);
        size  = ELFSizePSection(binElf, count);
        type  = ELFTypePSection(binElf, count);
        flag  = ELFFlagPSection(binElf, count);

        if ((size > 0) &&
            (src != 0) &&
            (type == PT_LOAD) &&
            (flag & (PF_X | PF_W | PF_R)) &&
            (start+size <= maxFlash))
        {
            if (start+size > highFlash) highFlash = start+size;
            if (start < lowFlash) lowFlash = start;
        }
    }

    if ((romram == RUN_ROM) && (lowFlash < BOOTLOAD_SECT*MAX_SECT))
    {
        printf("running from ROM, can not flash low sectors, load RAM version first\n");
        exit(2);
    }

    /* flashing code: persistent settings needs highest sector to be erased */
    if (highFlash >= BOOTLOAD_SECT*MAX_SECT)
    {
      highFlash = maxFlash-1;
    }

    /* anything to flash erase? */
    if (lowFlash != maxFlash)
    {
        startSec = getSec(lowFlash);
        endSec = getSec(highFlash);

        /* prepare sectors for erase */
        if (!prepareSectors(udev, startSec, endSec)) exit(1);

        /* erase sectors */
        if (!eraseSectors(udev, startSec, endSec)) exit(1);
    }

    for (count=0; count < secElf; count++)
    {
        start = ELFAddrPSection(binElf, count);
        src   = ELFOffsPSection(binElf, count);
        size  = ELFSizePSection(binElf, count);
        type  = ELFTypePSection(binElf, count);
        flag  = ELFFlagPSection(binElf, count);

        /* adjust size to 32 bit alignment */
        size = (size + 3) & 0xFFFFFFFC;

        if ((size > 0) &&
            (src != 0) &&
            (type == PT_LOAD) &&
            (flag & (PF_X | PF_W | PF_R)))
        {

            if (start+size <= maxFlash)
            {
                unsigned int cnt;
                unsigned int startPage, endPage, countPage;
                unsigned int splitGap, splitCnt;

#if 0
                /* Flash */
                printf( "Flash sect  = %02d, start = 0x%08X, src = 0x%08X, size = 0x%08X, type = 0x%08X, flag = 0x%08X\n",
                count,
                start,
                src,
                size,
                type,
                flag);
#endif

                /* end address in flash */
                end = start + size;

                cnt = 0;

                /* page address of the first sector */
                startPage = start & ~(MAX_SECT-1);
                /* page address of the end sector */
                endPage  = (end+MAX_SECT) & ~(MAX_SECT-1);

                /* always 4k "blocks" are written */
                for (countPage = startPage; countPage < endPage; countPage += MAX_SECT)
                {
                    /* do checksum for first block (LE machines) */
                    if (countPage == 0)
                    {
                        unsigned int * dat = (unsigned int *) (binElf+src);
                        unsigned int crc = 0;
                        int count;

                        dat[5] = 0;

                        for(count = 0; count < 8; count++) {
                            crc += dat[count];
                        }

                        dat[5] = (unsigned long) -crc;

                        printf("changing vector table\n");
                    }

                    printf("#");
                    fflush(stdout);

                    splitCnt = MAX_SECT;

                    /* The LPC214x does not allow pages (4k) to be
                       programmed twice without erasing in between, not even
                       with 0xFF's being filled into the gaps. So we have to
                       store all the partly filled pages until the end and
                       flash them in one flush.                              */

                    /* first page, not beginning at page start */
                    if ((countPage == startPage) && (start != startPage))
                    {
                        /* gap of non programmed bytes at the beginning of page */
                        splitGap = start % MAX_SECT;
                        /* number of bytes in the first block */
                        if ((splitGap + size) < MAX_SECT) {
                            /* data ends within first page */
                            splitCnt = size;
                        }
                        else {
                            /* data until end of page */
                            splitCnt = MAX_SECT - splitGap;
                        }
                    }
                    /* last page, not ending at page end */
                    else  if ((countPage+MAX_SECT == endPage) && (end != endPage))
                    {
                        /* no gap at beginning */
                        splitGap = 0;
                        /* number of bytes in the last sector */
                        splitCnt = end % MAX_SECT;
                    }
                    if ((splitCnt != MAX_SECT) && (splitCnt > 0))
                    {
                        /* keep partly filled pages for later programming */
                        actBuf = startBuf;
                        while (1)
                        {
                            if (actBuf->start == countPage)
                            {
                                break;
                            }
                            if (actBuf->next == NULL)
                            {
                                actBuf->next = malloc(sizeof(tIntermediateBuffer));
                                if (actBuf->next == NULL)
                                {
                                    perror("malloc failed");
                                    exit(1);
                                }
                                actBuf = actBuf->next;
                                actBuf->start = countPage;
                                actBuf->next = NULL;
                                memset(actBuf->data, 0xFF, MAX_SECT);
                                break;
                            }
                            actBuf = actBuf->next;
                        }

                        /* copy data to buffer */
                        // read(startBuf->data + splitGap, splitCnt):
                        memcpy(actBuf->data + splitGap, binElf+src+cnt, splitCnt);
                    }
                    else
                    {
                       /* flash full filled 4k block */

                        /* copy data to buffer */
                        // read(startBuf->data, MAX_SECT):
                        memcpy(startBuf->data, binElf+src+cnt, MAX_SECT);

                        if (!writeRAM(udev, ramAddr, MAX_SECT)) exit(1);
                        if (!USBReqData(udev, startBuf->data, MAX_SECT)) exit(1);
                        if (!prepareSectors(udev, getSec(countPage), getSec(countPage))) exit(1);
                        if (!copyRAMFlash(udev, countPage, ramAddr, MAX_SECT)) exit(1);
                        if (!compareMem(udev, countPage, ramAddr, MAX_SECT)) exit(1);
                    }
                    cnt += splitCnt;
                }
            }
            else
            {
#if 0
                /* RAM */
                printf( "RAM sect  = %02d, start = 0x%08X, src = 0x%08X, size = 0x%08X, type = 0x%08X, flag = 0x%08X\n",
                count,
                start,
                src,
                size,
                type,
                flag);
#endif

                /* copy it in one piece */
                if (!writeRAM(udev, start, size)) exit(1);
                if (!USBReqData(udev, (unsigned char*) (binElf+src), size)) exit(1);
            }
        }
    }

    actBuf = startBuf;

    /* go through all the split sectors */
    while (actBuf != NULL)
    {
        if (actBuf->start != 0xFFFFFFFF)
        {
            if (!writeRAM(udev, ramAddr, MAX_SECT)) exit(1);
            if (!USBReqData(udev, actBuf->data, MAX_SECT)) exit(1);
            if (!prepareSectors(udev, getSec(actBuf->start), getSec(actBuf->start))) exit(1);
            if (!copyRAMFlash(udev, actBuf->start & ~(MAX_SECT-1), ramAddr, MAX_SECT)) exit(1);
            if (!compareMem(udev, actBuf->start & ~(MAX_SECT-1), ramAddr, MAX_SECT)) exit(1);
        }
        startBuf = actBuf;
        actBuf = actBuf->next;
        free(startBuf);
    }

    printf("\n");

    free(binElf);

    if (EF_ARM_HASENTRY == (flagsElf & EF_ARM_HASENTRY))
    {
        printf( "Starting software at 0x%08X\n", entryElf );
        if (!goSw(udev, entryElf)) exit(1);
        Sleep(250);
    }

    return(0);
}
