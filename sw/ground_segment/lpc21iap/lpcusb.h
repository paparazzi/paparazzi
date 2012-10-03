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

int USBFindDevice(usb_dev_handle *udev);
int USBReqISP(usb_dev_handle *udev,
              unsigned int *command,
              unsigned int *result);
int USBReqData(usb_dev_handle *udev, unsigned char* data, int size);
int unlock(usb_dev_handle *udev);
int readBootCode(usb_dev_handle *udev, unsigned int * bootCode);
int readPartID(usb_dev_handle *udev, unsigned int * partID);
int prepareSectors(usb_dev_handle *udev, int startSec, int endSec);
int eraseSectors(usb_dev_handle *udev, int startSec, int endSec);
int blankCheckSectors(usb_dev_handle *udev, int startSec, int endSec);
int writeRAM(usb_dev_handle *udev, unsigned int startAdr, unsigned int size);
int copyRAMFlash(usb_dev_handle *udev, unsigned int startAdr, unsigned int base, unsigned int size);
int compareMem(usb_dev_handle *udev, unsigned int dst, unsigned int src, unsigned int size);
int goSw(usb_dev_handle *udev, unsigned int startAdr);
int readBootloaderVersion(usb_dev_handle *udev, unsigned int * bootVersion);
int readBootloaderLocation(usb_dev_handle *udev, unsigned int * bootLocation);
int readRAMAddress(usb_dev_handle *udev, unsigned int * ramAddr);
