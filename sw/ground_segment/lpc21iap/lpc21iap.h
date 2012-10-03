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

/* Philips proprietary defines */

#define CMD_SUCCESS                 0
#define INVALID_COMMAND             1
#define SRC_ADDR_ERROR              2
#define DST_ADDR_ERROR              3
#define SRC_ADDR_NOT_MAPPED         4
#define DST_ADDR_NOT_MAPPED         5
#define COUNT_ERROR                 6
#define INVALID_SECTOR              7
#define SECTOR_NOT_BLANK            8
#define SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION 9
#define COMPARE_ERROR               10
#define BUSY                        11
#define PARAM_ERROR                 12
#define ADDR_ERROR                  13
#define ADDR_NOT_MAPPED             14
#define CMD_LOCKED                  15
#define INVALID_CODE                16
#define INVALID_BAUD_RATE           17
#define INVALID_STOP_BIT            18
#define CODE_READ_PROTECTION_ENABLED    19

#define ISP_UNLOCK                  'U'
#define ISP_SET_BAUD_RATE           'B'
#define ISP_ECHO                    'A'
#define ISP_WRITE_TO_RAM            'W'
#define ISP_READ_MEMORY             'R'
#define ISP_PREPARE_SECTORS         'P'
#define ISP_COPY_RAM_TO_FLASH       'C'
#define ISP_GO                      'G'
#define ISP_ERASE_SECTORS           'E'
#define ISP_BLANK_CHECK_SECTORS     'I'
#define ISP_READ_PART_ID            'J'
#define ISP_READ_BOOT_CODE_VERSION  'K'
#define ISP_COMPARE                 'M'

#define IAP_PREPARE_SECTORS         50
#define IAP_COPY_RAM_TO_FLASH       51
#define IAP_ERASE_SECTORS           52
#define IAP_BLANK_CHECK_SECTORS     53
#define IAP_READ_PART_ID            54
#define IAP_READ_BOOT_CODE_VERSION  55
#define IAP_COMPARE                 56
#define IAP_REINVOKE_ISP            57

#define ARM_CODE                    'A'
#define THUMB_CODE                  'T'

#define UNLOCK_CODE                 23130
#define CODE_PROTECT_ADDR           0x1FC
#define CODE_PROTECT_DATA           0x87654321
#define IAP_LOCATION                0x7FFFFFF1


/* our own proprietary defines */

#define REQ_ISP_COMMAND             0xF0
#define REQ_IAP_COMMAND             0xF1
#define REQ_BTL_COMMAND             0xF2
#define REQ_DATA_TRANSFER           0xF3

#define BTL_READ_VERSION            10
#define BTL_READ_LOCATION           11
#define BTL_READ_RAM_ADDR           12

#define RUN_ROM                     0
#define RUN_RAM                     1

/* this USB vendor/device ID is fake - someone willing to  donate one? */
#define VENDOR_ID       0x7070
#define DEVICE_ID       0x1234


