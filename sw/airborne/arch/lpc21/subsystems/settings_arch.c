/*
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/lpc21/subsystems/settings_arch.c
 * Persistent settings low level flash routines lpc21.
 *
 * LPC2148 flash data is located in the last available page
 *
 * 0x00000000: Paparazzi bootloader   (16k)
 * 0x00004000: Paparazzi code        (480k)
 * 0x0007C000: persistent settings     (4k)
 * 0x0007D000: Philips/NXP bootloader (12k)
 *
 * data          flash_addr
 * data_size     flash_end - FSIZ
 * checksum      flash_end - FCHK
 *
 * LPC21: minimum write size 256 bytes, endurance 100k cycles,
 *        max sector erase time 400ms, max prog time 1ms per 256 bytes
 */

#include "subsystems/settings.h"

#include "LPC21xx.h"
#include BOARD_CONFIG
#include "armVIC.h"

#define IAP_LOCATION 0x7FFFFFF1

#define IAP_PREPARE_SECTORS     50
#define IAP_COPY_RAM_TO_FLASH   51
#define IAP_ERASE_SECTORS       52
#define IAP_BLANK_CHECK_SECTORS 53
#define IAP_READ_PART_ID        54
#define IAP_COMPARE             56

/* we have to operate on 256 byte flash boundaries */
#define BOUND       256

#define FSIZ        8
#define FCHK        4

typedef void (*IAP)(uint32_t[], uint32_t[]);

typedef struct {
  uint32_t addr;
  uint32_t total_size;
  uint32_t page_nr;
  uint32_t page_size;
} FlashInfo;

static uint32_t pflash_checksum(uint32_t ptr, uint32_t size);
static int32_t flash_detect(FlashInfo *flash);
static int32_t pflash_erase_page(FlashInfo *flash);
static int32_t pflash_program_array(FlashInfo *flash,
                                    uint32_t dest,
                                    uint32_t src);
static int32_t pflash_program_bytes(FlashInfo *flash,
                                    uint32_t src,
                                    uint32_t size,
                                    uint32_t chksum);


static uint32_t pflash_checksum(uint32_t ptr, uint32_t size)
{
  uint32_t i, sum = 0;

  /* do it cheap for now */
  for (i = 0; i < size; i++) {
    sum += *(uint8_t *)(ptr + i);
  }

  return sum;
}

static int32_t flash_detect(FlashInfo *flash)
{
  uint32_t command[5], result[3];
  IAP iap_entry;

  iap_entry = (IAP) IAP_LOCATION;

  /* get part ID */
  command[0] = IAP_READ_PART_ID;
  disableIRQ();
  iap_entry(command, result);
  enableIRQ();
  if (result[0] != 0) { return result[0]; }

  switch (result[1]) {
      /* LPC2141, 32k Flash, 8k RAM */
    case 0x0402FF01:
      /* LPC2142, 64k Flash, 16k RAM */
    case 0x0402FF11:
      /* LPC2144, 128k Flash, 16k RAM */
    case 0x0402FF12:
      /* LPC2146, 256k Flash, 32k+8k RAM */
    case 0x0402FF23: {
      return -1;
      break;
    }
    /* have LPC2148 support only */
    /* LPC2148, 512k Flash, 32k+8k RAM */
    case 0x0402FF25: {
      flash->page_size = 0x1000;
      flash->page_nr = 26;
      flash->addr = 0x7C000;
      break;
    }
    default: return -1;
  }

  return 0;
}

static int32_t pflash_erase_page(FlashInfo *flash)
{
  uint32_t command[5], result[3];
  IAP iap_entry;

  iap_entry = (IAP) IAP_LOCATION;

  /* prepare page/sector */
  command[0] = IAP_PREPARE_SECTORS;
  command[1] = flash->page_nr;
  command[2] = flash->page_nr;
  disableIRQ();
  iap_entry(command, result);
  enableIRQ();
  if (result[0] != 0) { return result[0]; }

  /* erase page/sector */
  command[0] = IAP_ERASE_SECTORS;
  command[1] = flash->page_nr;
  command[2] = flash->page_nr;
  disableIRQ();
  iap_entry(command, result);
  enableIRQ();
  if (result[0] != 0) { return result[0]; }

  /* verify erase */
  command[0] = IAP_BLANK_CHECK_SECTORS;
  command[1] = flash->page_nr;
  command[2] = flash->page_nr;
  iap_entry(command, result);
  if (result[0] != 0) { return result[0]; }

  return 0;
}

static int32_t pflash_program_array(FlashInfo *flash,
                                    uint32_t dest,
                                    uint32_t src)
{
  uint32_t command[5], result[3];
  IAP iap_entry;

  iap_entry = (IAP) IAP_LOCATION;

  /* prepare page/sector */
  command[0] = IAP_PREPARE_SECTORS;
  command[1] = flash->page_nr;
  command[2] = flash->page_nr;
  disableIRQ();
  iap_entry(command, result);
  enableIRQ();
  if (result[0] != 0) { return result[0]; }

  /* flash from ram */
  command[0] = IAP_COPY_RAM_TO_FLASH;
  command[1] = dest;
  command[2] = src;
  command[3] = BOUND;
  command[4] = CCLK / 1000;
  disableIRQ();
  iap_entry(command, result);
  enableIRQ();
  if (result[0] != 0) { return result[0]; }

  return 0;
}

static int32_t pflash_program_bytes(FlashInfo *flash,
                                    uint32_t src,
                                    uint32_t size,
                                    uint32_t chksum)
{
  uint32_t data[BOUND / 4], i, j, ret;
  uint32_t ptr = (uint32_t) &data;

  /* erase */
  if ((ret = pflash_erase_page(flash))) { return ret; }

  /* write data in arrays */
  for (i = 0; i < size; i += BOUND) {
    /* copy data to aligned memory */
    for (j = 0; j < BOUND; j++) {
      *(uint8_t *)(ptr + j) = *(uint8_t *)(src + i + j);
    }
    if (i == flash->page_size - BOUND) {
      data[(BOUND - FSIZ) / 4] = size;
      data[(BOUND - FCHK) / 4] = chksum;
    }
    if ((ret = pflash_program_array(flash, flash->addr + i, ptr))) { return ret; }
  }

  /* last array */
  if (i <= flash->page_size - BOUND) {
    data[(BOUND - FSIZ) / 4] = size;
    data[(BOUND - FCHK) / 4] = chksum;
    if ((ret = pflash_program_array(flash,
                                    flash->addr + flash->page_size - BOUND,
                                    ptr))) {
      return ret;
    }
  }

  /* verify data */
  for (i = 0; i < size; i++) {
    if ((*(uint8_t *)(flash->addr + i)) != (*(uint8_t *)(src + i))) { return -2; }
  }
  if (*(uint32_t *)(flash->addr + flash->page_size - FSIZ) != size) { return -3; }
  if (*(uint32_t *)(flash->addr + flash->page_size - FCHK) != chksum) { return -4; }

  return 0;
}

int32_t persistent_write(void *ptr, uint32_t size)
{
  FlashInfo flash_info;

  if (flash_detect(&flash_info)) { return -1; }
  if ((size > flash_info.page_size - FSIZ) || (size == 0)) { return -2; }

  return pflash_program_bytes(&flash_info,
                              (uint32_t)ptr,
                              size,
                              pflash_checksum((uint32_t)ptr, size));
}

int32_t persistent_read(void *ptr, uint32_t size)
{
  FlashInfo flash;
  uint32_t i;

  /* check parameters */
  if (flash_detect(&flash)) { return -1; }
  if ((size > flash.page_size - FSIZ) || (size == 0)) { return -2; }

  /* check consistency */
  if (size != *(uint32_t *)(flash.addr + flash.page_size - FSIZ)) { return -3; }
  if (pflash_checksum(flash.addr, size) !=
      *(uint32_t *)(flash.addr + flash.page_size - FCHK)) {
    return -4;
  }

  /* copy data */
  for (i = 0; i < size; i++) {
    *(uint8_t *)((uint32_t)ptr + i) = *(uint8_t *)(flash.addr + i);
  }

  return 0;
}
