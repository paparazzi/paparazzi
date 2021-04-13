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
 * @file arch/stm32/subsystems/settings_arch.c
 * Persistent settings low level flash routines stm32.
 *
 * data          flash_addr
 * data_size     flash_end - FSIZ
 * checksum      flash_end - FCHK
 *
 * STM32: minimum write size 2 bytes, endurance 10k cycles,
 *        max sector erase time 40ms, max prog time 70us per 2 bytes
 */

#include "subsystems/settings.h"

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/dbgmcu.h>

struct FlashInfo {
  uint32_t addr;
  uint32_t total_size;
  uint32_t page_nr;
  uint32_t page_size;
};


static uint32_t pflash_checksum(uint32_t ptr, uint32_t size);
static int32_t flash_detect(struct FlashInfo *flash);
static int32_t pflash_program_bytes(struct FlashInfo *flash,
                                    uint32_t src,
                                    uint32_t size,
                                    uint32_t chksum);

#if defined(STM32F1)
#define FLASH_SIZE_ MMIO16(0x1FFFF7E0)
#elif defined(STM32F4)
#define FLASH_SIZE_ MMIO16(0x1FFF7A22)
#endif

#define FLASH_BEGIN 0x08000000
#define FSIZ        8
#define FCHK        4


static uint32_t pflash_checksum(uint32_t ptr, uint32_t size)
{
  uint32_t i;

  /* reset crc */
  CRC_CR = CRC_CR_RESET;

  if (ptr % 4) {
    /* calc in 8bit chunks */
    for (i = 0; i < (size & ~3); i += 4) {
      CRC_DR = (*(uint8_t *)(ptr + i)) |
               (*(uint8_t *)(ptr + i + 1)) << 8 |
               (*(uint8_t *)(ptr + i + 2)) << 16 |
               (*(uint8_t *)(ptr + i + 3)) << 24;
    }
  } else {
    /* calc in 32bit */
    for (i = 0; i < (size & ~3); i += 4) {
      CRC_DR = *(uint32_t *)(ptr + i);
    }
  }

  /* remaining bytes */
  switch (size % 4) {
    case 1:
      CRC_DR = *(uint8_t *)(ptr + i);
      break;
    case 2:
      CRC_DR = (*(uint8_t *)(ptr + i)) |
               (*(uint8_t *)(ptr + i + 1)) << 8;
      break;
    case 3:
      CRC_DR = (*(uint8_t *)(ptr + i)) |
               (*(uint8_t *)(ptr + i + 1)) << 8 |
               (*(uint8_t *)(ptr + i + 2)) << 16;
      break;
    default:
      break;
  }

  return CRC_DR;
}

static int32_t flash_detect(struct FlashInfo *flash)
{

  flash->total_size = FLASH_SIZE_ * 0x400;

#if defined(STM32F1)
  /* FIXME This will not work for connectivity line (needs ID, see below), but
           device ID is only readable when freshly loaded through JTAG?! */

  /* WARNING If you are using this for F4 this only works for memory sizes
   * larger than 128kb. Otherwise the first few sectors are either 16kb or
   * 64kb. To make those small devices work we would need to know what the page
   * we want to put the settings into is. Otherwise we will might be writing
   * into a 64kb page that is actually 16kb big.
   */
  switch (flash->total_size) {
      /* low density */
    case 0x00004000: /* 16 kBytes */
    case 0x00008000: /* 32 kBytes */
      /* medium density, e.g. STM32F103RBT6 (Olimex STM32-H103) */
    case 0x00010000: /* 64 kBytes */
    case 0x00020000: { /* 128 kBytes */
      flash->page_size = 0x400;
      break;
    }
    /* high density, e.g. STM32F103RE (Joby Lisa/M, Lisa/L) */
    case 0x00040000: /* 256 kBytes */
    case 0x00080000: /* 512 kBytes */
      /* XL density */
    case 0x000C0000: /* 768 kBytes */
    case 0x00100000: { /* 1 MByte */
      flash->page_size = 0x800;
      break;
    }
    default: {return -1;}
  }

#elif defined(STM32F4) /* this is the correct way of detecting page sizes but we currently only use it for the F4 because the F1 version is broken. */
  uint32_t device_id;

  /* read device id */
  device_id = DBGMCU_IDCODE & DBGMCU_IDCODE_DEV_ID_MASK;

  switch (device_id) {
      /* low density */
    case 0x412:
      /* medium density, e.g. STM32F103RB (Olimex STM32-H103) */
    case 0x410: {
      flash->page_size = 0x400;
      break;
    }
    /* high density, e.g. STM32F103RE (Joby Lisa/L) */
    case 0x414:
      /* XL density */
    case 0x430:
      /* connectivity line */
    case 0x418: {
      flash->page_size = 0x800;
      break;
    }
    case 0x0413: /* STM32F405xx/07xx and STM32F415xx/17xx) */
    case 0x0419: /* STM32F42xxx and STM32F43xxx */
    case 0x0423: /* STM32F401xB/C */
    case 0x0433: /* STM32F401xD/E */
    case 0x0431: { /* STM32F411xC/E */
      flash->page_size = 0x20000;
      break;
    }
    default: return -1;
  }

  switch (flash->total_size) {
    case 0x00004000: /* 16 kBytes */
    case 0x00008000: /* 32 kBytes */
    case 0x00010000: /* 64 kBytes */
    case 0x00020000: /* 128 kBytes */
    case 0x00040000: /* 256 kBytes */
    case 0x00080000: /* 512 kBytes */
    case 0x000C0000: /* 768 kBytes */
    case 0x00100000: /* 1 MByte */
    case 0x00200000: /* 2 MByte */
      break;
    default: return -1;
  }
#else
#error Unknown device
#endif

#if defined(STM32F1)
  flash->page_nr = (flash->total_size / flash->page_size) - 1;
  flash->addr = FLASH_BEGIN + flash->page_nr * flash->page_size;
#elif defined(STM32F4)
  /* We are assuming all pages are 128kb so we have to compensate for the first
   * few pages that are smaller which means we have to skip the first 4.
   */
  flash->page_nr = (flash->total_size / flash->page_size) - 1 + 4;
  flash->addr = FLASH_BEGIN + (flash->page_nr - 4) * flash->page_size;
#endif

  return 0;
}


static int32_t pflash_erase(struct FlashInfo *flash)
{
  uint32_t i;

  /* erase */
  flash_unlock();
#if defined(STM32F1)
  flash_erase_page(flash->addr);
#elif defined(STM32F4)
  flash_erase_sector(flash->page_nr, FLASH_CR_PROGRAM_X32);
#endif
  flash_lock();

  /* verify erase */
  for (i = 0; i < flash->page_size; i += 4) {
    if ((*(uint32_t *)(flash->addr + i)) != 0xFFFFFFFF) { return -1; }
  }
  return 0;
}

// (gdb) p *flash
// $1 = {addr = 134739968, total_size = 524288, page_nr = 255, page_size = 2048}
//              0x807F800             0x80000
static int32_t pflash_program_bytes(struct FlashInfo *flash,
                                    uint32_t   src,
                                    uint32_t   size,
                                    uint32_t   chksum)
{
  uint32_t i;

  /* erase, return with error if not successful */
  if (pflash_erase(flash)) { return -1; }

  flash_unlock();
  /* write full 16 bit words */
  for (i = 0; i < (size & ~1); i += 2) {
    flash_program_half_word(flash->addr + i,
                            (uint16_t)(*(uint8_t *)(src + i) | (*(uint8_t *)(src + i + 1)) << 8));
  }
  /* fill bytes with a zero */
  if (size & 1) {
    flash_program_half_word(flash->addr + i, (uint16_t)(*(uint8_t *)(src + i)));
  }
  /* write size */
  flash_program_half_word(flash->addr + flash->page_size - FSIZ,
                          (uint16_t)(size & 0xFFFF));
  flash_program_half_word(flash->addr + flash->page_size - FSIZ + 2,
                          (uint16_t)((size >> 16) & 0xFFFF));
  /* write checksum */
  flash_program_half_word(flash->addr + flash->page_size - FCHK,
                          (uint16_t)(chksum & 0xFFFF));
  flash_program_half_word(flash->addr + flash->page_size - FCHK + 2,
                          (uint16_t)((chksum >> 16) & 0xFFFF));
  flash_lock();

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
  struct FlashInfo flash_info;
  if (flash_detect(&flash_info)) { return -1; }
  if ((size > flash_info.page_size - FSIZ) || (size == 0)) { return -2; }

  return pflash_program_bytes(&flash_info,
                              (uint32_t)ptr,
                              size,
                              pflash_checksum((uint32_t)ptr, size));
}

int32_t persistent_read(void *ptr, uint32_t size)
{
  struct FlashInfo flash;
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

int32_t persistent_clear(void)
{
  struct FlashInfo flash_info;
  if (flash_detect(&flash_info)) { return -1; }

  return pflash_erase(&flash_info);
}
