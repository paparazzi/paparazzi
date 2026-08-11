/*
 * Copyright (C) 2026 OpenUAS
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file arch/chibios/modules/core/settings_arch.c
 * Flight safe persistent settings flash routines for ChibiOS,
 * supporting the STM32F1, STM32F4, STM32F7 and STM32H7 families.
 *
 * The settings live in the last flash sector as an append only
 * journal of self contained records:
 *
 *   sector: | record | record | ... | erased space ...             |
 *   record: | magic, data size | data (padded) | crc32 (padded)   |
 *
 * with all record fields aligned to the family programming unit (at
 * least 4 bytes). Reading returns the newest record with a valid
 * checksum; storing appends a new record into erased space. The flash
 * registers are accessed directly (CMSIS definitions provided through
 * the ChibiOS HAL headers), so no HAL/EFL driver configuration is
 * required.
 *
 * Why it is safe to store settings in flight:
 *
 * - No sector erase is ever performed in flight. Appending makes an
 *   erase (which stalls the CPU on flash fetches for up to ~2 seconds)
 *   unnecessary until the sector is full: then persistent_write()
 *   returns PFLASH_ERR_FULL in flight, and reclaims the sector with an
 *   erase only when the autopilot reports motors off and not in flight
 *   (persistent_clear() is refused in flight the same way).
 *
 * - Programming a single unit stalls flash fetches only for tens of
 *   microseconds, and even that is hidden: the programming routines
 *   execute from RAM (ChibiOS .ramtext) with interrupts masked
 *   (chSysLock) for exactly that window, so neither the CPU nor a
 *   preempting ISR can fetch from flash while the flash bus is busy.
 *   Between units, interrupts are served normally.
 *
 * - All operations are serialized by a mutex (thread safe), and every
 *   busy poll is bounded, so a faulty flash controller degrades to an
 *   error return, never a hang.
 *
 * - A power cut during a store can at worst produce a torn record,
 *   which the checksum rejects on the next boot: the previous valid
 *   record (or the airframe defaults) are used instead. A sector
 *   holding torn data beyond the valid record chain is never appended
 *   to again (a partially programmed flash word must not be programmed
 *   twice); it is reclaimed by the next on ground store or clear.
 *   Erases and programs are verified by reading back.
 */

#include "modules/core/settings.h"
#include "autopilot.h"

#include <ch.h>
#include <hal.h>
#include <string.h>

#if defined(STM32F1XX) || defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)

/** Base address of the internal flash on all supported STM32 families. */
#define PFLASH_BASE 0x08000000UL

/* ST flash unlock key sequence, identical on all supported families
 * (defined here only if the CMSIS device header does not provide it) */
#ifndef FLASH_UNLOCK_KEY1
#define FLASH_UNLOCK_KEY1 0x45670123UL
#endif
#ifndef FLASH_UNLOCK_KEY2
#define FLASH_UNLOCK_KEY2 0xCDEF89ABUL
#endif

/**
 * Return codes of the persistent_write/read/clear API.
 * All errors are negative, so callers may simply test for != 0.
 */
enum pflash_status {
  PFLASH_OK           = 0,   ///< success
  PFLASH_ERR_SECTOR   = -1,  ///< no usable settings sector (geometry or overlap)
  PFLASH_ERR_SIZE     = -2,  ///< requested size is zero or does not fit a sector
  PFLASH_ERR_NODATA   = -3,  ///< no valid stored settings (or stored size mismatch)
  PFLASH_ERR_VERIFY   = -4,  ///< erase/program failed or read back mismatch
  PFLASH_ERR_FULL     = -5,  ///< journal full, erase deferred until on the ground
  PFLASH_ERR_INFLIGHT = -6,  ///< sector erase refused while in flight
};

/**
 * Upper bound for busy polling an erase, far above the worst case
 * erase time. It only turns a (theoretical) stuck flash controller
 * into an error return instead of an infinite loop in the autopilot.
 */
#define PFLASH_WAIT_MAX_POLLS 400000000UL

/**
 * Upper bound for busy polling one unit program operation, which runs
 * with interrupts masked: generous for the worst case programming time
 * (tens of microseconds), while still bounding the interrupt blackout
 * to milliseconds should the flash controller ever get stuck.
 */
#define PFLASH_PROG_MAX_POLLS 2000000UL

/**
 * Place a function in RAM (ChibiOS .ramtext, copied to RAM together
 * with .data by crt0) so it executes without a single flash fetch, and
 * forbid inlining it back into a flash resident caller.
 */
#define PFLASH_RAMFUNC __attribute__((noinline, section(".ramtext")))

/** Serializes all persistent settings operations (thread safety). */
static MUTEX_DECL(pflash_mutex);

/** Location and geometry of the settings sector, filled by pflash_detect(). */
struct FlashInfo {
  uint32_t addr;      ///< address of the settings sector
  uint32_t page_size; ///< size of the settings sector
  uint32_t snb;       ///< sector number encoding for the SNB register field (unused on F1)
};

/*
 * Family specific primitives:
 *
 * pflash_detect       locate the last flash sector
 * pflash_erase        erase (only) the settings sector, ground only
 * pflash_ram_program  program one unit, executes from RAM (.ramtext)
 * pflash_program_unit flight safe wrapper around pflash_ram_program:
 *                     interrupts masked only for the microseconds the
 *                     flash bus is actually busy
 * pflash_cache_flush  make cached flash reads coherent again
 */

#if defined(STM32F1XX)

/** F1 flash programming unit: one 16bit half word. */
#define PFLASH_PROG_SIZE 2

/** All F1 FPEC error flags: programming error, write protection error. */
#define PFLASH_F1_SR_ERRORS (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)

/**
 * Detect the flash geometry and locate the last page.
 *
 * Low/medium density parts (<= 128k) have 1k pages, high density and
 * connectivity line parts have 2k pages. XL density parts (> 512k)
 * are not supported, their upper flash bank has a second FPEC with
 * its own registers.
 *
 * @return 0 on success
 */
static int32_t pflash_detect(struct FlashInfo *flash)
{
  /* flash size in kBytes, from the factory programmed size register */
  uint32_t size_kb = *(volatile const uint16_t *)FLASHSIZE_BASE;
  if ((size_kb < 16) || (size_kb > 512)) { return -1; }

  flash->page_size = (size_kb <= 128) ? 0x400 : 0x800;
  flash->addr = PFLASH_BASE + size_kb * 1024 - flash->page_size;
  flash->snb = 0; /* not used, F1 erases by page address */

  return 0;
}

/**
 * Busy-wait until the flash controller is idle.
 * @return 0 when idle, -1 if the poll limit was exceeded
 */
static int32_t pflash_wait(void)
{
  for (uint32_t i = 0; i < PFLASH_WAIT_MAX_POLLS; i++) {
    if (!(FLASH->SR & FLASH_SR_BSY)) { return 0; }
  }
  return -1;
}

/** Unlock the flash control register (no-op when already unlocked). */
static void pflash_unlock(void)
{
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_UNLOCK_KEY1;
    FLASH->KEYR = FLASH_UNLOCK_KEY2;
  }
}

/** Re-lock the flash control register against accidental writes. */
static void pflash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * Erase the settings page via the FPEC page erase sequence.
 * The CPU stalls on any flash fetch while the erase is in progress.
 * @param flash settings sector location, from pflash_detect()
 * @return 0 on success
 */
static int32_t pflash_erase(const struct FlashInfo *flash)
{
  pflash_unlock();
  if (pflash_wait()) { pflash_lock(); return -1; }
  FLASH->SR = PFLASH_F1_SR_ERRORS | FLASH_SR_EOP; /* clear pending flags */

  /* page erase */
  FLASH->CR = FLASH_CR_PER;
  FLASH->AR = flash->addr;
  FLASH->CR |= FLASH_CR_STRT;
  int32_t ret = pflash_wait();
  FLASH->CR &= ~FLASH_CR_PER;
  pflash_lock();

  if (ret || (FLASH->SR & PFLASH_F1_SR_ERRORS)) { return -1; }
  return 0;
}

/**
 * Program one 16bit half word, executed entirely from RAM (.ramtext):
 * while the FPEC is busy the CPU must not fetch a single instruction
 * from flash or it would stall. Called with interrupts masked, all
 * polls bounded, keep it minimal and free of calls into flash code.
 * @return 0 on success
 */
PFLASH_RAMFUNC static int32_t pflash_ram_program(uint32_t addr, uint16_t val)
{
  uint32_t i;

  for (i = 0; (i < PFLASH_PROG_MAX_POLLS) && (FLASH->SR & FLASH_SR_BSY); i++) { }
  if (FLASH->SR & FLASH_SR_BSY) { return -1; }

  FLASH->CR = FLASH_CR_PG;
  *(volatile uint16_t *)addr = val;
  __DSB();
  for (i = 0; (i < PFLASH_PROG_MAX_POLLS) && (FLASH->SR & FLASH_SR_BSY); i++) { }

  uint32_t sr = FLASH->SR;
  FLASH->CR &= ~FLASH_CR_PG;

  return ((sr & FLASH_SR_BSY) || (sr & PFLASH_F1_SR_ERRORS)) ? -1 : 0;
}

/**
 * Program one 16bit half word (flash must be unlocked by the caller).
 *
 * Flight safe: the controller is confirmed idle first, then interrupts
 * are masked only for the microseconds of the actual programming,
 * executed from RAM so nothing can touch the stalled flash bus.
 *
 * @param addr destination address in erased flash, half word aligned
 * @param buf  source of PFLASH_PROG_SIZE bytes
 * @return 0 on success
 */
static int32_t pflash_program_unit(const struct FlashInfo *flash __attribute__((unused)),
                                   uint32_t addr, const uint8_t *buf)
{
  uint16_t half = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));

  if (pflash_wait()) { return -1; }              /* enter the critical zone idle */
  FLASH->SR = PFLASH_F1_SR_ERRORS | FLASH_SR_EOP; /* clear stale flags */
  chSysLock();
  int32_t ret = pflash_ram_program(addr, half);
  chSysUnlock();
  return ret;
}

#elif defined(STM32F4XX) || defined(STM32F7XX)

/** F4/F7 flash programming unit: one 32bit word. */
#define PFLASH_PROG_SIZE 4

/* optional family specific error flags, absent ones contribute 0 */
#ifdef FLASH_SR_PGSERR /* F4 */
#define PFLASH_SR_ERR_PGS FLASH_SR_PGSERR
#else
#define PFLASH_SR_ERR_PGS 0
#endif
#ifdef FLASH_SR_ERSERR /* F7 */
#define PFLASH_SR_ERR_ERS FLASH_SR_ERSERR
#else
#define PFLASH_SR_ERR_ERS 0
#endif
#ifdef FLASH_SR_RDERR  /* F42x/F43x, F7 */
#define PFLASH_SR_ERR_RD FLASH_SR_RDERR
#else
#define PFLASH_SR_ERR_RD 0
#endif

/**
 * All F4/F7 error flags. Deliberately a macro, not a static const: a
 * flash resident constant could not be read by the RAM programming
 * routine while the flash bus is stalled.
 */
#define PFLASH_SR_ERRORS (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | \
                          PFLASH_SR_ERR_PGS | PFLASH_SR_ERR_ERS | PFLASH_SR_ERR_RD)

/**
 * Detect the flash geometry and locate the last sector.
 *
 * Each STM32F4 flash bank: 4x 16k + 1x 64k + n x 128k sectors.
 * STM32F7 single bank devices: 512k parts (F72x/F73x) use the F4
 * sector sizes, larger parts (F74x..F77x) use doubled sector sizes
 * (4x 32k + 1x 128k + n x 256k). 2MB F76x/F77x parts can optionally
 * (nDBANK option cleared) run dual bank with the F4 sector sizes.
 *
 * @return 0 on success
 */
static int32_t pflash_detect(struct FlashInfo *flash)
{
  /* flash size in kBytes, from the factory programmed size register */
  uint32_t size_kb = *(volatile const uint16_t *)FLASHSIZE_BASE;
  if ((size_kb < 256) || (size_kb > 2048)) { return -1; }
  uint32_t total = size_kb * 1024;

#if defined(STM32F4XX)
  const uint32_t small = 0x20000; /* 128k sectors beyond the first 128k of a bank */
#else /* STM32F7XX */
  const uint32_t small = (size_kb > 512) ? 0x40000 : 0x20000;
#endif

  /* dual bank organization:
   * F4: always on 2MB parts, optional (DB1M set) on 1MB F42x/F43x parts
   * F7: optional (nDBANK cleared) on 2MB F76x/F77x parts */
  bool dual_bank = false;
#if defined(STM32F4XX)
  dual_bank = (size_kb == 2048);
#ifdef FLASH_OPTCR_DB1M
  if (FLASH->OPTCR & FLASH_OPTCR_DB1M) { dual_bank = true; }
#endif
#elif defined(FLASH_OPTCR_nDBANK)
  if (!(FLASH->OPTCR & FLASH_OPTCR_nDBANK)) { dual_bank = true; }
#endif

  /* F4 dual bank keeps the single bank sector sizes (each bank is
   * simply half the flash), only F7 dual bank halves them */
#if defined(STM32F4XX)
  uint32_t sector_size = small;
#else /* STM32F7XX */
  uint32_t sector_size = dual_bank ? small / 2 : small;
#endif
  uint32_t bank = dual_bank ? total / 2 : total;
  uint32_t last_offset = total - sector_size;
  /* bank relative offset of the last sector */
  uint32_t bank_offset = last_offset - (dual_bank ? bank : 0);

  /* need at least one full sized sector after the small first sectors,
   * which occupy the first sector_size bytes of the bank */
  if ((bank < 2 * sector_size) || (bank % sector_size)) { return -1; }

  flash->page_size = sector_size;
  flash->addr = PFLASH_BASE + last_offset;

  /* sector index: 4 small + 1 medium sector precede the first full
   * sized sector, dual bank sectors of bank2 are indexed from 12 */
  uint32_t sector = 5 + (bank_offset - sector_size) / sector_size
                    + (dual_bank ? 12 : 0);

  /* SNB field encoding: bank2 sectors (12..) map to 16.. */
  flash->snb = (sector < 12) ? sector : sector + 4;

  return 0;
}

/**
 * Busy-wait until the flash controller is idle.
 * @return 0 when idle, -1 if the poll limit was exceeded
 */
static int32_t pflash_wait(void)
{
  for (uint32_t i = 0; i < PFLASH_WAIT_MAX_POLLS; i++) {
    if (!(FLASH->SR & FLASH_SR_BSY)) { return 0; }
  }
  return -1;
}

/** Unlock the flash control register (no-op when already unlocked). */
static void pflash_unlock(void)
{
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_UNLOCK_KEY1;
    FLASH->KEYR = FLASH_UNLOCK_KEY2;
  }
}

/** Re-lock the flash control register against accidental writes. */
static void pflash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * Erase the settings sector (SER + SNB sequence).
 * The CPU stalls on any flash fetch while the erase is in progress
 * (up to ~2s for a large sector).
 * @param flash settings sector location, from pflash_detect()
 * @return 0 on success
 */
static int32_t pflash_erase(const struct FlashInfo *flash)
{
  pflash_unlock();
  if (pflash_wait()) { pflash_lock(); return -1; }
  FLASH->SR = PFLASH_SR_ERRORS; /* clear pending error flags */

  /* sector erase, 32bit parallelism (PSIZE = x32, VDD > 2.7V) */
  FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER |
              ((flash->snb << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk);
  FLASH->CR |= FLASH_CR_STRT;
  int32_t ret = pflash_wait();
  FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  pflash_lock();

  if (ret || (FLASH->SR & PFLASH_SR_ERRORS)) { return -1; }
  return 0;
}

/**
 * Program one 32bit word, executed entirely from RAM (.ramtext):
 * while the flash controller is busy the CPU must not fetch a single
 * instruction from flash or it would stall. Called with interrupts
 * masked, all polls bounded, keep it minimal and free of calls into
 * flash code.
 * @return 0 on success
 */
PFLASH_RAMFUNC static int32_t pflash_ram_program(uint32_t addr, uint32_t val)
{
  uint32_t i;

  for (i = 0; (i < PFLASH_PROG_MAX_POLLS) && (FLASH->SR & FLASH_SR_BSY); i++) { }
  if (FLASH->SR & FLASH_SR_BSY) { return -1; }

  FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
  *(volatile uint32_t *)addr = val;
  __DSB();
  for (i = 0; (i < PFLASH_PROG_MAX_POLLS) && (FLASH->SR & FLASH_SR_BSY); i++) { }

  uint32_t sr = FLASH->SR;
  FLASH->CR &= ~FLASH_CR_PG;

  return ((sr & FLASH_SR_BSY) || (sr & PFLASH_SR_ERRORS)) ? -1 : 0;
}

/**
 * Program one 32bit word (flash must be unlocked by the caller).
 *
 * Flight safe: the controller is confirmed idle first, then interrupts
 * are masked only for the microseconds of the actual programming,
 * executed from RAM so nothing can touch the stalled flash bus.
 *
 * @param addr destination address in erased flash, word aligned
 * @param buf  source of PFLASH_PROG_SIZE bytes
 * @return 0 on success
 */
static int32_t pflash_program_unit(const struct FlashInfo *flash __attribute__((unused)),
                                   uint32_t addr, const uint8_t *buf)
{
  uint32_t val;
  memcpy(&val, buf, 4);

  if (pflash_wait()) { return -1; } /* enter the critical zone idle */
  FLASH->SR = PFLASH_SR_ERRORS;     /* clear stale error flags */
  chSysLock();
  int32_t ret = pflash_ram_program(addr, val);
  chSysUnlock();
  return ret;
}

#elif defined(STM32H7XX)

/** H7 flash programming unit: one 256bit (32 byte) flash word. */
#define PFLASH_PROG_SIZE 32

/** All relevant H7 bank2 error flags. */
#define PFLASH_H7_SR_ERRORS (FLASH_SR_WRPERR | FLASH_SR_PGSERR | \
                             FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR)

/**
 * Detect the flash geometry and locate the last sector.
 *
 * Only the dual bank 2MB parts (e.g. H743) are supported: two banks
 * of 8x 128k sectors, the settings go in the last sector of bank2,
 * which has its own set of flash control registers.
 *
 * @return 0 on success
 */
static int32_t pflash_detect(struct FlashInfo *flash)
{
  uint32_t size_kb = *(volatile const uint16_t *)FLASHSIZE_BASE;
  if (size_kb != 2048) { return -1; }

  flash->page_size = 0x20000;
  flash->addr = PFLASH_BASE + size_kb * 1024 - flash->page_size;
  flash->snb = 7; /* last sector of bank2 */

  return 0;
}

/**
 * Busy-wait until bank2 is idle and its write buffer is drained.
 * @return 0 when idle, -1 if the poll limit was exceeded
 */
static int32_t pflash_wait(void)
{
  for (uint32_t i = 0; i < PFLASH_WAIT_MAX_POLLS; i++) {
    if (!(FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE))) { return 0; }
  }
  return -1;
}

/** Unlock the bank2 flash control register (no-op when already unlocked). */
static void pflash_unlock(void)
{
  if (FLASH->CR2 & FLASH_CR_LOCK) {
    FLASH->KEYR2 = FLASH_UNLOCK_KEY1;
    FLASH->KEYR2 = FLASH_UNLOCK_KEY2;
  }
}

/** Re-lock the bank2 flash control register against accidental writes. */
static void pflash_lock(void)
{
  FLASH->CR2 |= FLASH_CR_LOCK;
}

/**
 * Erase the settings sector (last sector of bank2).
 * Bank2 has its own control registers and erasing it does not stall
 * code fetched from bank1; still treat this as a ground-only operation.
 * @param flash settings sector location, from pflash_detect()
 * @return 0 on success
 */
static int32_t pflash_erase(const struct FlashInfo *flash)
{
  pflash_unlock();
  if (pflash_wait()) { pflash_lock(); return -1; }
  FLASH->CCR2 = PFLASH_H7_SR_ERRORS; /* clear pending error flags */

  /* sector erase, 32bit write parallelism (PSIZE = x32) */
  FLASH->CR2 = FLASH_CR_PSIZE_1 | FLASH_CR_SER |
               ((flash->snb << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk);
  FLASH->CR2 |= FLASH_CR_START;
  int32_t ret = pflash_wait();
  FLASH->CR2 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  pflash_lock();

  if (ret || (FLASH->SR2 & PFLASH_H7_SR_ERRORS)) { return -1; }
  return 0;
}

/**
 * Program one 256bit flash word, executed entirely from RAM (.ramtext)
 * with interrupts masked and bounded polls. On H7 the settings sector
 * is in bank2 while code runs from bank1, so fetches would not stall,
 * but the uniform RAM + masked window keeps this correct even for
 * firmware images that grow into bank2.
 * @param addr destination, 32 byte aligned
 * @param w    source of 8 words, 4 byte aligned, in RAM
 * @return 0 on success
 */
PFLASH_RAMFUNC static int32_t pflash_ram_program(uint32_t addr, const uint32_t *w)
{
  uint32_t i;

  for (i = 0; (i < PFLASH_PROG_MAX_POLLS) &&
       (FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE)); i++) { }
  if (FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE)) { return -1; }

  FLASH->CR2 = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
  __DSB();

  /* fill the write buffer with the complete flash word */
  volatile uint32_t *dst = (volatile uint32_t *)addr;
  for (int j = 0; j < 8; j++) { dst[j] = w[j]; }
  __DSB();
  for (i = 0; (i < PFLASH_PROG_MAX_POLLS) &&
       (FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)); i++) { }

  uint32_t sr = FLASH->SR2;
  FLASH->CR2 &= ~FLASH_CR_PG;

  return ((sr & (FLASH_SR_BSY | FLASH_SR_QW)) || (sr & PFLASH_H7_SR_ERRORS)) ? -1 : 0;
}

/**
 * Program one 256bit flash word (flash must be unlocked by the caller).
 *
 * Flight safe: the controller is confirmed idle first, then interrupts
 * are masked only for the actual programming, executed from RAM.
 *
 * @param addr destination address in erased flash, 32 byte aligned
 * @param buf  source of PFLASH_PROG_SIZE bytes, 4 byte aligned
 *             (guaranteed by pflash_program_buffer())
 * @return 0 on success
 */
static int32_t pflash_program_unit(const struct FlashInfo *flash __attribute__((unused)),
                                   uint32_t addr, const uint8_t *buf)
{
  const uint32_t *w = (const uint32_t *)(const void *)buf;

  if (pflash_wait()) { return -1; }  /* enter the critical zone idle */
  FLASH->CCR2 = PFLASH_H7_SR_ERRORS; /* clear stale error flags */
  chSysLock();
  int32_t ret = pflash_ram_program(addr, w);
  chSysUnlock();
  return ret;
}

#endif /* family specific primitives */

/**
 * Make cached flash reads coherent again after erase/program, so the
 * verification reads hit the actual flash content instead of stale
 * cache lines (which could false-fail or, worse, false-pass).
 *
 * F4: reset the ART accelerator caches, their only invalidation
 * mechanism (permitted only while the caches are disabled).
 * F7/H7: invalidate the affected L1 data cache lines, if a data
 * cache is present and enabled at runtime.
 */
static void pflash_cache_flush(const struct FlashInfo *flash __attribute__((unused)))
{
#if defined(STM32F4XX)
  /* reset the ART accelerator caches (only allowed while disabled) */
  uint32_t acr = FLASH->ACR;
  FLASH->ACR = acr & ~(FLASH_ACR_DCEN | FLASH_ACR_ICEN);
  FLASH->ACR |= FLASH_ACR_DCRST | FLASH_ACR_ICRST;
  FLASH->ACR &= ~(FLASH_ACR_DCRST | FLASH_ACR_ICRST);
  FLASH->ACR = acr;
#endif
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
  /* F7/H7 cores: invalidate the L1 data cache lines of the sector */
  if (SCB->CCR & SCB_CCR_DC_Msk) {
    SCB_InvalidateDCache_by_Addr((void *)flash->addr, flash->page_size);
  }
#endif
  __DSB();
  __ISB();
}

/*
 * Generic part, common to all families: an append only journal.
 */

/** Journal record marker ("PS1" + version), never 0xFFFFFFFF (erased). */
#define PJRN_MAGIC 0x50533101UL

/** Journal alignment: the programming unit, but at least one 32bit word. */
#define PFLASH_UNIT ((PFLASH_PROG_SIZE) > 4 ? (PFLASH_PROG_SIZE) : 4)

_Static_assert((PFLASH_UNIT & (PFLASH_UNIT - 1)) == 0, "PFLASH_UNIT must be a power of two");

/** Round a byte count up to a whole number of journal units. */
#define PFLASH_ALIGN_UP(x) (((uint32_t)(x) + PFLASH_UNIT - 1) & ~(uint32_t)(PFLASH_UNIT - 1))

/** Header block of a record: magic and data size words, unit padded. */
#define PJRN_HDR PFLASH_ALIGN_UP(8)
/** Checksum block of a record: one crc32 word, unit padded. */
#define PJRN_CRC PFLASH_ALIGN_UP(4)
/** Total flash footprint of a record carrying @p size data bytes. */
#define PJRN_RECORD_SIZE(size) (PJRN_HDR + PFLASH_ALIGN_UP(size) + PJRN_CRC)

/** Read one 32bit word directly from flash. */
static inline uint32_t pflash_read32(uint32_t addr)
{
  return *(volatile const uint32_t *)addr;
}

/**
 * Update a software CRC-32 (polynomial 0x04C11DB7, MSB first) over a
 * byte range, allowing non contiguous fields to be chained.
 *
 * Deliberately bitwise and table free: the settings blob is tiny, so a
 * 1k lookup table or the hardware CRC unit (extra clock and peripheral
 * dependency) would buy nothing. Zero RAM, portable.
 *
 * @param crc  running value, start with 0xFFFFFFFF
 * @param ptr  start address (RAM or flash)
 * @param size number of bytes
 * @return updated CRC-32
 */
static uint32_t pflash_crc32(uint32_t crc, uint32_t ptr, uint32_t size)
{
  for (uint32_t i = 0; i < size; i++) {
    crc ^= ((uint32_t)(*(volatile const uint8_t *)(ptr + i))) << 24;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80000000UL) { crc = (crc << 1) ^ 0x04C11DB7UL; }
      else { crc <<= 1; }
    }
  }
  return crc;
}

/**
 * CRC-32 of a record: chained over the data size word and the data
 * itself, so a corrupted size field can never validate.
 */
static uint32_t pflash_record_crc(uint32_t size, uint32_t data_addr)
{
  uint32_t crc = pflash_crc32(0xFFFFFFFFUL, (uint32_t)&size, 4);
  return pflash_crc32(crc, data_addr, size);
}

/**
 * Maximum data size of a single journal record.
 * @return sector size minus the record overhead
 */
static uint32_t pflash_max_size(const struct FlashInfo *flash)
{
  return flash->page_size - PJRN_HDR - PJRN_CRC;
}

/** Journal state, as found in the settings sector by pflash_scan(). */
struct PflashJournal {
  uint32_t last_off;  ///< sector offset of the newest valid record, or UINT32_MAX
  uint32_t last_size; ///< data size of the newest valid record
  uint32_t chain_off; ///< offset just past the last valid record of the chain
  uint32_t free_off;  ///< first offset of fully erased space, unit aligned
};

/**
 * Scan the settings sector.
 *
 * Forward pass: walk the record chain from the sector start, remember
 * the newest record whose checksum validates, stop at the first hole
 * or corruption (later records cannot be trusted once the chain is
 * broken, since record boundaries derive from the stored sizes).
 *
 * Backward pass: locate the erased tail of the sector as everything
 * after the last non erased word. Comparing it against the chain end
 * tells a healthy journal (free_off <= chain_off, where any 0xFF
 * padding or checksum words of valid records may make free_off
 * smaller) from one holding torn or foreign data past the chain
 * (free_off > chain_off): such garbage must never be programmed
 * again (on H7 reprogramming a flash word is a hard error), and a
 * record appended beyond it would be unreachable by this very scan,
 * so the sector is reclaimed instead (see persistent_write()).
 *
 * @param flash    settings sector location
 * @param[out] j   scan result
 */
static void pflash_scan(const struct FlashInfo *flash, struct PflashJournal *j)
{
  uint32_t off = 0;

  j->last_off = UINT32_MAX;
  j->last_size = 0;

  while ((off + PJRN_HDR + PJRN_CRC) <= flash->page_size) {
    if (pflash_read32(flash->addr + off) != PJRN_MAGIC) { break; }

    uint32_t size = pflash_read32(flash->addr + off + 4);
    if ((size == 0) || (size > pflash_max_size(flash))) { break; }
    uint32_t rec = PJRN_RECORD_SIZE(size);
    if (rec > (flash->page_size - off)) { break; }

    uint32_t data = flash->addr + off + PJRN_HDR;
    uint32_t stored_crc = pflash_read32(data + PFLASH_ALIGN_UP(size));
    if (pflash_record_crc(size, data) != stored_crc) { break; }

    j->last_off = off;
    j->last_size = size;
    off += rec;
  }
  j->chain_off = off;

  /* backward pass: first fully erased offset in the whole sector */
  uint32_t used = 0;
  for (uint32_t end = flash->page_size; end > 0; end -= 4) {
    if (pflash_read32(flash->addr + end - 4) != 0xFFFFFFFFUL) {
      used = end;
      break;
    }
  }
  j->free_off = PFLASH_ALIGN_UP(used);
}

/**
 * ChibiOS GCC linker symbols used to compute the end of the firmware
 * image in flash: load address of the initialized data section plus its
 * size (the ChibiOS linker rules place .data, which also carries the
 * .ramtext programming routines, last in the flash image).
 */
extern uint8_t __textdata_base__, __data_base__, __data_end__;

/**
 * Locate the settings sector and make sure the firmware image does
 * not reach into it, so the driver can never erase its own code as
 * the firmware grows.
 * @param[out] flash filled with the settings sector location
 * @return 0 on success, -1 if unsupported geometry or unsafe overlap
 */
static int32_t pflash_get(struct FlashInfo *flash)
{
  if (pflash_detect(flash)) { return -1; }

  uint32_t image_end = (uint32_t)&__textdata_base__ +
                       ((uint32_t)&__data_end__ - (uint32_t)&__data_base__);
  if (image_end > flash->addr) { return -1; }

  return 0;
}

/**
 * True while erasing the settings sector would endanger the flight:
 * sector erase is only permitted with motors off and not in flight.
 */
static bool pflash_in_flight(void)
{
  return autopilot_get_motors_on() || autopilot_in_flight();
}

/**
 * Program a byte buffer as a sequence of programming units, padding the
 * last partial unit with 0xFF (the erased state, so the padding does
 * not disturb anything). Flash must be unlocked by the caller.
 * @param flash settings sector location
 * @param dst   destination flash address, unit aligned
 * @param src   source address in RAM
 * @param size  number of bytes to program
 * @return 0 on success
 */
static int32_t pflash_program_buffer(const struct FlashInfo *flash,
                                     uint32_t dst, uint32_t src, uint32_t size)
{
  /* 4 byte aligned so the H7 primitive can use direct word access */
  uint8_t unit[PFLASH_PROG_SIZE] __attribute__((aligned(4)));

  for (uint32_t i = 0; i < size; i += PFLASH_PROG_SIZE) {
    uint32_t n = size - i;
    if (n >= PFLASH_PROG_SIZE) {
      memcpy(unit, (const void *)(src + i), PFLASH_PROG_SIZE);
    } else {
      memset(unit, 0xFF, PFLASH_PROG_SIZE);
      memcpy(unit, (const void *)(src + i), n);
    }
    if (pflash_program_unit(flash, dst + i, unit)) { return -1; }
  }
  return 0;
}

/**
 * Erase the settings sector and verify word by word that it reads back
 * fully erased (flash erases can fail silently).
 *
 * Ground only: the erase stalls flash fetches for up to ~2 seconds,
 * callers must check pflash_in_flight() first.
 *
 * @param flash settings sector location
 * @return 0 on success
 */
static int32_t pflash_erase_verified(const struct FlashInfo *flash)
{
  if (pflash_erase(flash)) { return -1; }

  pflash_cache_flush(flash);

  /* verify erase */
  for (uint32_t i = 0; i < flash->page_size; i += 4) {
    if (pflash_read32(flash->addr + i) != 0xFFFFFFFFUL) { return -1; }
  }
  return 0;
}

/**
 * Append one journal record into erased space and read it back for
 * verification (a false "stored OK" is worse than an error).
 *
 * Flight safe: only unit programming, no erase; interrupts run
 * normally between units.
 *
 * @param flash settings sector location
 * @param off   destination sector offset, fully erased, record fits
 * @param src   RAM address of the data
 * @param size  data size in bytes
 * @return PFLASH_OK or PFLASH_ERR_VERIFY
 */
static int32_t pflash_append_record(const struct FlashInfo *flash,
                                    uint32_t off, uint32_t src, uint32_t size)
{
  uint32_t crc = pflash_record_crc(size, src);
  uint32_t base = flash->addr + off;
  int32_t ret;

  uint8_t hdr[PJRN_HDR];
  memset(hdr, 0xFF, PJRN_HDR);
  const uint32_t magic = PJRN_MAGIC;
  memcpy(&hdr[0], &magic, 4);
  memcpy(&hdr[4], &size, 4);

  pflash_unlock();
  ret = pflash_program_buffer(flash, base, (uint32_t)hdr, PJRN_HDR);
  if (ret == 0) {
    ret = pflash_program_buffer(flash, base + PJRN_HDR, src, size);
  }
  if (ret == 0) {
    ret = pflash_program_buffer(flash, base + PJRN_HDR + PFLASH_ALIGN_UP(size),
                                (uint32_t)&crc, 4);
  }
  pflash_lock();
  if (ret) { return PFLASH_ERR_VERIFY; }

  pflash_cache_flush(flash);

  /* verify the complete record by reading back */
  if (pflash_read32(base) != PJRN_MAGIC) { return PFLASH_ERR_VERIFY; }
  if (pflash_read32(base + 4) != size) { return PFLASH_ERR_VERIFY; }
  for (uint32_t i = 0; i < size; i++) {
    if ((*(volatile const uint8_t *)(base + PJRN_HDR + i)) !=
        (*(const uint8_t *)(src + i))) {
      return PFLASH_ERR_VERIFY;
    }
  }
  if (pflash_read32(base + PJRN_HDR + PFLASH_ALIGN_UP(size)) != crc) {
    return PFLASH_ERR_VERIFY;
  }

  return PFLASH_OK;
}

/**
 * Store the settings as a new journal record in the last flash sector.
 *
 * Flight safe: appends into erased space without erasing (microsecond
 * interrupt masking per programmed unit only). When the journal is
 * full, the sector is reclaimed with a blocking erase strictly on the
 * ground; in flight PFLASH_ERR_FULL is returned instead and the store
 * can be retried after landing.
 *
 * Thread safe, must be called from thread context (not from an ISR).
 *
 * @param ptr  RAM address of the settings data
 * @param size data size in bytes
 * @return PFLASH_OK on success, negative pflash_status otherwise
 */
int32_t persistent_write(void *ptr, uint32_t size)
{
  struct FlashInfo flash;
  struct PflashJournal j;

  if (pflash_get(&flash)) { return PFLASH_ERR_SECTOR; }
  if ((size == 0) || (size > pflash_max_size(&flash))) { return PFLASH_ERR_SIZE; }

  chMtxLock(&pflash_mutex);

  pflash_scan(&flash, &j);

  /* append strictly at the end of the valid record chain, so the new
   * record is always found by the boot time scan */
  uint32_t off = j.chain_off;

  if ((j.free_off > off) || (PJRN_RECORD_SIZE(size) > (flash.page_size - off))) {
    /* the sector is full, or it holds torn/foreign data beyond the
     * chain (power cut, legacy layout) which must never be programmed
     * again and would make an appended record unreachable: reclaiming
     * either state needs a sector erase, ground only */
    if (pflash_in_flight()) {
      chMtxUnlock(&pflash_mutex);
      return PFLASH_ERR_FULL;
    }
    if (pflash_erase_verified(&flash)) {
      chMtxUnlock(&pflash_mutex);
      return PFLASH_ERR_VERIFY;
    }
    off = 0;
  }

  int32_t ret = pflash_append_record(&flash, off, (uint32_t)ptr, size);

  chMtxUnlock(&pflash_mutex);
  return ret;
}

/**
 * Load the newest valid settings record from the journal.
 *
 * The record checksum (chained over size and data) guarantees that
 * stale, torn or corrupt data is never loaded; on any failure the
 * caller falls back to the airframe file defaults.
 *
 * Thread safe, must be called from thread context (not from an ISR).
 *
 * @param ptr  RAM destination for the settings data
 * @param size expected data size in bytes (must equal the stored size)
 * @return PFLASH_OK on success, negative pflash_status otherwise
 */
int32_t persistent_read(void *ptr, uint32_t size)
{
  struct FlashInfo flash;
  struct PflashJournal j;

  if (pflash_get(&flash)) { return PFLASH_ERR_SECTOR; }
  if ((size == 0) || (size > pflash_max_size(&flash))) { return PFLASH_ERR_SIZE; }

  chMtxLock(&pflash_mutex);

  pflash_scan(&flash, &j);
  if ((j.last_off == UINT32_MAX) || (j.last_size != size)) {
    chMtxUnlock(&pflash_mutex);
    return PFLASH_ERR_NODATA;
  }

  uint32_t data = flash.addr + j.last_off + PJRN_HDR;
  for (uint32_t i = 0; i < size; i++) {
    ((uint8_t *)ptr)[i] = *(volatile const uint8_t *)(data + i);
  }

  chMtxUnlock(&pflash_mutex);
  return PFLASH_OK;
}

/**
 * Erase the settings sector, invalidating any stored settings
 * (a following persistent_read() will fail with PFLASH_ERR_NODATA).
 *
 * Ground only: the blocking erase is refused with PFLASH_ERR_INFLIGHT
 * while the autopilot reports motors on or in flight.
 *
 * Thread safe, must be called from thread context (not from an ISR).
 *
 * @return PFLASH_OK on success, negative pflash_status otherwise
 */
int32_t persistent_clear(void)
{
  struct FlashInfo flash;

  if (pflash_get(&flash)) { return PFLASH_ERR_SECTOR; }

  chMtxLock(&pflash_mutex);

  if (pflash_in_flight()) {
    chMtxUnlock(&pflash_mutex);
    return PFLASH_ERR_INFLIGHT;
  }

  int32_t ret = pflash_erase_verified(&flash) ? PFLASH_ERR_VERIFY : PFLASH_OK;

  chMtxUnlock(&pflash_mutex);
  return ret;
}

#else /* unsupported MCU family: dummy implementation so it still links */

/*
 * All dummies return an error; persistent_read() failing means
 * settings_init() falls back to the airframe file defaults instead
 * of silently loading garbage.
 */

int32_t persistent_write(void *ptr __attribute__((unused)), uint32_t size __attribute__((unused)))
{
  return -1;
}

int32_t persistent_read(void *ptr __attribute__((unused)), uint32_t size __attribute__((unused)))
{
  return -1;
}

int32_t persistent_clear(void)
{
  return -1;
}

#endif
