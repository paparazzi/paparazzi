/*
 * Copyright (C) 2026 The Paparazzi Team
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file arch/chibios/modules/core/settings_arch.c
 * Persistent settings in the internal flash, using the ChibiOS EFL driver.
 *
 * The settings sector (by default the last flash sector) holds a list of
 * records, each one appended after the previous one:
 *
 *   | record | record | ... | erased (0xFF) |
 *   record: | header: magic, hash, size, crc | data | padding to 32 bytes |
 *
 * - Read: the newest record with a valid crc and with the size and settings
 *   hash of this firmware is loaded. Otherwise the airframe defaults stay.
 * - Write: a new record is appended. A sector erase stalls the CPU for up to
 *   a few seconds, so it is never done while the motors are on or in flight:
 *   when the sector is full, the write fails and the erase is done by the next
 *   write or clear on the ground.
 * - A power cut during a write leaves a record with a bad crc: the previous
 *   record is used, and the next write on the ground erases the sector.
 * - Each record is programmed in one go and padded to 32 bytes, so no flash
 *   word is written twice (required on the STM32H7, 32 byte flash words).
 *
 * MCUs without an EFL driver in ChibiOS are not supported (build error in halconf.h).
 */

#include "modules/core/settings.h"
#include "generated/settings.h"
#include "autopilot.h"
#include <hal.h>
#include <string.h>

#if USE_PERSISTENT_SETTINGS

/** Flash sector used for the settings, default: the last one */
#ifndef PERSISTENT_SETTINGS_SECTOR
#define PERSISTENT_SETTINGS_SECTOR (flashGetDescriptor(&EFLD1)->sectors_count - 1)
#endif

#define RECORD_MAGIC 0x50505331UL   ///< "PPS1"
#define RECORD_ALIGN 32             ///< record size multiple, >= flash programming unit

struct RecordHeader {
  uint32_t magic;
  uint32_t hash;    ///< PERSISTENT_SETTINGS_HASH of the firmware that wrote it
  uint32_t size;    ///< data size in bytes
  uint32_t crc;     ///< crc32 of the data
};

/** Location of the settings sector */
struct SettingsSector {
  BaseFlash *flash;
  flash_sector_t sector;
  flash_offset_t offset;    ///< sector start, from the start of the flash
  uint32_t size;
  const uint8_t *address;   ///< sector start in memory
};

/** First free byte of the flash after the firmware (ChibiOS linker script) */
extern uint8_t __flash0_free__;

static uint32_t record_length(uint32_t data_size)
{
  uint32_t length = sizeof(struct RecordHeader) + data_size;
  return ((length + RECORD_ALIGN - 1) / RECORD_ALIGN) * RECORD_ALIGN;
}

/** RAM copy of a record of this firmware, programmed in one go */
static union {
  uint8_t bytes[((sizeof(struct RecordHeader) + sizeof(struct PersistentSettings) + RECORD_ALIGN - 1)
                 / RECORD_ALIGN) * RECORD_ALIGN];
  struct RecordHeader header;
} record;

static uint32_t crc32(const uint8_t *data, uint32_t size)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0; i < size; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320UL & -(crc & 1));
    }
  }
  return ~crc;
}

static bool is_erased(const uint8_t *data, uint32_t size)
{
  for (uint32_t i = 0; i < size; i++) {
    if (data[i] != 0xFF) {
      return false;
    }
  }
  return true;
}

/** An erase stalls the CPU: only allowed on the ground with motors off */
static bool erase_allowed(void)
{
  return !autopilot_get_motors_on() && !autopilot_in_flight();
}

/**
 * Start the flash driver and locate the settings sector.
 * Fails when the firmware reaches into the sector.
 */
static bool get_sector(struct SettingsSector *s)
{
  static bool started = false;
  if (!started) {
    eflStart(&EFLD1, NULL);
    started = true;
  }

  s->flash = getBaseFlash(&EFLD1);
  s->sector = PERSISTENT_SETTINGS_SECTOR;
  s->offset = flashGetSectorOffset(s->flash, s->sector);
  s->size = flashGetSectorSize(s->flash, s->sector);
  s->address = flashGetOffsetAddress(s->flash, s->offset);

  return &__flash0_free__ <= s->address;
}

/**
 * Walk the records of the sector.
 * @param s      settings sector
 * @param size   data size of this firmware's settings
 * @param newest offset of the newest valid record of this firmware, or -1
 * @return offset after the last valid record, where the next one goes
 */
static uint32_t scan_records(const struct SettingsSector *s, uint32_t size, int32_t *newest)
{
  uint32_t offset = 0;
  *newest = -1;

  while (offset + sizeof(struct RecordHeader) <= s->size) {
    struct RecordHeader h;
    memcpy(&h, s->address + offset, sizeof(h));
    if (h.magic != RECORD_MAGIC || h.size > s->size - offset - sizeof(h) ||
        crc32(s->address + offset + sizeof(h), h.size) != h.crc) {
      break;  // erased space or a broken record: the list ends here
    }
    if (h.hash == PERSISTENT_SETTINGS_HASH && h.size == size) {
      *newest = (int32_t)offset;
    }
    offset += record_length(h.size);
  }
  return offset;
}

static bool erase_sector(const struct SettingsSector *s)
{
  if (flashStartEraseSector(s->flash, s->sector) != FLASH_NO_ERROR ||
      flashWaitErase(s->flash) != FLASH_NO_ERROR) {
    return false;
  }
  return flashVerifyErase(s->flash, s->sector) == FLASH_NO_ERROR;
}

/** Program a record in one go, then check the result */
static bool write_record(const struct SettingsSector *s, uint32_t offset, const void *data, uint32_t size)
{
  uint32_t length = record_length(size);

  memset(record.bytes, 0xFF, sizeof(record.bytes));
  record.header.magic = RECORD_MAGIC;
  record.header.hash = PERSISTENT_SETTINGS_HASH;
  record.header.size = size;
  record.header.crc = crc32(data, size);
  memcpy(record.bytes + sizeof(struct RecordHeader), data, size);

  if (flashProgram(s->flash, s->offset + offset, length, record.bytes) != FLASH_NO_ERROR) {
    return false;
  }
  // flash is cached on the F7: drop the old (erased) content before reading back
  cacheBufferInvalidate(s->address + offset, length);
  return memcmp(s->address + offset, record.bytes, length) == 0;
}

int32_t persistent_write(void *ptr, uint32_t size)
{
  struct SettingsSector s;
  if (!get_sector(&s) || size == 0 || record_length(size) > sizeof(record.bytes)) {
    return -1;
  }

  flashAcquireExclusive(s.flash);

  int32_t newest;
  uint32_t offset = scan_records(&s, size, &newest);
  bool fits = offset + record_length(size) <= s.size &&
              is_erased(s.address + offset, record_length(size));

  int32_t ret = -1;
  if (!fits) {
    // sector full, or a broken record from a power cut: erase on the ground only
    if (erase_allowed() && erase_sector(&s)) {
      offset = 0;
      fits = true;
    }
  }
  if (fits && write_record(&s, offset, ptr, size)) {
    ret = 0;
  }

  flashReleaseExclusive(s.flash);
  return ret;
}

int32_t persistent_read(void *ptr, uint32_t size)
{
  struct SettingsSector s;
  if (!get_sector(&s)) {
    return -1;
  }

  flashAcquireExclusive(s.flash);
  int32_t newest;
  scan_records(&s, size, &newest);
  if (newest >= 0) {
    memcpy(ptr, s.address + newest + sizeof(struct RecordHeader), size);
  }
  flashReleaseExclusive(s.flash);

  return newest >= 0 ? 0 : -1;
}

int32_t persistent_clear(void)
{
  struct SettingsSector s;
  if (!get_sector(&s) || !erase_allowed()) {
    return -1;
  }

  flashAcquireExclusive(s.flash);
  bool ok = erase_sector(&s);
  flashReleaseExclusive(s.flash);

  return ok ? 0 : -1;
}

#else /* !USE_PERSISTENT_SETTINGS */

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

#endif /* USE_PERSISTENT_SETTINGS */
