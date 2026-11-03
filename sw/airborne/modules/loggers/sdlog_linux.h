/*
 * Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
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
 */

/**
 * @file modules/loggers/sdlog_linux.h
 * @brief Linux (and NPS) replacement of the ChibiOS sdLog API.
 *
 * Only the part of the sdLog API used by the modules is provided, so the
 * modules logging on the SD card work unchanged on Linux and in simulation.
 */

#ifndef SDLOG_LINUX_H
#define SDLOG_LINUX_H

#include "std.h"
#include <stddef.h>

typedef enum {
  SDLOG_OK,
  SDLOG_NOCARD,
  SDLOG_FATFS_ERROR,
  SDLOG_FATFS_NOENT,
  SDLOG_FSFULL,
  SDLOG_FDFULL,
  SDLOG_QUEUEFULL,
  SDLOG_MEMFULL,
  SDLOG_NOTHREAD,
  SDLOG_INTERNAL_ERROR,
  SDLOG_CANNOT_EXPAND,
  SDLOG_LOGNUM_ERROR,
  SDLOG_WAS_LAUNCHED,
} SdioError;

typedef int8_t FileDes;

extern SdioError sdLogWriteLog(const FileDes fd, const char *fmt, ...)
__attribute__((format(printf, 2, 3)));
extern SdioError sdLogWriteRaw(const FileDes fd, const uint8_t *buffer, const size_t len);
extern SdioError sdLogWriteByte(const FileDes fd, const uint8_t value);
extern SdioError sdLogFlushLog(const FileDes fd);
extern SdioError sdLogFlushAllLogs(void);
extern size_t sdLogGetNbBytesWrittenToStorage(void);
extern SdioError sdLogGetStorageStatus(void);

#endif /* SDLOG_LINUX_H */
