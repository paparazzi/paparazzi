/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com
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

/** @file pprz_version.h
 *
 * Provides stringified paparazzi version numbers.
 *
 * Defined at build time:
 * - GIT_SHA1: full git SHA1 hash
 * - GIT_DESC: paparazzi version description (from paparazzi_version using git describe)
 * - PPRZ_VER: paparazzi version in <MAJOR>.<MINOR>.<PATCH> format
 * - PPRZ_VER_MAJOR: major version number
 * - PPRZ_VER_MINOR: minor version number
 * - PPRZ_VER_PATCH: patch version number
 *
 * Stringified defines provided:
 * - GIT_VERSION
 * - PPRZ_VERSION_DESC
 * - PPRZ_VERSION
 *
 *
 */

#ifndef PPRZ_VERSION_H
#define PPRZ_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef STRINGIFY
#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)
#endif

#define GIT_VERSION        STRINGIFY(GIT_SHA1)
#define PPRZ_VERSION_DESC  STRINGIFY(GIT_DESC)
#define PPRZ_VERSION       STRINGIFY(PPRZ_VER)

/** paparazzi version encoded as one 32bit integer */
#define PPRZ_VERSION_INT (PPRZ_VER_MAJOR * 10000 + PPRZ_VER_MINOR * 100 + PPRZ_VER_PATCH)

static inline uint8_t nibble_from_char(char c)
{
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  return 255;
}

/** Get git SHA1 of paparazzi version.
 * Write the first 8bytes (16chars) to byte array.
 * @param sha1 array to write to
 */
static inline void get_pprz_git_version(uint8_t sha1[8])
{
  static char *git_sha = GIT_VERSION;
  uint8_t *p;
  uint8_t i;

  for (i = 0, p = (uint8_t *) git_sha; i < 8; i++) {
    sha1[i] = (nibble_from_char(*p) << 4) | nibble_from_char(*(p + 1));
    p += 2;
  }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_VERSION_H */
