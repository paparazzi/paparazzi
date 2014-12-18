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

#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

#define GIT_VERSION        STRINGIFY(GIT_SHA1)
#define PPRZ_VERSION_DESC  STRINGIFY(GIT_DESC)
#define PPRZ_VERSION       STRINGIFY(PPRZ_VER)

/** paparazzi version encoded as one integer */
#define PPRZ_VERSION_INT (PPRZ_VER_MAJOR * 100 + PPRZ_VER_MINOR * 10 + PPRZ_VER_PATCH)

#endif /* PPRZ_VERSION_H */
