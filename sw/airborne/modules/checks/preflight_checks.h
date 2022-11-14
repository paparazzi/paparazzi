/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
 * @file "modules/checks/preflight_checks.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Adds preflight checks for takeoff
 */

#ifndef PREFLIGHT_CHECKS_H
#define PREFLIGHT_CHECKS_H

#include "std.h"

typedef bool (*preflight_check_f)(char *error_msg);

struct preflight_check_t {
  preflight_check_f func;
  struct preflight_check_t *next;
};

extern void preflight_check_register(struct preflight_check_t *check, preflight_check_f func);
extern bool preflight_check(void);

#endif /* PREFLIGHT_CHECKS_H */
