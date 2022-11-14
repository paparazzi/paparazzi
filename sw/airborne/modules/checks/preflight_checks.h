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
#include <stdarg.h>

struct preflight_result_t {
  char *message;
  uint16_t max_len;
  uint16_t fail_cnt;
  uint16_t success_cnt;
};

typedef void (*preflight_check_f)(struct preflight_result_t *result);

struct preflight_check_t {
  preflight_check_f func;
  struct preflight_check_t *next;
};

extern bool preflight_bypass;
extern void preflight_check_register(struct preflight_check_t *check, preflight_check_f func);
extern bool preflight_check(void);
extern void preflight_error(struct preflight_result_t *result, const char *fmt, ...);
extern void preflight_success(struct preflight_result_t *result, const char *fmt, ...);

#endif /* PREFLIGHT_CHECKS_H */
