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

struct preflight_error_t {
  char *message;
  uint16_t max_len;
  uint16_t fail_cnt;
  uint16_t success_cnt;
};

typedef void (*preflight_check_f)(struct preflight_error_t *error);

static inline void preflight_error(struct preflight_error_t *error, const char *fmt, ...) {
  // Record the error count
  error->fail_cnt++;

  // No more space in the message
  if(error->max_len <= 0) {
    return;
  }

  // Add the error
  va_list args;
  va_start(args, fmt);
  int rc = vsnprintf(error->message, error->max_len, fmt, args);
  va_end(args);

  // Remove the length (minus \0 character) from the buffer
  if(rc > 0) {
    error->max_len -= (rc - 1);
    error->message += (rc - 1);
  }
}

static inline void preflight_success(struct preflight_error_t *error, const char *fmt __attribute__((unused)), ...) {
  // Record the success count
  error->success_cnt++;
}

struct preflight_check_t {
  preflight_check_f func;
  struct preflight_check_t *next;
};

extern void preflight_check_register(struct preflight_check_t *check, preflight_check_f func);
extern bool preflight_check(void);

#endif /* PREFLIGHT_CHECKS_H */
