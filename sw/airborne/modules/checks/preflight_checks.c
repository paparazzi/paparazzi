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
 * @file "modules/checks/preflight_checks.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Adds preflight checks for takeoff
 */

#include "preflight_checks.h"

static struct preflight_check_t *preflight_head = NULL;

void preflight_check_register(struct preflight_check_t *check, preflight_check_f func) {
  // Prepend the preflight check
  struct preflight_check_t *next = preflight_head;
  preflight_head = check;
  check->func = func;
  check->next = next;
}

bool preflight_check(void) {
  char error_msg[240];
  uint16_t checks_cnt = 0;
  uint16_t checks_fail_cnt = 0;
  bool success = true;

  // Go through all the checks
  struct preflight_check_t *check = preflight_head;
  while(check != NULL) {
    checks_cnt++;

    // Peform the check and register fails
    if(!check->func(error_msg)) {
      printf("Preflight fail: %s", error_msg);
      success = false;
      checks_fail_cnt++;
    }

    check = check->next;
  }

  // Return the result if a single check fails
  return success;
}
