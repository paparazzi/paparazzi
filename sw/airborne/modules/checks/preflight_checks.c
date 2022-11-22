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

#ifndef PREFLIGHT_MAX_MSG_BUF
#define PREFLIGHT_MAX_MSG_BUF 512
#endif

static struct preflight_check_t *preflight_head = NULL;

void preflight_check_register(struct preflight_check_t *check, preflight_check_f func) {
  // Prepend the preflight check
  struct preflight_check_t *next = preflight_head;
  preflight_head = check;
  check->func = func;
  check->next = next;
}

#include "modules/datalink/telemetry.h"
bool preflight_check(void) {
  char error_msg[PREFLIGHT_MAX_MSG_BUF];
  struct preflight_error_t error = {
    .message = error_msg,
    .max_len = PREFLIGHT_MAX_MSG_BUF,
    .fail_cnt = 0,
    .success_cnt = 0
  };

  // Go through all the checks
  struct preflight_check_t *check = preflight_head;
  while(check != NULL) {
    // Peform the check and register errors
    check->func(&error);
    check = check->next;
  }

  // We failed a check
  if(error.fail_cnt > 0) {
    printf("Preflight fail [%d/%d]:\n%s\n", error.fail_cnt, (error.fail_cnt+error.success_cnt), error_msg);
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, PREFLIGHT_MAX_MSG_BUF-error.max_len, error_msg);
    return false;
  }

  // Return success if we didn't fail a preflight check
  printf("Preflight success [%d]\n", error.success_cnt);
  return true;
}
