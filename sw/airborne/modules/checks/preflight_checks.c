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
#include "modules/datalink/telemetry.h"
#include <stdio.h>

/** Maximum combined message size for storing the errors */
#ifndef PREFLIGHT_CHECK_MAX_MSGBUF
#define PREFLIGHT_CHECK_MAX_MSGBUF 1024
#endif

/** Seperating character for storing the errors */
#ifndef PREFLIGHT_CHECK_SEPERATOR
#define PREFLIGHT_CHECK_SEPERATOR ';'
#endif

/** Only send messages down every xx amount of seconds */
#ifndef PREFLIGHT_CHECK_INFO_TIMEOUT
#define PREFLIGHT_CHECK_INFO_TIMEOUT 5
#endif

static struct preflight_check_t *preflight_head = NULL;
bool preflight_bypass = FALSE;

/**
 * @brief Register a preflight check and add it to the linked list
 *
 * @param check The check to add containing a linked list
 * @param func The function to register for the check
 */
void preflight_check_register(struct preflight_check_t *check, preflight_check_f func)
{
  // Prepend the preflight check
  struct preflight_check_t *next = preflight_head;
  preflight_head = check;
  check->func = func;
  check->next = next;
}

/**
 * @brief Perform all the preflight checks
 *
 * @return true When all preflight checks are successful
 * @return false When one or more preflight checks fail
 */
bool preflight_check(void)
{
  static float last_info_time = 0;
  char error_msg[PREFLIGHT_CHECK_MAX_MSGBUF];
  struct preflight_result_t result = {
    .message = error_msg,
    .max_len = PREFLIGHT_CHECK_MAX_MSGBUF,
    .fail_cnt = 0,
    .warning_cnt = 0,
    .success_cnt = 0
  };

  // Go through all the checks
  struct preflight_check_t *check = preflight_head;
  while (check != NULL) {
    // Peform the check and register errors
    check->func(&result);
    check = check->next;
  }

  // We failed a check or have a warning
  if (result.fail_cnt > 0 || result.warning_cnt > 0) {
    // Only send every xx amount of seconds
    if ((get_sys_time_float() - last_info_time) > PREFLIGHT_CHECK_INFO_TIMEOUT) {
      // Record the total
      int rc = 0;
      if (result.fail_cnt > 0) {
        rc = snprintf(result.message, result.max_len, "*Preflight fail [fail:%d warn:%d tot:%d]*", result.fail_cnt,
                      result.warning_cnt, (result.fail_cnt + result.warning_cnt + result.success_cnt));
      } else {
        rc = snprintf(result.message, result.max_len, "*Preflight success with warnings [%d/%d]*", result.warning_cnt,
                      (result.fail_cnt + result.warning_cnt + result.success_cnt));
      }
      if (rc > 0) {
        result.max_len -= rc;
      }

      // Send the errors seperatly
      uint16_t last_sendi = 0;
      for (uint16_t i = 0; i <= PREFLIGHT_CHECK_MAX_MSGBUF - result.max_len; i++) {
        if (error_msg[i] == PREFLIGHT_CHECK_SEPERATOR || i == (PREFLIGHT_CHECK_MAX_MSGBUF - result.max_len)) {
          DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, i - last_sendi, &error_msg[last_sendi]);
          last_sendi = i + 1;
        }
      }

      // Update the time
      last_info_time = get_sys_time_float();
    }

    // Only if we fail a check
    if (result.fail_cnt > 0) {
      return false;
    } else {
      return true;
    }
  }

  // Send success down
  int rc = snprintf(error_msg, PREFLIGHT_CHECK_MAX_MSGBUF, "Preflight success [%d]", result.success_cnt);
  if (rc > 0) {
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, error_msg);
  }

  // Return success if we didn't fail a preflight check
  return true;
}

/**
 * @brief Register a preflight error used inside the preflight checking functions
 *
 * @param result Where the error gets registered
 * @param fmt A formatted string describing the error used in a vsnprintf
 * @param ... The arguments for the vsnprintf
 */
void preflight_error(struct preflight_result_t *result, const char *fmt, ...)
{
  // Record the error count
  result->fail_cnt++;

  // No more space in the message
  if (result->max_len <= 0) {
    return;
  }

  // Add the error
  va_list args;
  va_start(args, fmt);
  int rc = vsnprintf(result->message, result->max_len, fmt, args);
  va_end(args);

  // Remove the length from the buffer if it was successfull
  if (rc > 0) {
    result->max_len -= rc;
    result->message += rc;

    // Add seperator if it fits
    if (result->max_len > 0) {
      result->message[0] = PREFLIGHT_CHECK_SEPERATOR;
      result->max_len--;
      result->message++;

      // Add the '\0' character
      if (result->max_len > 0) {
        result->message[0] = 0;
      }
    }
  }
}

/**
 * @brief Register a preflight error used inside the preflight checking functions
 *
 * @param result Where the error gets registered
 * @param fmt A formatted string describing the error used in a vsnprintf
 * @param ... The arguments for the vsnprintf
 */
void preflight_warning(struct preflight_result_t *result, const char *fmt, ...)
{
  // Record the warning count
  result->warning_cnt++;

  // No more space in the message
  if (result->max_len <= 0) {
    return;
  }

  // Add the warning to the error string
  va_list args;
  va_start(args, fmt);
  int rc = vsnprintf(result->message, result->max_len, fmt, args);
  va_end(args);

  // Remove the length from the buffer if it was successfull
  if (rc > 0) {
    result->max_len -= rc;
    result->message += rc;

    // Add seperator if it fits
    if (result->max_len > 0) {
      result->message[0] = PREFLIGHT_CHECK_SEPERATOR;
      result->max_len--;
      result->message++;

      // Add the '\0' character
      if (result->max_len > 0) {
        result->message[0] = 0;
      }
    }
  }
}

/**
 * @brief Register a preflight success used inside the preflight checking functions
 *
 * @param result
 * @param __attribute__
 * @param ...
 */
void preflight_success(struct preflight_result_t *result, const char *fmt __attribute__((unused)), ...)
{
  // Record the success count
  result->success_cnt++;
}
