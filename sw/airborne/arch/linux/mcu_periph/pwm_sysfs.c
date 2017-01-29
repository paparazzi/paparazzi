/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file arch/linux/mcu_periph/pwm_sysfs.c
 *  PWM servos handling using Linux sysfs
 *
 *  Based on PWM_Sysfs driver from ardupilot released under GPL v3
 */

#include "mcu_periph/pwm_sysfs.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>

// default period in nano seconds (100Hz)
#define PWM_SYSFS_DEFAULT_PERIOD 10000000

// print debug
#if PWM_SYSFS_DEBUG
#define PS_DEBUG_PRINT printf
#else
#define PS_DEBUG_PRINT(...) {}
#endif

// Write in file
static int write_file(const char *path, const char *fmt, ...);
// Read file content
static int read_file(const char *path, const char *fmt, ...) __attribute__((unused));

int pwm_sysfs_init(struct PWM_Sysfs *pwm, char *base_path,
    char *_export, char *_enable,
    char *_duty, char *_period,
    uint8_t channel)
{
  // path to export file (create new PWM)
  char export_path[PWM_SYSFS_PATH_LEN];

  if (base_path == NULL || _export == NULL || _enable == NULL ||
      _period == NULL || _duty == NULL) {
    // invalid paths
    PS_DEBUG_PRINT("invalid paths\n");
    return -1;
  }
  // store paths
  snprintf(export_path, PWM_SYSFS_PATH_LEN, "%s/%s", base_path, _export);
  snprintf(pwm->enable_path, PWM_SYSFS_PATH_LEN, "%s/pwm_%u/%s", base_path, channel, _enable);
  snprintf(pwm->duty_path, PWM_SYSFS_PATH_LEN, "%s/pwm_%u/%s", base_path, channel, _duty);
  snprintf(pwm->period_path, PWM_SYSFS_PATH_LEN, "%s/pwm_%u/%s", base_path, channel, _period);

  PS_DEBUG_PRINT("%s\n%s\n%s\n%s\n",
      export_path, pwm->enable_path, pwm->duty_path, pwm->period_path);

  // export new PWM entry
  write_file(export_path, "%u", channel);
  // open duty cycle file
  pwm->duty_cycle_fd = open(pwm->duty_path, O_RDWR | O_CLOEXEC);
  if (pwm->duty_cycle_fd < 0) {
    // fail to open duty file
    PS_DEBUG_PRINT("failed to open FD cycle: %d\n", pwm->duty_cycle_fd);
    return -2;
  }
  // set default period
  pwm_sysfs_set_period(pwm, PWM_SYSFS_DEFAULT_PERIOD);

  // init with sucess
  return 0;
}

void pwm_sysfs_set_period(struct PWM_Sysfs *pwm, uint32_t period)
{
  if (write_file(pwm->period_path, "%u", period) < 0) {
    PS_DEBUG_PRINT("failed to set period\n");
    return;
  }
  PS_DEBUG_PRINT("setting period %d\n", period);
  pwm->period_nsec = period;
}

void pwm_sysfs_set_duty(struct PWM_Sysfs *pwm, uint32_t duty)
{
  if (dprintf(pwm->duty_cycle_fd, "%u", duty) < 0) {
    PS_DEBUG_PRINT("failed to set duty %u; %d; %s\n", duty, pwm->duty_cycle_fd, pwm->duty_path);
    return;
  }
  pwm->duty_cycle_nsec = duty;
  // enable pwm if not done
  if (!pwm->enabled) {
    pwm_sysfs_enable(pwm, true);
  }
}

void pwm_sysfs_enable(struct PWM_Sysfs *pwm, bool enable)
{
  int en = enable ? 1 : 0;
  int ret = write_file(pwm->enable_path, "%u", en);
  if (ret < 0) {
    PS_DEBUG_PRINT("failed to enable: %d; %s\n", ret, pwm->enable_path);
    return;
  }
  PS_DEBUG_PRINT("enable channel: %d; %s\n", en, pwm->enable_path);
  pwm->enabled = enable;
}


static int write_file(const char *path, const char *fmt, ...)
{
  errno = 0;

  int fd = open(path, O_WRONLY | O_CLOEXEC);
  if (fd == -1) {
    return -errno;
  }

  va_list args;
  va_start(args, fmt);

  int ret = vdprintf(fd, fmt, args);
  int errno_bkp = errno;
  close(fd);

  va_end(args);

  if (ret < 1) {
    return -errno_bkp;
  }

  return ret;
}

static int read_file(const char *path, const char *fmt, ...)
{
  errno = 0;

  FILE *file = fopen(path, "re");
  if (!file) {
    return -errno;
  }

  va_list args;
  va_start(args, fmt);

  int ret = vfscanf(file, fmt, args);
  int errno_bkp = errno;
  fclose(file);

  va_end(args);

  if (ret < 1) {
    return -errno_bkp;
  }

  return ret;
}


