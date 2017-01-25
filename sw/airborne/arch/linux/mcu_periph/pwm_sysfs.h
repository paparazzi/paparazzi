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

/** @file arch/linux/mcu_periph/pwm_sysfs.h
 *  PWM servos handling using Linux sysfs
 *
 *  Based on PWM_Sysfs driver from ardupilot released under GPL v3
 */

#ifndef PWM_SYSFS_H
#define PWM_SYSFS_H

#include "std.h"

#define PWM_SYSFS_PATH_LEN 64

/** PWM structure
 */
struct PWM_Sysfs {
  uint32_t duty_cycle_nsec;               ///< current duty cycle (in nsec)
  uint32_t period_nsec;                   ///< current period (in nsec)
  bool enabled;                           ///< true if pwm is enabled
  int duty_cycle_fd;                      ///< file descriptor to write/update duty cycle
  char enable_path[PWM_SYSFS_PATH_LEN];   ///< path to enable file
  char duty_path[PWM_SYSFS_PATH_LEN];     ///< path to duty file
  char period_path[PWM_SYSFS_PATH_LEN];   ///< path to period file
};

extern int pwm_sysfs_init(struct PWM_Sysfs *pwm, char *base_path,
    char *_export, char *_enable,
    char *_duty, char *_period,
    uint8_t channel);
extern void pwm_sysfs_set_period(struct PWM_Sysfs *pwm, uint32_t period);
extern void pwm_sysfs_set_duty(struct PWM_Sysfs *pwm, uint32_t duty);
extern void pwm_sysfs_enable(struct PWM_Sysfs *pwm, bool enable);

#endif

