/*
 * Copyright (C) 2017 The Paparazzi team
 *                    based on code from the Ardupilot project
 *
 * This file is part of paparazzi.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ins/imu_temp_ctrl.c
 *
 * INS temperature control on pwm 6 for Bebop2, pwm 10 for DISCO
 *
 * Controls the heating resistors in the Bebop2 to keep the MPU6050
 * gyro/accel INS sensors at a constant temperature
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "imu_temp_ctrl.h"

uint8_t imu_temp_ctrl_ok = 0;
int pwm_heat_duty_fd = 0;

#ifdef BEBOP_VERSION2
#define PWM_HEAT_CHAN PWM_HEAT_CHAN_BEBOP2
#elif BOARD==disco
#define PWM_HEAT_CHAN PWM_HEAT_CHAN_DISCO
#else
#error temp control only implemented in Parrot Bebop2 and Disco
#endif

void imu_temp_ctrl_periodic(void)
{
  float temp_current, error;
  static float sum_error = 0;
  uint32_t output = 0;

  temp_current = imu_bebop.mpu.temp;

  if (imu_temp_ctrl_ok == 1) {
    /* minimal PI algo without dt from Ardupilot */
    error = (float) ((IMU_TEMP_TARGET) - temp_current);

    /* Don't accumulate errors if the integrated error is superior
     * to the max duty cycle(pwm_period)
     */
    if ((fabsf(sum_error) * IMU_TEMP_CTRL_KI < IMU_TEMP_CTRL_DUTY_MAX)) {
      sum_error = sum_error + error;
    }

    output = IMU_TEMP_CTRL_KP * error + IMU_TEMP_CTRL_KI * sum_error;

    if (output > IMU_TEMP_CTRL_DUTY_MAX) {
      output = IMU_TEMP_CTRL_DUTY_MAX;
    } else if (output < 0) {
      output = 0;
    }

    if (dprintf(pwm_heat_duty_fd, "%u", output) < 0) {
      printf("[temp-ctrl] could not set duty cycle\n");
    }
  }

#ifdef IMU_TEMP_CTRL_DEBUG
  uint16_t duty_cycle;
  duty_cycle = (uint16_t) ((uint32_t) output / (IMU_TEMP_CTRL_DUTY_MAX/100));

  RunOnceEvery(IMU_TEMP_CTRL_PERIODIC_FREQ, DOWNLINK_SEND_TMP_STATUS(DefaultChannel, DefaultDevice, &duty_cycle, &temp_current));
#endif
}

void imu_temp_ctrl_init(void)
{
  int pwm_heat_run_fd, ret;
  char* path;

  ret = asprintf(&path, "/sys/class/pwm/pwm_%u/run", PWM_HEAT_CHAN);
  if (ret < 0) {
    printf("[temp-ctrl] could not create pwm path\n");
    return;
  }

  pwm_heat_run_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  free(path);
  if (pwm_heat_run_fd < 0) {
    printf("[temp-ctrl] could not open pwm run device\n");
    return;
  }

  ret = asprintf(&path, "/sys/class/pwm/pwm_%u/duty_ns", PWM_HEAT_CHAN);
  if (ret < 0) {
    printf("[temp-ctrl] could not create pwm duty path\n");
    close(pwm_heat_run_fd);
    return;
  }

  pwm_heat_duty_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  free(path);
  if (pwm_heat_duty_fd < 0) {
    printf("[temp-ctrl] could not open pwm duty path\n");
    close(pwm_heat_run_fd);
    return;
  }

  ret = write(pwm_heat_duty_fd, "0", 1);
  if (ret != 1) {
    printf("[temp-ctrl] could not set duty cycle\n");
    goto error;
  }

  ret = write(pwm_heat_run_fd, "0", 1);
  if (ret != 1) {
    printf("[temp-ctrl] could not disable pwm\n");
    goto error;
  }

  ret = write(pwm_heat_run_fd, "1", 1);
  if (ret != 1) {
    printf("[temp-ctrl] could not enable pwm\n");
    goto error;
  }

  imu_temp_ctrl_ok = 1;
  return;

error:
    close(pwm_heat_run_fd);
    close(pwm_heat_duty_fd);
}
