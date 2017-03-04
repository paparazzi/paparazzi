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

#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

#include "std.h"

void imu_temp_ctrl_init(void);
void imu_temp_ctrl_periodic(void);

#define IMU_TEMP_CTRL_DUTY_MAX 125000

#ifndef IMU_TEMP_TARGET
#define IMU_TEMP_TARGET 50
#endif

#define IMU_TEMP_CTRL_KP 20000
#define IMU_TEMP_CTRL_KI 60

#define PWM_HEAT_CHAN_BEBOP2 6
#define PWM_HEAT_CHAN_DISCO 10

#endif /* IMU_TEMP_CTRL_H */
