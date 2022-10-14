/*
 * Copyright (C) 2022 Freek van tieen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/imu/imu_heater.h
 * IMU heater module which can actuate a resistor heater through GPIO or IOMCU.
 */

#ifndef IMU_HEATER_H
#define IMU_HEATER_H

#include "std.h"

/** Main IMU heater structure */
struct imu_heater_t {
  float target_temp;        ///< Target temeperature in degrees Celcius
  float meas_temp;          ///< Measered average temperature in degrees Celcius
  float meas_temp_sum;      ///< Summed temperature in degrees Celcius
  uint32_t meas_temp_cnt;   ///< Amount of summed temperatures
  float gain_p;             ///< Heater kP gain
  float gain_i;             ///< Heater kI gain
  float integrated;         ///< Integrated temperature error multiplied by kI (max 70%)
  uint32_t last_ts;         ///< Last integration timestamp
  uint8_t heat_cmd;         ///< Heater command 0-100%
};
extern struct imu_heater_t imu_heater;

/** Main external functions */
void imu_heater_init(void);
void imu_heater_periodic(void);
void imu_heater_periodic_10hz(void);

#endif /* IMU_HEATER_H */
