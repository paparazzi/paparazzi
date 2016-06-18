/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/sensors/rpm_sensor.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * RPM sensor based on time difference between pulses
 */

#include "modules/sensors/rpm_sensor.h"
#include "mcu_periph/pwm_input.h"
#include "subsystems/electrical.h"

#ifndef USE_PWM_INPUT
#error rpm sensor requires module pwm_meas.xml
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void rpm_sensor_send_motor(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t rpm = rpm_sensor_get_rpm();

  pprz_msg_send_MOTOR(trans, dev, AC_ID, &rpm, &electrical.current);
}
#endif

/* Initialize the RPM measurement by configuring the telemetry */
void rpm_sensor_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MOTOR, rpm_sensor_send_motor);
#endif
}

uint16_t rpm_sensor_get_rpm(void)
{
  uint16_t rpm = 0;
  uint32_t period_us = get_pwm_input_period_in_usec(RPM_PWM_CHANNEL) * RPM_PULSE_PER_RND;
  if (period_us > 0) {
    rpm = ((uint32_t)1000000 * 60) / period_us;
  }

  return rpm;
}
