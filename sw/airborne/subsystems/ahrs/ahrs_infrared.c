/*
 * Copyright (C) 2011 The Paparazzi Team
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
 * @file subsystems/ahrs/ahrs_infrared.c
 *
 * Attitude estimation using infrared sensors detecting the horizon.
 * For fixedwings only:
 * - GPS course is used as heading.
 * - ADC channels can be used for gyros.
 *
 */

#include "subsystems/ahrs/ahrs_infrared.h"

#include "subsystems/sensors/infrared.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "state.h"

float heading;

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

static void send_infrared(void) {
  DOWNLINK_SEND_IR_SENSORS(DefaultChannel, DefaultDevice,
      &infrared.value.ir1, &infrared.value.ir2, &infrared.pitch, &infrared.roll, &infrared.top);
}

static void send_status(void) {
  uint16_t contrast = abs(infrared.roll) + abs(infrared.pitch) + abs(infrared.top);
  uint8_t mde = 3;
  if (contrast < 50) mde = 7;
  DOWNLINK_SEND_STATE_FILTER_STATUS(DefaultChannel, DefaultDevice, &mde, &contrast);
}
#endif

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;

  heading = 0.;

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "IR_SENSORS", send_infrared);
  register_periodic_telemetry(DefaultPeriodic, "STATE_FILTER_STATUS", send_status);
#endif
}

void ahrs_align(void) {

  //TODO set gyro bias if used

  ahrs.status = AHRS_RUNNING;
}

void ahrs_propagate(void) {
  struct FloatRates body_rate = { 0., 0., 0. };
#ifdef ADC_CHANNEL_GYRO_P
  body_rate.p = RATE_FLOAT_OF_BFP(imu.gyro.p);
#endif
#ifdef ADC_CHANNEL_GYRO_Q
  body_rate.q = RATE_FLOAT_OF_BFP(imu.gyro.q);
#endif
#ifdef ADC_CHANNEL_GYRO_R
  body_rate.r = RATE_FLOAT_OF_BFP(imu.gyro.r);
#endif
  stateSetBodyRates_f(&body_rate);
}

void ahrs_update_accel(void) {
}

void ahrs_update_mag(void) {
}

void ahrs_update_gps(void) {

  float hspeed_mod_f = gps.gspeed / 100.;
  float course_f = gps.course / 1e7;

  // Heading estimator from wind-information, usually computed with -DWIND_INFO
  // wind_north and wind_east initialized to 0, so still correct if not updated
  float w_vn = cosf(course_f) * hspeed_mod_f - stateGetHorizontalWindspeed_f()->x;
  float w_ve = sinf(course_f) * hspeed_mod_f - stateGetHorizontalWindspeed_f()->y;
  heading = atan2f(w_ve, w_vn);
  if (heading < 0.)
    heading += 2 * M_PI;

}

void ahrs_update_infrared(void) {
  float phi  = atan2(infrared.roll, infrared.top) - infrared.roll_neutral;
  float theta  = atan2(infrared.pitch, infrared.top) - infrared.pitch_neutral;

  if (theta < -M_PI_2) theta += M_PI;
  else if (theta > M_PI_2) theta -= M_PI;

  if (phi >= 0) phi *= infrared.correction_right;
  else phi *= infrared.correction_left;

  if (theta >= 0) theta *= infrared.correction_up;
  else theta *= infrared.correction_down;

  struct FloatEulers att = { phi, theta, heading };
  stateSetNedToBodyEulers_f(&att);

}

