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
 * @file modules/ahrs/ahrs_infrared.c
 *
 * Attitude estimation using infrared sensors detecting the horizon.
 * For fixedwings only:
 * - GPS course is used as heading.
 * - ADC channels can be used for gyros.
 *
 */

#include "modules/ahrs/ahrs_infrared.h"

#include "subsystems/sensors/infrared.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "state.h"
#include "subsystems/abi.h"

#ifndef INFRARED_FILTER_ID
#define INFRARED_FILTER_ID 2
#endif

static float heading;

/** ABI binding for gyro data.
 * Used for gyro ABI messages.
 */
#ifndef AHRS_INFRARED_GYRO_ID
#define AHRS_INFRARED_GYRO_ID ABI_BROADCAST
#endif
static abi_event gyro_ev;

#ifndef AHRS_INFRARED_GPS_ID
#define AHRS_INFRARED_GPS_ID GPS_MULTI_ID
#endif
static abi_event gps_ev;
void ahrs_infrared_update_gps(struct GpsState *gps_s);

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
  stateSetBodyRates_i(gyro);
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ahrs_infrared_update_gps(gps_s);
}


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_infrared(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IR_SENSORS(trans, dev, AC_ID,
                           &infrared.value.ir1, &infrared.value.ir2, &infrared.pitch, &infrared.roll, &infrared.top);
}

static void send_status(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t contrast = abs(infrared.roll) + abs(infrared.pitch) + abs(infrared.top);
  uint8_t mde = 3;
  uint8_t id = INFRARED_FILTER_ID;
  if (contrast < 50) { mde = 7; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &id, &mde, &contrast);
}
#endif


void ahrs_infrared_init(void)
{
  heading = 0.;

  AbiBindMsgIMU_GYRO_INT32(AHRS_INFRARED_GYRO_ID, &gyro_ev, gyro_cb);
  AbiBindMsgGPS(AHRS_INFRARED_GPS_ID, &gps_ev, &gps_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IR_SENSORS, send_infrared);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_status);
#endif
}


void ahrs_infrared_update_gps(struct GpsState *gps_s)
{
  float hspeed_mod_f = gps_s->gspeed / 100.;
  float course_f = gps_s->course / 1e7;

  // Heading estimator from wind-information, usually computed with -DWIND_INFO
  // wind_north and wind_east initialized to 0, so still correct if not updated
  float w_vn = cosf(course_f) * hspeed_mod_f - stateGetHorizontalWindspeed_f()->x;
  float w_ve = sinf(course_f) * hspeed_mod_f - stateGetHorizontalWindspeed_f()->y;
  heading = atan2f(w_ve, w_vn);
  if (heading < 0.) {
    heading += 2 * M_PI;
  }

}

void ahrs_infrared_periodic(void)
{
  float phi  = atan2(infrared.roll, infrared.top) - infrared.roll_neutral;
  float theta  = atan2(infrared.pitch, infrared.top) - infrared.pitch_neutral;

  if (theta < -M_PI_2) { theta += M_PI; }
  else if (theta > M_PI_2) { theta -= M_PI; }

  if (phi >= 0) { phi *= infrared.correction_right; }
  else { phi *= infrared.correction_left; }

  if (theta >= 0) { theta *= infrared.correction_up; }
  else { theta *= infrared.correction_down; }

  struct FloatEulers att = { phi, theta, heading };
  stateSetNedToBodyEulers_f(&att);
}
