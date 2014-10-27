/*
 * Copyright (C) 2014 Christophe De Wagter
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file qnh.c
 * QNH module.
 *
 */

#include "qnh.h"
#include "state.h"
#include "subsystems/abi.h"
#include "subsystems/sensors/baro.h"
#include "generated/airframe.h"
#include "math/pprz_isa.h"

#ifndef QNH_BARO_ID
#define QNH_BARO_ID ABI_BROADCAST
#endif

struct qnh_struct qnh;
abi_event qnh_baro_event = {0, 0, 0};

void received_abs_baro_for_qnh(uint8_t sender_id, const float * pressure);
void received_abs_baro_for_qnh(__attribute__((__unused__)) uint8_t sender_id, const float * pressure)
{
  qnh.baro_pressure = *pressure;
  const float MeterPerFeet = 0.3048;
  qnh.amsl_baro = pprz_isa_height_of_pressure_full(qnh.baro_pressure, qnh.qnh * 100.0f) /
    MeterPerFeet;
  qnh.baro_counter = 10;
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_amsl(void)
{
  DOWNLINK_SEND_AMSL(DefaultChannel, DefaultDevice, &qnh.amsl_baro, &qnh.amsl_gps);
}
#endif


void init_qnh(void) {
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(&telemetry_Ap, "AMSL", send_amsl);
#endif
  qnh.qnh = 1013.25;
  qnh.amsl_baro = 0;
  qnh.baro_counter = 0;
  qnh.amsl_gps = 0;
  qnh.baro_pressure = 0;
  AbiBindMsgBARO_ABS(QNH_BARO_ID, &qnh_baro_event, &received_abs_baro_for_qnh);
}

void compute_qnh(void)
{
  float h = stateGetPositionLla_f()->alt;
  qnh.qnh = pprz_isa_ref_pressure_of_height_full(qnh.baro_pressure, h) / 100.0f;
}

float GetAmsl(void)
{
  // If baro is OK
  if (qnh.baro_counter > 0)
    return qnh.amsl_baro;
  // Otherwise use GPS
  return qnh.amsl_gps;
}

void periodic_qnh(void)
{
  const float MeterPerFeet = 0.3048;
/*
  Check:
  1200Pa per 100m
  8.333cm per Pa

  float Trel = 1 - L*h/T0;
  float p = qnh * pow(Trel,Expo);
*/
  float h = stateGetPositionLla_f()->alt;
  qnh.amsl_gps = h / MeterPerFeet;

  // Watchdog on baro
  if (qnh.baro_counter > 0)
    qnh.baro_counter--;
  else
    qnh.amsl_baro = 0.0;
}
