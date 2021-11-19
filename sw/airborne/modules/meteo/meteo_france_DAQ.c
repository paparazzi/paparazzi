/*
 * Copyright (C) 2014 Gautier Hattenberger
 *
 * This file is part of paparazzi

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
 *
 */

/**
 * @file modules/meteo/meteo_france_DAQ.c
 *
 * Communication module with the Data Acquisition board
 * from Meteo France
 *
 * DAQ board sends measurments to the AP
 * AP sends periodic report to the ground, store data on SD card
 * and sends A/C state to DAQ board
 */

#include "modules/meteo/meteo_france_DAQ.h"

#include "state.h"
#include "autopilot.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/downlink.h"
#include "modules/loggers/sdlog_chibios.h"
#include "modules/loggers/pprzlog_tp.h"

#include "modules/gps/gps.h"
#include "modules/datalink/extra_pprz_dl.h"

struct MF_DAQ mf_daq;
bool log_started;

#ifndef MF_DAQ_POWER_INIT
#define MF_DAQ_POWER_INIT TRUE
#endif

#if !(defined MF_DAQ_POWER_PORT) && !(defined MF_DAQ_POWER_PIN)
INFO("MF_DAQ power pin is not defined")
#endif

void init_mf_daq(void)
{
  mf_daq.nb = 0;
  mf_daq.power = MF_DAQ_POWER_INIT;
#if (defined MF_DAQ_POWER_PORT) && (defined MF_DAQ_POWER_PIN)
  gpio_setup_output(MF_DAQ_POWER_PORT, MF_DAQ_POWER_PIN);
#endif
  meteo_france_DAQ_SetPower(mf_daq.power);
  log_started = false;
}

void mf_daq_send_state(void)
{
  // Send aircraft state to DAQ board
  DOWNLINK_SEND_MF_DAQ_STATE(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
                             &autopilot.flight_time,
                             &stateGetBodyRates_f()->p,
                             &stateGetBodyRates_f()->q,
                             &stateGetBodyRates_f()->r,
                             &stateGetNedToBodyEulers_f()->phi,
                             &stateGetNedToBodyEulers_f()->theta,
                             &stateGetNedToBodyEulers_f()->psi,
                             &stateGetAccelNed_f()->x,
                             &stateGetAccelNed_f()->y,
                             &stateGetAccelNed_f()->z,
                             &stateGetSpeedEnu_f()->x,
                             &stateGetSpeedEnu_f()->y,
                             &stateGetSpeedEnu_f()->z,
                             &stateGetPositionLla_f()->lat,
                             &stateGetPositionLla_f()->lon,
                             &stateGetPositionLla_f()->alt,
                             &stateGetHorizontalWindspeed_f()->y,
                             &stateGetHorizontalWindspeed_f()->x);
}

void mf_daq_send_report(void)
{
  // Send report over normal telemetry
  if (mf_daq.nb > 0) {
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 9, mf_daq.values);
  }
  // Test if log is started
  if (pprzLogFile != -1) {
    if (log_started == FALSE) {
      // Log MD5SUM once
      DOWNLINK_SEND_ALIVE(pprzlog_tp, chibios_sdlog, 16, MD5SUM);
      log_started = true;
    }
    // Log GPS for time reference
    uint8_t foo = 0;
    int16_t climb = -gps.ned_vel.z;
    int16_t course = (DegOfRad(gps.course) / ((int32_t)1e6));
    struct UtmCoor_f utm = *stateGetPositionUtm_f();
    int32_t east = utm.east * 100;
    int32_t north = utm.north * 100;
    DOWNLINK_SEND_GPS(pprzlog_tp, chibios_sdlog, &gps.fix,
                      &east, &north, &course, &gps.hmsl, &gps.gspeed, &climb,
                      &gps.week, &gps.tow, &utm.zone, &foo);
  }
}

void parse_mf_daq_msg(uint8_t *buf)
{
  mf_daq.nb = buf[2];
  if (mf_daq.nb > 0) {
    if (mf_daq.nb > MF_DAQ_SIZE) { mf_daq.nb = MF_DAQ_SIZE; }
    // Store data struct directly from dl_buffer
    float *bufloc = (float*)(buf+3);
    memcpy(mf_daq.values, bufloc, mf_daq.nb * sizeof(float));
    // Log on SD card
    if (log_started) {
      DOWNLINK_SEND_PAYLOAD_FLOAT(pprzlog_tp, chibios_sdlog, mf_daq.nb, mf_daq.values);
      DOWNLINK_SEND_MF_DAQ_STATE(pprzlog_tp, chibios_sdlog,
                                 &autopilot.flight_time,
                                 &stateGetBodyRates_f()->p,
                                 &stateGetBodyRates_f()->q,
                                 &stateGetBodyRates_f()->r,
                                 &stateGetNedToBodyEulers_f()->phi,
                                 &stateGetNedToBodyEulers_f()->theta,
                                 &stateGetNedToBodyEulers_f()->psi,
                                 &stateGetAccelNed_f()->x,
                                 &stateGetAccelNed_f()->y,
                                 &stateGetAccelNed_f()->z,
                                 &stateGetSpeedEnu_f()->x,
                                 &stateGetSpeedEnu_f()->y,
                                 &stateGetSpeedEnu_f()->z,
                                 &stateGetPositionLla_f()->lat,
                                 &stateGetPositionLla_f()->lon,
                                 &stateGetPositionLla_f()->alt,
                                 &stateGetHorizontalWindspeed_f()->y,
                                 &stateGetHorizontalWindspeed_f()->x);
    }
  }
}


