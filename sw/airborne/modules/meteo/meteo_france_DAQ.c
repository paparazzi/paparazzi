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
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/chibios-libopencm3/sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"

#include "modules/datalink/extra_pprz_dl.h"

struct MF_DAQ mf_daq;

void init_mf_daq(void) {
  mf_daq.nb = 0;
}

void mf_daq_send_state(void) {
  // Send aircraft state to DAQ board
  DOWNLINK_SEND_MF_DAQ_STATE(PprzTransport, EXTRA_PPRZ_UART,
      &autopilot_flight_time,
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

void mf_daq_send_report(void) {
  // Send report over normal telemetry
  if (mf_daq.nb > 0) {
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, mf_daq.nb, mf_daq.values);
  }
}

void parse_mf_daq_msg(void) {
  mf_daq.nb = DL_PAYLOAD_FLOAT_values_length(dl_buffer);
  if (mf_daq.nb > 0) {
    // Store data struct directly from dl_buffer
    memcpy(mf_daq.values, DL_PAYLOAD_FLOAT_values(dl_buffer), mf_daq.nb * sizeof(float));
    // Log on SD card
    DOWNLINK_SEND_PAYLOAD_FLOAT(PprzLogTransport, SDLOG, mf_daq.nb, mf_daq.values);
  }
}


