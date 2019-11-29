/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
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

#include "cc2500_paparazzi.h"

#include "subsystems/radio_control.h"
#include "peripherals/cc2500.h"
#include "cc2500_common.h"
#include "cc2500_frsky_common.h"
#include "cc2500_settings.h"
#include "cc2500_rx.h"

#include <stdint.h>

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/downlink.h"
#endif

//#define RX_SPI_MAX_PAYLOAD_SIZE 35
//static uint8_t rxSpiPayload[RX_SPI_MAX_PAYLOAD_SIZE];

static uint32_t reset_value = 0;
static uint32_t spiinit_result = 0;
static uint32_t status = 0;
static uint32_t counter = 0;

static uint32_t rc_raw[4];

static uint16_t frsky_raw[RADIO_CTL_NB];

void radio_control_impl_init(void) {
  cc2500_settings_init();
  cc2500_init();
  reset_value = cc2500Reset();
//  spiinit_result = cc2500SpiInit();
//  spiinit_result = frSkySpiInit(rxSpiConfig(), rxRuntimeState());
//  spiinit_result = rxSpiInit(rxSpiConfig(), &rxRuntimeState);
  rxInit();
  spiinit_result = 255;
}

void radio_control_impl_event(void (* _received_frame_handler)(void)) {
  if (rxRuntimeState.rcFrameStatusFn(&rxRuntimeState) & RX_FRAME_COMPLETE) {
    rxRuntimeState.rcProcessFrameFn(&rxRuntimeState);
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    if (radio_control.radio_ok_cpt > 0) {
      radio_control.radio_ok_cpt--;
    } else {
      radio_control.status = RC_OK;
      for (int i = 0; i < RADIO_CONTROL_NB_CHANNEL; ++i) {
        frsky_raw[i] = rxRuntimeState.rcReadRawFn(&rxRuntimeState, i);
      }
      NormalizePpmIIR(frsky_raw, radio_control);
      _received_frame_handler();
    }
#if PERIODIC_TELEMETRY
    DOWNLINK_SEND_CC2500(DefaultChannel, DefaultDevice,
        (sizeof(frsky_raw) / sizeof(frsky_raw[0])),
        frsky_raw);
#endif
  }


//  (void) _received_frame_handler;

//  timeUs_t now = micros();
//  static timeUs_t previous = 0;
//  if (previous == 0) { previous = now; }
//  status = rxUpdateCheck(now, now - previous);
//  previous = now;

//  status = rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
//  if (status & RX_FRAME_COMPLETE) {
//    rxRuntimeState.rcProcessFrameFn(&rxRuntimeState);
//    for (int rawChannel = 0; rawChannel < 4; ++rawChannel) {
//      rc_raw[rawChannel] = rxRuntimeState.rcReadRawFn(&rxRuntimeState, rawChannel);
//    }
//  }

//  counter++;
////  if((counter % 100) == 0) {
//    DOWNLINK_SEND_CC2500(DefaultChannel, DefaultDevice,
//        &(rc_raw[0]), &(rc_raw[1]), &(rc_raw[2]), &(rc_raw[3]));
//  }
//  if((counter % 100000) == 0) {
//    static char text[] = "Hello GCS!";
//    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(text), text);
//  }
}
