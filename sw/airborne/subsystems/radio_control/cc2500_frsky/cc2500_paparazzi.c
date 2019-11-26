/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This code is based on the betaflight cc2500 and FrskyX implementation.
 * https://github.com/betaflight/betaflight
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

#include "subsystems/datalink/downlink.h"

#include <stdint.h>

//#define RX_SPI_MAX_PAYLOAD_SIZE 35
//static uint8_t rxSpiPayload[RX_SPI_MAX_PAYLOAD_SIZE];

static uint32_t reset_value = 0;
static uint32_t spiinit_result = 0;
static uint32_t status = 0;
static uint32_t counter = 0;

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
  (void) _received_frame_handler;

  timeUs_t now = micros();
  static timeUs_t previous = 0;
  if (previous == 0) { previous = now; }
  status = rxUpdateCheck(now, now - previous);
  previous = now;

//  status = frSkySpiDataReceived(rxSpiPayload);
//  if (status) {
//    frSkySpiProcessFrame(rxSpiPayload); // ???
//  }

  counter++;
  if((counter % 10000) == 0) {
    DOWNLINK_SEND_CC2500(DefaultChannel, DefaultDevice,
        &reset_value, &spiinit_result, &status, &counter);
  }
  if((counter % 100000) == 0) {
    static char text[] = "Hello GCS!";
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(text), text);
  }
}
