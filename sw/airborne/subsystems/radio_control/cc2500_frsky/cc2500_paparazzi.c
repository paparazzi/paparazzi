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


static uint16_t frsky_raw[RADIO_CTL_NB];


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_cc2500(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CC2500(trans, dev, AC_ID,
      (sizeof(frsky_raw) / sizeof(frsky_raw[0])),
      frsky_raw);
}
#endif


void radio_control_impl_init(void) {
  cc2500_settings_init();
  cc2500_init();
  cc2500Reset();
  rxInit();
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CC2500, send_cc2500);
#endif
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
  }
}
