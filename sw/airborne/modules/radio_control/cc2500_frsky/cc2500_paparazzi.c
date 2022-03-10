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

#include "modules/radio_control/radio_control.h"
#include "modules/core/abi.h"
#include "peripherals/cc2500.h"
#include "cc2500_common.h"
#include "cc2500_frsky_common.h"
#include "cc2500_settings.h"
#include "cc2500_rx.h"

#include <stdint.h>


static uint16_t frsky_raw[RADIO_CTL_NB];


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_cc2500_ppm(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_PPM(trans, dev, AC_ID,
      &radio_control.frame_rate,
      (sizeof(frsky_raw) / sizeof(frsky_raw[0])),
      frsky_raw);
}
#endif


void radio_control_cc2500_init(void) {
  cc2500_settings_init();
  cc2500_init();
  cc2500Reset();
  rxInit();
  radio_control.nb_channel = RADIO_CTL_NB;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PPM, send_cc2500_ppm);
#endif
}


void radio_control_cc2500_event(void) {
  if (rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig) & RX_FRAME_COMPLETE) {
    rxRuntimeConfig.rcProcessFrameFn(&rxRuntimeConfig);
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    if (radio_control.radio_ok_cpt > 0) {
      radio_control.radio_ok_cpt--;
    } else {
      radio_control.status = RC_OK;
      for (int i = 0; i < RADIO_CTL_NB; ++i) {
        frsky_raw[i] = rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, i);
      }
      NormalizePpmIIR(frsky_raw, radio_control);
      AbiSendMsgRADIO_CONTROL(RADIO_CONTROL_FRSKY_ID, &radio_control);
    }
  }
}

