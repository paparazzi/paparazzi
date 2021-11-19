/*
 * Copyright (C) 2013 Alexandre Bustico, Gautier Hattenberger
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

/** @file modules/radio_control/hott.c
 *
 * Single HOTT radio_control SUMD
 */

#include "modules/radio_control/radio_control.h"
#include "modules/radio_control/hott.h"
#include BOARD_CONFIG


/** HOTT struct */
struct SHott hott;

// Telemetry function
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_hott(struct transport_tx *trans, struct link_device *dev)
{
  // Using PPM message
  pprz_msg_send_PPM(trans, dev, AC_ID,
                    &radio_control.frame_rate, HOTT_NB_CHANNEL, hott.ppm);
}
#endif

// Init function
void radio_control_impl_init(void)
{
  hott_common_init(&hott, &HOTT_UART_DEV);

  // Register telemetry message
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PPM, send_hott);
#endif
}


// Decoding event function
// Reading from UART
static inline void hott_decode_event(void)
{
  hott_common_decode_event(&hott, &HOTT_UART_DEV);
}

void radio_control_impl_event(void (* _received_frame_handler)(void))
{
  hott_decode_event();
  if (hott.frame_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    if (radio_control.radio_ok_cpt > 0) {
      radio_control.radio_ok_cpt--;
    } else {
      radio_control.status = RC_OK;
      NormalizePpmIIR(hott.pulses, radio_control);
      _received_frame_handler();
    }
    hott.frame_available = false;
  }
}
