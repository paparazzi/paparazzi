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

/** @file subsystems/radio_control/sbus.c
 *
 * Single SBUS radio_control
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/sbus.h"
#include BOARD_CONFIG


/** SBUS struct */
struct Sbus sbus;

// Telemetry function
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_sbus(struct transport_tx *trans, struct link_device *dev)
{
  // Using PPM message
  pprz_msg_send_PPM(trans, dev, AC_ID,
                    &radio_control.frame_rate, SBUS_NB_CHANNEL, sbus.ppm);
}
#endif

// Init function
void radio_control_impl_init(void)
{
  sbus_common_init(&sbus, &SBUS_UART_DEV);

  // Register telemetry message
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PPM, send_sbus);
#endif
}


// Decoding event function
// Reading from UART
static inline void sbus_decode_event(void)
{
  sbus_common_decode_event(&sbus, &SBUS_UART_DEV);
}

void radio_control_impl_event(void (* _received_frame_handler)(void))
{
  sbus_decode_event();
  if (sbus.frame_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    if (radio_control.radio_ok_cpt > 0) {
      radio_control.radio_ok_cpt--;
    } else {
      radio_control.status = RC_OK;
      NormalizePpmIIR(sbus.pulses, radio_control);
      _received_frame_handler();
    }
    sbus.frame_available = false;
  }
}
