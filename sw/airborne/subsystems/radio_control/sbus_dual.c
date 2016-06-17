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
 * @file subsystems/radio_control/sbus_dual.c
 *
 * Dual SBUS radio_control
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/sbus_dual.h"
#include BOARD_CONFIG
#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"
#include <string.h>


/** SBUS struct */
struct Sbus sbus1, sbus2;

// Telemetry function
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_sbus(struct transport_tx *trans, struct link_device *dev)
{
  // Using PPM message
  pprz_msg_send_PPM(trans, dev, AC_ID,
                    &radio_control.frame_rate, SBUS_NB_CHANNEL, sbus1.ppm);
}
#endif

// Init function
void radio_control_impl_init(void)
{
  sbus_common_init(&sbus1, &SBUS1_UART_DEV);
  sbus_common_init(&sbus2, &SBUS2_UART_DEV);

  // Register telemetry message
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PPM, send_sbus);
#endif
}

static inline void sbus_dual_decode_event(void)
{
  sbus_common_decode_event(&sbus1, &SBUS1_UART_DEV);
  sbus_common_decode_event(&sbus2, &SBUS2_UART_DEV);
}

void radio_control_impl_event(void (* _received_frame_handler)(void))
{
  sbus_dual_decode_event();
  if (sbus2.frame_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    if (radio_control.radio_ok_cpt > 0) {
      radio_control.radio_ok_cpt--;
    } else {
      radio_control.status = RC_OK;
      NormalizePpmIIR(sbus2.pulses, radio_control);
      _received_frame_handler();
    }
    sbus2.frame_available = false;
  }
  if (sbus1.frame_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    if (radio_control.radio_ok_cpt > 0) {
      radio_control.radio_ok_cpt--;
    } else {
      radio_control.status = RC_OK;
      NormalizePpmIIR(sbus1.pulses, radio_control);
      _received_frame_handler();
    }
    sbus1.frame_available = false;
  }
}
