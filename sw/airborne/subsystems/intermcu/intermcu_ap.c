/*
 * Copyright (C) 2015 The Paparazzi Team
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
 *
 */

/** @file subsystems/intermcu/intermcu_ap.c
 *  @brief Rotorcraft Inter-MCU on the autopilot
 */

#include "intermcu_ap.h"
#include "intermcu_msg.h"
#include "subsystems/radio_control.h"
#include "subsystems/datalink/pprz_transport.h"
#include "mcu_periph/uart.h"

#if COMMANDS_NB > 8
#error "INTERMCU UART CAN ONLY SEND 8 COMMANDS OR THE UART WILL BE OVERFILLED"
#endif

// Used for communication
static struct link_device *intermcu_device = (&((INTERMCU_LINK).device));
static struct pprz_transport intermcu_transport;

struct intermcu_t inter_mcu;
static inline void intermcu_parse_msg(struct transport_rx *trans, void (*rc_frame_handler)(void));

void intermcu_init(void)
{
  pprz_transport_init(&intermcu_transport);
}

void intermcu_periodic(void)
{
  /* Check for interMCU loss */
  if (inter_mcu.time_since_last_frame >= INTERMCU_LOST_CNT) {
    inter_mcu.status = INTERMCU_LOST;
  } else {
    inter_mcu.time_since_last_frame++;
  }
}

void intermcu_set_actuators(pprz_t *command_values, uint8_t ap_mode __attribute__((unused)))
{
  pprz_msg_send_IMCU_COMMANDS(&(intermcu_transport.trans_tx), intermcu_device,
                              INTERMCU_AP, 0, COMMANDS_NB, command_values); //TODO: Fix status
}

static inline void intermcu_parse_msg(struct transport_rx *trans, void (*rc_frame_handler)(void))
{
  /* Parse the Inter MCU message */
  uint8_t msg_id = trans->payload[1];
  switch (msg_id) {
    case DL_IMCU_RADIO_COMMANDS: {
      uint8_t i;
      uint8_t size = DL_IMCU_RADIO_COMMANDS_values_length(trans->payload);
      int16_t *rc_values = DL_IMCU_RADIO_COMMANDS_values(trans->payload);
      for (i = 0; i < size; i++) {
        radio_control.values[i] = rc_values[i];
      }

      radio_control.frame_cpt++;
      radio_control.time_since_last_frame = 0;
      radio_control.status = RC_OK;
      rc_frame_handler();
      break;
    }

    default:
      break;
  }

  // Set to receive another message
  trans->msg_received = FALSE;
}

void RadioControlEvent(void (*frame_handler)(void))
{
  /* Parse incoming bytes */
  if (intermcu_device->char_available(intermcu_device->periph)) {
    while (intermcu_device->char_available(intermcu_device->periph) && !intermcu_transport.trans_rx.msg_received) {
      parse_pprz(&intermcu_transport, intermcu_device->get_byte(intermcu_device->periph));
    }

    if (intermcu_transport.trans_rx.msg_received) {
      intermcu_parse_msg(&(intermcu_transport.trans_rx), frame_handler);
    }
  }
}
