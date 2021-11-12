/*
 * Copyright (C) 2010-2012 The Paparazzi Team
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

#include "link_mcu_can.h"
#include "mcu_periph/can.h"
#include "led.h"

#include "modules/core/commands.h"


//////////////////////////////////////////////////////////////////////////////////////////////
// INTERMCU CAN MESSAGES

// Commands
#define MSG_INTERMCU_COMMAND_MASTER_ID 0x01
#define MSG_INTERMCU_COMMAND_EXTRA_ID 0x02
// Channels
#define MSG_INTERMCU_RADIO_LOW_ID 0x04
#define MSG_INTERMCU_RADIO_HIGH_ID 0x05
// Trim
#define MSG_INTERMCU_TRIM_ID 0x09
// Status
#define MSG_INTERMCU_FBW_STATUS_ID 0x10


union {
  uint8_t data[8];
  pprz_t cmd[4];
} imcu_cmd_mstr, imcu_cmd_ext, imcu_chan1, imcu_chan2, imcu_trim;

#define INTERMCU_COMMAND(_intermcu_payload, nr) (pprz_t)((uint16_t)(*((uint8_t*)_intermcu_payload+0+(2*(nr)))|*((uint8_t*)_intermcu_payload+1+(2*(nr)))<<8))

//////////////////////////////////////////////////////////////////////////////////////////////
// READ MESSAGES


void link_mcu_on_can_msg(uint32_t id, uint8_t *data, int len);
void link_mcu_on_can_msg(uint32_t id, uint8_t *data, int len)
{
#if COMMANDS_NB > 8
#error "INTERMCU_CAN CAN ONLY SEND 4 OR 8 COMMANDS (packets of 8 bytes)"
#endif
#if RADIO_CONTROL_NB_CHANNEL > 8
#warning "INTERMCU_CAN CAN ONLY SEND 8 RADIO CHANNELS: CHANNELS 9 and higher will not be sent"
#endif
  if (len) {} //Remove compile warning

  if (id == MSG_INTERMCU_COMMAND_MASTER_ID) {
    for (int i = 0; (i < 4) && (i < COMMANDS_NB); i++) {
      ap_state->commands[i] = INTERMCU_COMMAND(data, i);
    }
#ifdef LINK_MCU_LED
    LED_TOGGLE(LINK_MCU_LED);
#endif
    inter_mcu_received_ap = true;
  }

  if (id ==  MSG_INTERMCU_COMMAND_EXTRA_ID) {
    for (int i = 0; (i < 4) && (i < (COMMANDS_NB - 4)); i++) {
      ap_state->commands[4 + i] = INTERMCU_COMMAND(data, i);
    }
  }


  if (id == MSG_INTERMCU_TRIM_ID) {
    ap_state->command_roll_trim  = ((pprz_t) INTERMCU_COMMAND(data, 0));
    ap_state->command_pitch_trim = ((pprz_t) INTERMCU_COMMAND(data, 1));
  }

  if (id == MSG_INTERMCU_RADIO_LOW_ID) {
    for (int i = 0; (i < 4) && (i < RADIO_CONTROL_NB_CHANNEL); i++) {
      fbw_state->channels[i] = ((pprz_t)INTERMCU_COMMAND(data, i));
    }
  }

  if (id == MSG_INTERMCU_RADIO_HIGH_ID) {
    for (int i = 0; (i < 4) && (i < (RADIO_CONTROL_NB_CHANNEL - 4)); i++) {
      fbw_state->channels[4 + i] = ((pprz_t)INTERMCU_COMMAND(data, i));
    }
  }

  if ((id == MSG_INTERMCU_FBW_STATUS_ID) && (len == 5)) {
    fbw_state->ppm_cpt = data[0];
    fbw_state->status = data[1];
    fbw_state->nb_err = data[2];
    fbw_state->electrical.vsupply = (float)(data[3] + (data[4] << 8))/10.f;
    fbw_state->electrical.current = 0;

#ifdef LINK_MCU_LED
    LED_TOGGLE(LINK_MCU_LED);
#endif
    inter_mcu_received_fbw = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// SEND MESSAGES


#ifdef AP
void link_mcu_send(void)
{
  for (int i = 0; (i < COMMANDS_NB) && (i < 4); i++) {
    imcu_cmd_mstr.cmd[i] = ap_state->commands[i];
  }
  for (int i = 0; (i < (COMMANDS_NB - 4)) && (i < 4); i++) {
    imcu_cmd_ext.cmd[i] = ap_state->commands[4 + i];
  }

  ppz_can_transmit(MSG_INTERMCU_COMMAND_MASTER_ID, imcu_cmd_mstr.data, 8);
  ppz_can_transmit(MSG_INTERMCU_COMMAND_EXTRA_ID, imcu_cmd_ext.data, 8);

  imcu_trim.cmd[0] = ap_state->command_roll_trim;
  imcu_trim.cmd[1] = ap_state->command_pitch_trim;
  imcu_trim.cmd[2] = ap_state->command_yaw_trim;
  RunOnceEvery(6, ppz_can_transmit(MSG_INTERMCU_TRIM_ID, imcu_trim.data, 6));
}
#endif

#ifdef FBW
void link_mcu_periodic_task(void)
{
  // Import: Prepare the next message for AP
  inter_mcu_fill_fbw_state();

  // Transmit Status
  uint8_t intermcu_tx_buff[8];
  intermcu_tx_buff[0] = fbw_state->ppm_cpt;
  intermcu_tx_buff[1] = fbw_state->status;
  intermcu_tx_buff[2] = fbw_state->nb_err;
  uint16_t vsupply = fbw_state->electrical.vsupply * 10;
  intermcu_tx_buff[3] = (uint8_t) vsupply;
  intermcu_tx_buff[4] = (uint8_t)((vsupply & 0xff00) >> 8);
  ppz_can_transmit(MSG_INTERMCU_FBW_STATUS_ID, intermcu_tx_buff, 5);

#if defined RADIO_CONTROL || RADIO_CONTROL_AUTO1
  // Copy the CHANNELS to the 2 CAN buffers
  for (int i = 0; (i < RADIO_CONTROL_NB_CHANNEL) && (i < 4); i++) {
    imcu_chan1.cmd[i] = fbw_state->channels[i];
  }
  for (int i = 0; (i < (RADIO_CONTROL_NB_CHANNEL - 4)) && (i < 4); i++) {
    imcu_chan2.cmd[i] = fbw_state->channels[4 + i];
  }

  if (bit_is_set(fbw_state->status, RC_OK)) {
    ppz_can_transmit(MSG_INTERMCU_RADIO_LOW_ID,  imcu_chan1.data, 8);
    ppz_can_transmit(MSG_INTERMCU_RADIO_HIGH_ID, imcu_chan2.data, 8);
  }
#endif
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// STATUS LOGIC AND TELEMETRY
// Downlink FBW status from AP

struct link_mcu_msg link_mcu_from_ap_msg;
struct link_mcu_msg link_mcu_from_fbw_msg;


#ifdef AP
#include "modules/datalink/telemetry.h"

#define RC_OK          0
#define RC_LOST        1
#define RC_REALLY_LOST 2


static void send_commands(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_COMMANDS(trans, dev, AC_ID, COMMANDS_NB, ap_state->commands);
}

static void send_fbw_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t rc_status = 0;
  uint8_t fbw_status = 0;
  if (bit_is_set(fbw_state->status, STATUS_MODE_AUTO)) {
    fbw_status = FBW_MODE_AUTO;
  }
  if (bit_is_set(fbw_state->status, STATUS_MODE_FAILSAFE)) {
    fbw_status = FBW_MODE_FAILSAFE;
  }
  if (bit_is_set(fbw_state->status, STATUS_RADIO_REALLY_LOST)) {
    rc_status = RC_REALLY_LOST;
  } else if (bit_is_set(fbw_state->status, RC_OK)) {
    rc_status = RC_OK;
  } else {
    rc_status = RC_LOST;
  }
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &(rc_status), &(fbw_state->ppm_cpt), &(fbw_status),
                           &(fbw_state->electrical.vsupply), &(fbw_state->electrical.current));
}
#endif

void link_mcu_init(void)
{
  ppz_can_init((can_rx_callback_t)link_mcu_on_can_msg);

#ifdef AP
#if PERIODIC_TELEMETRY
  // If FBW has not telemetry, then AP can send some of the info
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_COMMANDS, send_commands);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FBW_STATUS, send_fbw_status);
#endif
#endif
}

void link_mcu_event_task(void)
{
  // No event function: CAN_RX_IRQ is called on reception of data
}
