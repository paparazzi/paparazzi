/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 * @file modules/datalink/datalink_rc.c
 *
 * Radio control input via datalink for split AP/FBW aputopilots
 * such as PX4
 */

#include "modules/datalink/datalink_rc.h"
#include "subsystems/datalink/datalink.h"
#include "pprzlink/messages.h"
#include "subsystems/intermcu/intermcu_ap.h"
#include "subsystems/intermcu.h"
#include "pprzlink/intermcu_msg.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/autopilot.h"

#include <string.h> // memset

static void rc_datalink_normalize(int8_t *in, int16_t *out);

int8_t rc_dl_values[ RC_DL_NB_CHANNEL];
int16_t rc_radio_values[ RC_DL_NB_CHANNEL];

void datalink_rc_init(void)
{
  // clear the buffer
  memset(rc_dl_values, 0, sizeof(rc_dl_values));
}


/**
 * Datalink Message definition:
 *   <message name="RC_5CH" id="53" link="broadcasted">
 *     <field name="ac_id"       type="uint8"/>
 *     <field name="throttle"    type="uint8"/>
 *     <field name="roll"        type="int8"/>
 *     <field name="pitch"       type="int8"/>
 *     <field name="yaw"         type="int8"/>
 *     <field name="mode"        type="int8"/>
 *     <field name="kill"        type="int8"/>
 *   </message>
 *
 * Intermcu message definiton:
 *   <message name="IMCU_RADIO_COMMANDS" id="2">
 *     <field name="status" type="uint8"/>
 *     <field name="values" type="int16[]"/>
 *   </message>
 *
 * We need to parse and repack the radio message,
 * assuming the radio config file and the joystick
 * configuration is the same
 */
void dl_parse_datalink_rc_5ch(void)
{

  rc_dl_values[RADIO_THROTTLE] = DL_RC_5CH_throttle(dl_buffer);
  rc_dl_values[RADIO_ROLL] = DL_RC_5CH_roll(dl_buffer);
  rc_dl_values[RADIO_PITCH] = DL_RC_5CH_pitch(dl_buffer);
  rc_dl_values[RADIO_YAW] = DL_RC_5CH_yaw(dl_buffer);
  rc_dl_values[RADIO_MODE] = DL_RC_5CH_mode(dl_buffer);
  rc_dl_values[RADIO_KILL_SWITCH] = DL_RC_5CH_kill(dl_buffer);

  // normalize the values
  rc_datalink_normalize(rc_dl_values, rc_radio_values);

  // send intermcu message immediately after receiving the datalink packet
  intermcu_send_datalink_rc();
}

/* Forward the datalink-rc telemetry message to be parsed in FBW */
void intermcu_send_datalink_rc(void)
{
  /*
  if (intermcu.enabled) {
    uint8_t dummy_status = 1;
    pprz_msg_send_IMCU_RADIO_COMMANDS(&(intermcu.transport.trans_tx), intermcu.device,
        INTERMCU_AP, &dummy_status, RC_DL_NB_CHANNEL, rc_radio_values);
  }
  */
  for (uint8_t i = 0; i < RC_DL_NB_CHANNEL; i++) {
    radio_control.values[i] = rc_radio_values[i];
  }
  radio_control.frame_cpt++;
  radio_control.time_since_last_frame = 0;
  radio_control.status = RC_OK;
  autopilot_on_rc_frame();
}

/**
 * Normalize rc_dl_values to radio values.
 */
static void rc_datalink_normalize(int8_t *in, int16_t *out)
{
  out[RADIO_ROLL] = (MAX_PPRZ / 128) * in[RADIO_ROLL];
  Bound(out[RADIO_ROLL], MIN_PPRZ, MAX_PPRZ);
  out[RADIO_PITCH] = (MAX_PPRZ / 128) * in[RADIO_PITCH];
  Bound(out[RADIO_PITCH], MIN_PPRZ, MAX_PPRZ);
  out[RADIO_YAW] = (MAX_PPRZ / 128) * in[RADIO_YAW];
  Bound(out[RADIO_YAW], MIN_PPRZ, MAX_PPRZ);
  out[RADIO_THROTTLE] = ((MAX_PPRZ / 128) * in[RADIO_THROTTLE]);
  Bound(out[RADIO_THROTTLE], 0, MAX_PPRZ);
  out[RADIO_MODE] = ((MAX_PPRZ / 128) * in[RADIO_MODE]);
  Bound(out[RADIO_MODE], MIN_PPRZ, MAX_PPRZ);
  out[RADIO_KILL_SWITCH] = (MAX_PPRZ / 128) * in[RADIO_KILL_SWITCH];
  Bound(out[RADIO_KILL_SWITCH], MIN_PPRZ, MAX_PPRZ);
}

