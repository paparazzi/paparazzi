/*
 * Copyright (C) Kirk Scheper
 *
 * This file is part of paparazzi
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
 * @file "modules/multi/rssi.c"
 * @author Kirk Scheper
 * stores received rssi values for communication protocols that support it
 */

#include "modules/multi/rssi.h"

#include "subsystems/datalink/datalink.h"
#include "pprzlink/messages.h"

#include "subsystems/abi.h"                 // rssi messages subscription
#include "generated/airframe.h"             // AC_ID

#ifndef NB_ACS_ID
#define NB_ACS_ID 256
#endif
#ifndef NB_ACS
#define NB_ACS 24
#endif

uint8_t rssi_acs_idx;
struct rssi_info_ rssi_acs[NB_ACS];
uint8_t rssi_acs_id[NB_ACS_ID];

abi_event ev;

static void rssi_cb(uint8_t sender_id __attribute__((unused)), uint8_t _ac_id, int8_t _tx_strength, int8_t _rssi)
{
  set_rssi(_ac_id, _tx_strength, _rssi);
}

void rssi_init()
{
  memset(rssi_acs_id, 0, NB_ACS_ID);

  rssi_acs_id[0] = 0;  // ground station
  rssi_acs_id[AC_ID] = 1;
  rssi_acs[rssi_acs_id[AC_ID]].ac_id = AC_ID;
  rssi_acs_idx = 2;

  /* register for rssi messages */
  AbiBindMsgRSSI(ABI_BROADCAST, &ev, rssi_cb);
}

void parse_rssi_dl(uint8_t *buf)
{
  uint8_t sender_id = SenderIdOfPprzMsg(buf);
  uint8_t msg_id = IdOfPprzMsg(buf);

  if (sender_id > 0 && msg_id == DL_RSSI) {
    set_rssi(sender_id,
             DL_RSSI_tx_power(buf),
             DL_RSSI_rssi(buf));
  }
}

void set_rssi(uint8_t _ac_id, int8_t _tx_strength, int8_t _rssi)
{
  if (rssi_acs_idx < NB_ACS) {
    if (_ac_id > 0 && rssi_acs_id[_ac_id] == 0) {
      rssi_acs_id[_ac_id] = rssi_acs_idx++;
      rssi_acs[rssi_acs_id[_ac_id]].ac_id = _ac_id;
    }

    rssi_acs[rssi_acs_id[_ac_id]].rssi = _rssi;
    rssi_acs[rssi_acs_id[_ac_id]].tx_strength = _tx_strength;
  }
}

struct rssi_info_ get_rssi(uint8_t _ac_id)
{
  return rssi_acs[rssi_acs_id[_ac_id]];
}
