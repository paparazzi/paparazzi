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
 * @file "modules/multi/rssi.h"
 * @author Kirk Scheper
 * stores received rssi values for communication protocols that support it
 */

#ifndef RSSI_H
#define RSSI_H

#include <inttypes.h>

struct rssi_info_ {
  uint8_t ac_id;
  int8_t rssi;
  int8_t tx_strength;
};

extern uint8_t rssi_acs_idx;
extern uint8_t rssi_acs_id[];
extern struct rssi_info_ rssi_acs[];

extern void rssi_init(void);
extern void set_rssi(uint8_t _ac_id, int8_t _rssi, int8_t _tx_strength);
extern struct rssi_info_  get_rssi(uint8_t _ac_id);

extern void parse_rssi_dl(void);

#endif
