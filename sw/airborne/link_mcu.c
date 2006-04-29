/*
 * $Id$
 *  
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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

#include "link_mcu.h"
#include "spi.h"
#include "crc_hw.h"

#define CRC_INIT 0xffff
#define CrcUpdate(_crc, _data) _crc_ccitt_update(_crc, _data)
#define Crc1(x) ((x)&0xff)
#define Crc2(x) ((x)>>8)


volatile uint8_t link_mcu_tx_idx;
volatile uint8_t link_mcu_rx_idx;

struct link_mcu_msg link_mcu_from_ap_msg;
struct link_mcu_msg link_mcu_from_fbw_msg;

#ifdef FBW

volatile bool_t link_mcu_is_busy;
volatile bool_t link_mcu_was_busy;

void link_mcu_restart(void) {
  LED_TOGGLE(2);
  link_mcu_rx_idx = 0;
  link_mcu_tx_idx = 0;
  LINK_MCU_TRANSMIT();
  link_mcu_is_busy = TRUE;
}

#endif /* FBW */

#ifdef AP

volatile uint8_t link_fbw_nb_err;
uint8_t link_fbw_fbw_nb_err;

static uint16_t crc_in, crc_out;

void link_fbw_init(void) {
  link_fbw_nb_err = 0;
  
  uint8_t i;
  for (i=0; i<sizeof(struct link_mcu_msg); i++)
    ((uint8_t*)&link_mcu_from_ap_msg)[i] = i;
}

void link_fbw_send(void) {

  if (!SpiCheckAvailable()) {
    SpiOverRun();
    return;
  }

  //  from_fbw_receive_valid = FALSE;
  link_mcu_rx_idx = 0;
  link_mcu_tx_idx = 0;
  /* compute checksum */
  SpiStart();
  SpiSelectSlave0();
  LinkMcuStart();
}

#endif /* AP */
