/*  $Id$
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

/** \brief handling of avr inter mcu link
 *  
 */



#ifndef LINK_MCU_HW_H
#define LINK_MCU_HW_H

#if (__GNUC__ == 3)
#include <avr/signal.h>
#include <avr/crc16.h>
#else
#include <util/crc16.h>
#endif

#define CRC_INIT 0xffff
#define CrcUpdate(_crc, _data) _crc_ccitt_update(_crc, _data)
#define Crc1(x) ((x)&0xff)
#define Crc2(x) ((x)>>8)

#ifdef FBW
#define TX_BUF ((uint8_t*)&link_mcu_from_fbw_msg)
#define RX_BUF ((uint8_t*)&link_mcu_from_ap_msg)
#define EndTransmit() {}

#define LinkMcuMessageReceived() ( !SpiIsSelected() && link_mcu_was_busy )
#endif

#ifdef AP 
#define TX_BUF ((uint8_t*)&link_mcu_from_ap_msg)
#define RX_BUF ((uint8_t*)&link_mcu_from_fbw_msg)
#define EndTransmit() \
      SpiDisableTxi();


#define LinkMcuStart() { \
  LinkMcuTransmit();  /* fill fifo */ \
  SpiEnableTxi();     /* enable tx fifo half empty interrupt */ \
  SpiEnableRti();     /* enable rx timeout interrupt         */ \
}
#endif

#define FRAME_LENGTH sizeof(struct link_mcu_msg)

#define LinkMcuTransmit() {						\
    if (link_mcu_tx_idx < FRAME_LENGTH) {				\
      SpiSend(TX_BUF[link_mcu_tx_idx]);	\
      link_mcu_tx_idx++;						\
    } else			\
      EndTransmit() \
}

#define LinkMcuReceive() {						\
      SpiRead(RX_BUF[link_mcu_rx_idx]);	\
      link_mcu_rx_idx++;						\
  }

#endif /* LINK_MCU_HW_H */
