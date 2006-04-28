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

/** \brief handling of arm7 inter mcu link
 *  
 */

#ifndef LINK_MCU_HW_H
#define LINK_MCU_HW_H

#ifdef FBW

#define LINK_MCU_TRANSMIT() {						\
    while (link_mcu_tx_idx < FRAME_LENGTH && bit_is_set(SSPSR, TNF)) {	\
      SPI_SEND(((uint8_t*)&from_ap)[link_mcu_tx_idx]);			\
      link_mcu_tx_idx++;						\
    }									\
}

#define LINK_MCU_RECEIVE() {				\
    while ( bit_is_set(SSPSR, RNE)) {			\
      SPI_READ(((uint8_t*)&from_fbw)[link_mcu_rx_idx]);	\
      link_mcu_rx_idx++;				\
    }							\
  }

#endif /* FBW */


#ifdef AP 

#define LINK_MCU_TRANSMIT() { \
    while (link_mcu_tx_idx < FRAME_LENGTH && bit_is_set(SSPSR, TNF)) {	\
      SPI_SEND(((uint8_t*)&from_ap)[link_mcu_tx_idx]);			\
      link_mcu_tx_idx++;						\
    }									\
    if (link_mcu_tx_idx == FRAME_LENGTH)				\
      SPI_DISABLE_TXI();						\
}

#define LINK_MCU_RECEIVE() {				\
    while ( bit_is_set(SSPSR, RNE)) {			\
      SPI_READ(((uint8_t*)&from_fbw)[link_mcu_rx_idx]);	\
      link_mcu_rx_idx++;				\
    }							\
  }

#endif /* AP */


#endif /* LINK_MCU_HW_H */
