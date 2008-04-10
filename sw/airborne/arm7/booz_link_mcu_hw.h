/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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

#ifndef BOOZ_LINK_MCU_HW_H
#define BOOZ_LINK_MCU_HW_H

#include "LPC21xx.h"
#include "booz_debug.h"

#define LINK_MCU_STATUS_IDLE 0
#define LINK_MCU_STATUS_BUSY 1

#define LINK_IMU_ERR_OVERRUN 0

extern volatile uint8_t link_mcu_status;
extern uint8_t* link_mcu_tx_buf;
extern uint8_t* link_mcu_rx_buf;
#define LINK_MCU_BUF_LEN (sizeof(struct booz_inter_mcu_state)/sizeof(uint8_t))

#ifdef  BOOZ_FILTER_MCU

extern volatile uint8_t link_mcu_buf_idx;

#define SS_PIN 7
#define SPI0_SelectSlave()   SetBit(IO0CLR, SS_PIN)
#define SPI0_UnselectSlave() SetBit(IO0SET, SS_PIN)

#define BoozLinkMcuHwSend() {						\
    ASSERT((link_mcu_status == LINK_MCU_STATUS_IDLE),			\
	   DEBUG_LINK_MCU_IMU, LINK_IMU_ERR_OVERRUN);			\
    if (link_mcu_status != LINK_MCU_STATUS_IDLE) return;		\
    SPI0_SelectSlave();							\
    link_mcu_status = LINK_MCU_STATUS_BUSY;				\
    link_mcu_buf_idx = 0;						\
    S0SPDR = link_mcu_tx_buf[0];					\
  }

#endif /* BOOZ_FILTER_MCU */


#ifdef  BOOZ_CONTROLLER_MCU

extern volatile uint8_t link_mcu_rx_buf_idx;
extern volatile uint8_t link_mcu_tx_buf_idx;

#define SSP_Enable()     SetBit(SSPCR1, SSE);
#define SSP_Disable()    ClearBit(SSPCR1, SSE);
#define SSP_EnableRxi()  SetBit(SSPIMSC, RXIM)
#define SSP_DisableRxi() ClearBit(SSPIMSC, RXIM)
#define SSP_EnableTxi()  SetBit(SSPIMSC, TXIM)
#define SSP_DisableTxi() ClearBit(SSPIMSC, TXIM)
#define SSP_EnableRti()  SetBit(SSPIMSC, RTIM);
#define SSP_DisableRti() ClearBit(SSPIMSC, RTIM);
#define SSP_ClearRti()   SetBit(SSPICR, RTIC);

#define LinkMcuTransmit() {						\
    while (link_mcu_tx_buf_idx < LINK_MCU_BUF_LEN			\
	   && bit_is_set(SSPSR, TNF)) {					\
      SSPDR = link_mcu_tx_buf[link_mcu_tx_buf_idx];			\
      link_mcu_tx_buf_idx++;						\
    }			                                                \
    if (link_mcu_tx_buf_idx == LINK_MCU_BUF_LEN)			\
      SSP_DisableRxi();							\
  }

#define BoozLinkMcuHwRestart() { \
    link_mcu_rx_buf_idx = 0;	 \
    link_mcu_tx_buf_idx = 0;	 \
    SSP_EnableRxi();		 \
    SSP_Enable();		 \
    /* LinkMcuTransmit(); */	 \
  }



#endif /* BOOZ_CONTROLLER_MCU */

#endif /* BOOZ_LINK_MCU_HW_H */
