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

#ifndef BOOZ_LINK_LINK_MCU_H
#define BOOZ_LINK_LINK_MCU_H

#include <inttypes.h>

#include "6dof.h"

#include "booz_link_mcu_hw.h"

#include "booz_inter_mcu.h"

extern void booz_link_mcu_init ( void );
extern void booz_link_mcu_hw_init ( void );
extern struct booz_inter_mcu_state booz_link_mcu_state_unused; /* single dir only */
extern uint16_t booz_link_mcu_crc;

#define BOOZ_LINK_MCU_CRC_INIT 0x0
#define BOOZ_LINK_MCU_PAYLOAD_LENGTH (sizeof(struct booz_inter_mcu_state) - 2)
#define BoozLinkMcuCrcLow(x) (((x)&0xff))
#define BoozLinkMcuCrcHigh(x) (((x)>>8))
#define BoozLinkMcuComputeCRC() {					\
    uint8_t i;								\
    booz_link_mcu_crc = BOOZ_LINK_MCU_CRC_INIT;				\
    for(i = 0; i < BOOZ_LINK_MCU_PAYLOAD_LENGTH; i++) {			\
      uint8_t _byte = ((uint8_t*)&inter_mcu_state)[i];			\
      uint8_t a = ((uint8_t)BoozLinkMcuCrcHigh(booz_link_mcu_crc)) + _byte; \
      uint8_t b = ((uint8_t)BoozLinkMcuCrcLow(booz_link_mcu_crc)) + a;	\
      booz_link_mcu_crc = b | a << 8;					\
    }									\
  }

#ifdef BOOZ_FILTER_MCU

extern void booz_link_mcu_send( void );

#endif /* BOOZ_FILTER_MCU */




#ifdef BOOZ_CONTROLLER_MCU

#define BOOZ_LINK_MCU_IDLE           0
#define BOOZ_LINK_MCU_DATA_AVAILABLE 1
#define BOOZ_LINK_MCU_SHORT_READ     2
#define BOOZ_LINK_MCU_OVER_READ      3

extern volatile uint8_t  booz_link_mcu_status;
extern uint32_t booz_link_mcu_nb_err;
extern uint32_t booz_link_mcu_timeout;
extern void booz_link_mcu_periodic( void );

#define BoozLinkMcuEventCheckAndHandle(handler) {		\
    if (booz_link_mcu_status == BOOZ_LINK_MCU_DATA_AVAILABLE) {	\
      BoozLinkMcuComputeCRC();					\
      if (booz_link_mcu_crc == inter_mcu_state.crc) {		\
	booz_link_mcu_timeout = 0;				\
	handler();						\
      }								\
      else {							\
	booz_link_mcu_nb_err++;					\
      }								\
      BoozLinkMcuHwRestart();					\
      booz_link_mcu_status = BOOZ_LINK_MCU_IDLE;		\
    }								\
  }								\

#endif /* BOOZ_CONTROLLER_MCU */


#endif /* BOOZ_LINK_LINK_MCU_H */
