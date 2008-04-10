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

#include "booz_link_mcu.h"

#include "std.h"

struct booz_inter_mcu_state booz_link_mcu_state_unused;
uint16_t booz_link_mcu_crc;



#ifdef BOOZ_FILTER_MCU  /* FILTER LPC code */

/* FIXME!!!! two function with same name in single MCU configuration */
/* by chance  booz_link_mcu_hw_init does nothing in sim              */
#ifndef SITL
void booz_link_mcu_init ( void ) {
  booz_link_mcu_hw_init();
}
#endif

void booz_link_mcu_send ( void ) {
  inter_mcu_fill_state();
  BoozLinkMcuComputeCRC();
  inter_mcu_state.crc = booz_link_mcu_crc;
  BoozLinkMcuHwSend();
}



#endif /* BOOZ_FILTER_MCU */





#ifdef BOOZ_CONTROLLER_MCU  /* lpc controller board */

#include "spi.h"

#include "booz_estimator.h"

volatile uint8_t  booz_link_mcu_status;
uint32_t booz_link_mcu_nb_err;
uint32_t booz_link_mcu_timeout;
#define BOOZ_LINK_MCU_TIMEOUT 100

void booz_link_mcu_init ( void ) {

  booz_link_mcu_hw_init();
  booz_link_mcu_status = BOOZ_LINK_MCU_IDLE;
  booz_link_mcu_nb_err = 0;
  booz_link_mcu_timeout = BOOZ_LINK_MCU_TIMEOUT;

}

extern void booz_link_mcu_periodic( void ) {
  if (booz_link_mcu_timeout < BOOZ_LINK_MCU_TIMEOUT)
    booz_link_mcu_timeout++;
}

#endif /* BOOZ_CONTROLLER_MCU */
