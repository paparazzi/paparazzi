/*
* Copyright (C) 2012-2013 Freek van Tienen and Dino Hensen
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with paparazzi; see the file COPYING. If not, write to
* the Free Software Foundation, 59 Temple Place - Suite 330,
* Boston, MA 02111-1307, USA.
*
*/

/* Udp ethernet connection over UDP */

#ifndef UDP_TELEM_H
#define UDP_TELEM_H

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"

//#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/pprz_transport.h"

#define STX 0x99

void udp_init( void );
void udp_transmit( uint8_t data );
void udp_send( void );
void udp_receive( void );

#define UdpInit() udp_init()
#define UdpCheckFreeSpace(_x) (TRUE)
#define UdpTransmit(_x) udp_transmit(_x)
#define UdpSendMessage() udp_send()

#define UdpCheckAndParse() {       \
    udp_receive(); \
  }

#endif /* UDP_TELEM_H */
