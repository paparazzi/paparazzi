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

#include "subsystems/datalink/udp.h"
#include "fms/fms_network.h"
#include <string.h>

//Check if variables are set and else define them
#ifndef LINK_HOST
#define LINK_HOST     "192.168.1.255"
#endif
#ifndef LINK_PORT
#define LINK_PORT             4242
#endif
#ifndef DATALINK_PORT
#define DATALINK_PORT         4243
#endif
#ifndef FMS_NETWORK_BROADCAST
#define FMS_NETWORK_BROADCAST TRUE
#endif

//Define the buffer, check bytes and FmsNetwork
char udp_buffer[1496];
unsigned char udp_read_buffer[128];
uint16_t udp_buffer_id;
uint8_t ck_a, ck_b;
struct FmsNetwork* network;

void udp_init( void ) {
  network = network_new(LINK_HOST, LINK_PORT, DATALINK_PORT, FMS_NETWORK_BROADCAST);
  udp_buffer_id = 0;
}

void udp_transmit( uint8_t data ) {
  udp_buffer[udp_buffer_id] = data;
  udp_buffer_id++;
}

void udp_send( void ) {
  network_write(network, udp_buffer, udp_buffer_id);
  udp_buffer_id =0;
}

void udp_receive( void ) {
  //First check if it isn't already having a message
  if(dl_msg_available == TRUE) {
    return;
  }

  //Read from the network
  network_read(network, udp_read_buffer, TRANSPORT_PAYLOAD_LEN);

  //Parse the packet
  if(udp_read_buffer[0] == STX) {
    uint8_t size = udp_read_buffer[1]-4; // minus STX, LENGTH, CK_A, CK_B
    uint8_t ck_aa, ck_bb;
    ck_aa = ck_bb = size+4;

    // index-offset plus 2 for STX and LENGTH
    for (int i = 2; i < size+2; i++) {
      dl_buffer[i-2] = udp_read_buffer[i];
      ck_aa += udp_read_buffer[i];
      ck_bb += ck_aa;
    }

    // if both checksums are good, tell datalink that the message is available
    if (udp_read_buffer[2+size] == ck_aa && udp_read_buffer[2+size+1] == ck_bb) {
      dl_msg_available = TRUE;
    }

  }

  memset(&udp_read_buffer[0], 0, sizeof(udp_read_buffer));
}

