/*
 * Copyright (C) 2012-2013 Freek van Tienen
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
 */

/**
 * @file boards/ardrone/at_com.c
 * Sending and receiving of AT-commands specified by the ardrone API
 */

#include "at_com.h"
#include "boards/ardrone2_sdk.h"
#include "generated/airframe.h"

#include <stdlib.h>
#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

int packet_seq = 1; //Packet sequence number

int at_socket = -1, //AT socket connection
    navdata_socket = -1; //Navdata socket connection

struct sockaddr_in pc_addr, //Own pc address
    drone_at, //Drone AT address
    drone_nav, //Drone nav address
    from; //From address

bool_t at_com_ready = FALSE; //Status of the at communication
char sessionId[9]; //THe config session ID

void at_com_send(char *command);
void init_at_config(void);

//Init the at_com
void init_at_com(void)
{
  //Check if already initialized
  if (at_com_ready) {
    return;
  }

  //Create the at and navdata socket
  if ((at_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("at_com: at_socket error (%s)\n", strerror(errno));
  }
  if ((navdata_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("at_com: navdata_socket error (%s)\n", strerror(errno));
  }

  //For recvfrom
  pc_addr.sin_family = AF_INET;
  pc_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  pc_addr.sin_port = htons(9800);

  //For sendto AT
  drone_at.sin_family = AF_INET;
  drone_at.sin_addr.s_addr = inet_addr(ARDRONE_IP);
  drone_at.sin_port = htons(ARDRONE_AT_PORT);

  //For sendto navadata init
  drone_nav.sin_family = AF_INET;
  drone_nav.sin_addr.s_addr = inet_addr(ARDRONE_IP);
  drone_nav.sin_port = htons(ARDRONE_NAVDATA_PORT);

  //Bind the navdata socket
  if (bind(navdata_socket, (struct sockaddr *) &pc_addr, sizeof(pc_addr)) < 0) {
    printf("at_com: bind error (%s)\n", strerror(errno));
  }

  //Set unicast mode on
  int one = 1;
  sendto(navdata_socket, &one, 4, 0, (struct sockaddr *) &drone_nav,
         sizeof(drone_nav));

  //Init at config
  init_at_config();

  //Set at_com to ready
  at_com_ready = TRUE;
}

//Init the at config
void init_at_config(void)
{
  //Generate a session id
  uint32_t binaryId = (uint32_t) rand();
  binaryId = (0 != binaryId) ? binaryId : 1u;
  snprintf(sessionId, 9, "%08x", binaryId);
  sessionId[8] = '\0';

  //Send session, application and user id:
  at_com_send_config("custom:session_id", sessionId);
  at_com_send_config("custom:application_id", "9D7BFD45");
  at_com_send_config("custom:profile_id", "2BF07F58");

  //Send config values
  at_com_send_config("control:euler_angle_max", "0.52");
  at_com_send_config("control:altitude_max", "20000");
  at_com_send_config("control:control_vz_max", "2000");
  at_com_send_config("control:control_yaw", "6.11");

  //Send config values with the airframe.h
#ifndef ARDRONE_FLIGHT_INDOOR
  at_com_send_config("control:outdoor", "TRUE");
#else
  at_com_send_config("control:outdoor", "FALSE");
#endif
#ifndef ARDRONE_WITHOUT_SHELL
  at_com_send_config("control:flight_without_shell", "FALSE");
#else
  at_com_send_config("control:flight_without_shell", "TRUE");
#endif
#ifdef ARDRONE_OWNER_MAC
  at_com_send_config("network:owner_mac", ARDRONE_OWNER_MAC);
#endif
}

//Recieve a navdata packet
int at_com_recieve_navdata(unsigned char *buffer)
{
  int l = sizeof(from);
  int n;
  // FIXME(ben): not clear why recvfrom() and not recv() is used.
  n = recvfrom(navdata_socket, buffer, ARDRONE_NAVDATA_BUFFER_SIZE, 0x0,
               (struct sockaddr *) &from, (socklen_t *) &l);

  return n;
}

//Send an AT command
void at_com_send(char *command)
{
  sendto(at_socket, command, strlen(command), 0, (struct sockaddr *) &drone_at,
         sizeof(drone_at));
}

//Send a Config
void at_com_send_config(char *key, char *value)
{
  char command[256];
  sprintf(command, "AT*CONFIG_IDS=%d,\"%s\",\"2BF07F58\",\"9D7BFD45\"\r",
          packet_seq++, sessionId);
  at_com_send(command);
  sprintf(command, "AT*CONFIG=%d,\"%s\",\"%s\"\r", packet_seq++, key, value);
  at_com_send(command);
}

//Send a Flat trim
void at_com_send_ftrim(void)
{
  char command[256];
  sprintf(command, "AT*FTRIM=%d\r", packet_seq++);
  at_com_send(command);
}

//Send a Ref
void at_com_send_ref(int bits)
{
  char command[256];
  sprintf(command, "AT*REF=%d,%d\r", packet_seq++, bits | REF_DEFAULT);
  at_com_send(command);
}

//Send a Pcmd
void at_com_send_pcmd(int mode, float thrust, float roll, float pitch,
                      float yaw)
{
  int f_thrust, f_roll, f_pitch, f_yaw;
  char command[256];

  //Change the floats to ints(dereferencing)
  memcpy(&f_thrust, &thrust, sizeof thrust);
  memcpy(&f_roll, &roll, sizeof roll);
  memcpy(&f_pitch, &pitch, sizeof pitch);
  memcpy(&f_yaw, &yaw, sizeof yaw);

  sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", packet_seq++, mode, f_roll,
          f_pitch, f_thrust, f_yaw);
  at_com_send(command);
}

//Send a Calib
void at_com_send_calib(int device)
{
  char command[256];
  sprintf(command, "AT*CALIB=%d,%d\r", packet_seq++, device);
  at_com_send(command);
}
