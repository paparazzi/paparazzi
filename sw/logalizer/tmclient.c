/*  $Id$
 *
 * tmclient, an telemetry client to distribute paparazzi location data
 * Copyright (C) 2007  Martin Mueller <martinmm@pfump.org>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define PORT 7123
#define HOSTADDR "localhost"

#define PACKET_SIZE 9

int sock, length;
struct sockaddr_in server;
char hostaddr[4096] = {HOSTADDR};

unsigned int  uvalue[PACKET_SIZE];
unsigned char cvalue[4*PACKET_SIZE];

static const char usage_str[] =
"client [options]\n"
"options:\n"
"  -s HOST       server address\n";

void on_FLIGHT_PARAM_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  int count, cnt;
  float ftemp;

  if (argc != 10) return;

  uvalue[0] = 42;//atoi(argv[0]);
  cvalue[0] = uvalue[0] & 0xFF;
  cvalue[1] = 0;
  cvalue[2] = 'P';
  cvalue[3] = 'Z';

  for (count=1; count < PACKET_SIZE; count++) {
    ftemp = atof(argv[count]);
    uvalue[count] = *(unsigned int*) &ftemp;
    uvalue[count] = htonl(uvalue[count]);
    for (cnt=0; cnt<4; cnt++) {
      cvalue[count*4 + cnt] = (uvalue[count] >> (cnt*8)) & 0xFF;
    }
  }

  /* send data as binary 32bit floats through UDP */
  count = sendto(sock, cvalue, PACKET_SIZE*4, 0, (struct sockaddr*) &server, length);
  if (count != PACKET_SIZE*4) perror("sendto");
}


int main ( int argc, char** argv) {

  struct hostent *hent;
  int c;
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  while ((c = getopt(argc, argv, "h:s:")) != EOF) {
    switch (c) {

    case 'h':
      printf(usage_str);
      break;

    case 's':
      strncpy(hostaddr, optarg, strlen(optarg)+1);
      break;
    }
  }

  IvyInit ("IvyFlightParams", "IvyFlightParams READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_FLIGHT_PARAM_STATUS, NULL, "^\\S* FLIGHT_PARAM (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  printf("Using server: %s\n", hostaddr);

  sock= socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) perror("socket");

  server.sin_family = AF_INET;
  hent = gethostbyname(hostaddr);
  if (hent == 0) {
    perror("unknown host %s");
    exit(1);
  }

  memcpy((char *)&server.sin_addr,
         (char *)hent->h_addr,
         hent->h_length);

  server.sin_port = htons(PORT);
  length=sizeof(struct sockaddr_in);

  g_main_loop_run(ml);

  return 0;
}
