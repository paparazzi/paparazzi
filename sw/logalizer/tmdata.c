/*  $Id$
 *
 * tmdata, an telemetry client to distribute paparazzi location data
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
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#define PORT_OUT 7124
#define HOSTADDR "localhost"
#define BUFSIZE  0x10000

int sock, length, fromlen;
struct sockaddr_in server;
char hostaddr[4096] = {HOSTADDR};

static const char usage_str[] =
"tmdata [options] [parameter]\n"
"options:\n"
"  -s HOST       server address\n";

int main ( int argc, char** argv) {

  struct hostent *hent;
  char buf[BUFSIZE];
  int c, count;
  fd_set fds;
  struct timeval tvt;
  char noparam = 0;
  char *param = &noparam;

  while ((c = getopt(argc, argv, "hs:")) != EOF)
  {
    switch (c)
    {
    case 'h':
      printf(usage_str);
      break;

    case 's':
      strncpy(hostaddr, optarg, strlen(optarg)+1);
      break;
    }
  }

  if (argv[optind])
  {
    param = argv[optind];
  }

  sock = socket(PF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    perror("socket");
    exit(1);
  }

  server.sin_family = AF_INET;
  hent = gethostbyname(hostaddr);
  if (hent == 0)
  {
    perror("unknown host");
    exit(1);
  }

  memcpy((char *)hent->h_addr,
         (char *)&server.sin_addr,
         hent->h_length);

  server.sin_port = htons(PORT_OUT);
  length=sizeof(struct sockaddr_in);

  if (connect(sock, &server, length))
  {
    perror("connect");
    exit(1);
  }

  /* send some data to trigger output */
  count = send(sock, param, strlen(param), 0);

  /* we will wait for data 500ms */
  tvt.tv_sec = 0;
  tvt.tv_usec = 500000;

  FD_CLEAR(&fds);
  FD_SET(sock, &fds);

  count = select(sock+1, &fds, NULL, NULL, &tvt);

  if (count > 0)
  {
    /* receive data packet containing formatted data */
    count = recv(sock, buf, sizeof(buf), 0);

    if (count < 0)
    {
      perror("recv");
      exit(1);
    }

    write(1, buf, count);
  }
  else
  {
    perror("select");
  }

  return 0;
}
