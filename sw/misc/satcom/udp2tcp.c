/*
 * Paparazzi udp to tcp handling for sat based telemetry
 *
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
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

#include <unistd.h>
#include <error.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>

#define MAX(a, b) ((a > b) ? a : b)

#define PORT_IN  7023
#define PORT_OUT 7024
#define BUFSIZE  0x10000

#define NUM_STREAMS 20

int main(int argc, char *argv[]) {
  socklen_t fromlen;
  int length, tolen, n, count, fdmax, i;
  int sourcesock, sinksock, consock[NUM_STREAMS] = {0};
  struct sockaddr_in sourceaddr, sinkaddr;
  char buf[BUFSIZE];
  fd_set fds;
  struct timeval tvt, tvcur;
  int on = 1;

  signal(SIGCHLD, SIG_IGN);

  /* open sockets */
  sourcesock = socket(AF_INET, SOCK_DGRAM, 0);
  sinksock = socket(PF_INET, SOCK_STREAM, 0);
  if ((sourcesock < 0) || (sinksock < 0)) {
    perror("socket");
    exit(1);
  }

  tolen = sizeof(struct sockaddr_in);
  length = sizeof(sourceaddr);
  memset(&sourceaddr, 0, sizeof(sourceaddr));
  memset(&sinkaddr, 0, sizeof(sinkaddr));

  sourceaddr.sin_family = AF_INET;
  sourceaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  sourceaddr.sin_port = htons(PORT_IN);

  sinkaddr.sin_family = AF_INET;
  sinkaddr.sin_addr.s_addr = INADDR_ANY;
  sinkaddr.sin_port = htons(PORT_OUT);

  setsockopt(sinksock, SOL_SOCKET, SO_REUSEADDR, (void *)&on, sizeof(on)) ;

  if ((bind(sourcesock, (struct sockaddr *)&sourceaddr, length) < 0) ||
      (bind(sinksock, (struct sockaddr *)&sinkaddr, length) < 0)) {
    perror("bind");
    exit(1);
  }

  if (listen(sinksock, 3)) {
    perror("listen");
    exit(1);
  }

  fdmax = MAX(sourcesock, sinksock);
  FD_ZERO(&fds);

  while (1) {
    fdmax = MAX(sourcesock, sinksock);
    FD_ZERO(&fds);
    FD_SET(sourcesock, &fds);
    FD_SET(sinksock, &fds);
    for (i=0; i<NUM_STREAMS; i++) {
      if (consock[i] != 0) {
        FD_SET(consock[i], &fds);
        fdmax=MAX(fdmax, consock[i]);
      }
    }

    /* loop every second */
    tvt.tv_sec = 1;
    tvt.tv_usec = 0;

    count = select(fdmax+1, &fds, NULL, NULL, &tvt);

    for (i=0; i<NUM_STREAMS; i++) {
      if (FD_ISSET(consock[i], &fds)) {
        n = recv(consock[i], buf, sizeof(buf), 0);

        if (n <= 0) {
          if (n < 0) perror("recv");
          printf("disconnect\n");
          close(consock[i]);
          FD_CLR(consock[i], &fds);
          fdmax = MAX(sourcesock, sinksock);
          consock[i] = 0;
        }
        else {
          printf("receive uplink %d\n", n);
          /* create email to satcom */
        }
      }
    }


    if (FD_ISSET(sourcesock, &fds)) {
      gettimeofday(&tvcur, NULL);
      n = recvfrom(sourcesock, buf, sizeof(buf), 0,
                   (struct sockaddr *)&sourceaddr, (socklen_t *)&fromlen);
      if (n < 0) perror("recvfrom");

      printf("receive downlink %d\n", n);

      for (i=0; i<NUM_STREAMS; i++) {
        if (consock[i] != 0) {
          printf("send downlink %d\n", i);
          count = send(consock[i], buf, n, 0);
          if (count <= 0) {
            printf("disconnect\n");
            close(consock[i]);
            FD_CLR(consock[i], &fds);
            fdmax = MAX(sourcesock, sinksock);
            consock[i] = 0;
          }
        }
      }
    }

    if (FD_ISSET(sinksock, &fds)) {
      for (i=0; i<NUM_STREAMS; i++) {
        if (consock[i] == 0) {
          consock[i] = accept(sinksock, (struct sockaddr *) &sinkaddr,
                              (socklen_t *)&fromlen);
          if (consock[i] < 0) {
            perror("socket");
            consock[i] = 0;
          }
          else {
            printf("connect\n");
            /* set keepalive */
            setsockopt(consock[i], SOL_SOCKET, SO_KEEPALIVE, &i, sizeof(i));
            /* set non blocking */
            fcntl(consock[i], F_SETFL, O_NONBLOCK);
            break;
          }
          printf("connect\n");
          FD_SET(consock[i], &fds);
          fdmax=MAX(fdmax, consock[i]);
          break;
        }
      }
    }
  }
  return 0;
}
