/*  $Id$
 *
 * tmserver, an telemetry server to distribute paparazzi location data
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

#define MAX(a, b) ((a > b) ? a : b)

#define PORT_IN  7123
#define PORT_OUT 7124
#define BUFSIZE  0x10000

#define PACKET_SIZE 9

#define AC_NUM      10
#define AC_TIMEOUT  5

typedef struct
{
  long addr;
  struct timeval tvvalid;
  unsigned char ac_id;
  float fval[PACKET_SIZE];
} ac_dat_t;

static char ppfg[] = {"ppfg"};
static char ppua[] = {"ppua"};
static char ppac[] = {"ppac"};

int getac(ac_dat_t * ac_dati, unsigned char ac_id, long addr)
{
  int count;

  for(count=0; count < AC_NUM; count++)
  {
    if ((ac_dati[count].ac_id == ac_id) &&
        (ac_dati[count].addr == addr))
    {
      return count;
    }
  }
  for(count=0; count < AC_NUM; count++)
  {
    if (ac_dati[count].ac_id == 0) return count;
  }
  return -1;
}

int getnumac(ac_dat_t * ac_dati)
{
  int count, num=0;

  for(count=0; count < AC_NUM; count++)
  {
    if (ac_dati[count].ac_id != 0) num++;
  }
  return num;
}

int cleanupac(ac_dat_t * ac_dati)
{
  int count, num=0;
  struct timeval tvcur;

  gettimeofday(&tvcur, NULL);

  for(count=0; count < AC_NUM; count++)
  {
    if (ac_dati[count].tvvalid.tv_sec + AC_TIMEOUT < tvcur.tv_sec)
    {
      ac_dati[count].ac_id = 0;
    }
  }
  return num;
}

int main(int argc, char *argv[])
{
  int length, fromlen, tolen, n, count, cnt, fdmax, ac;
  int sourcesock, sinksock, consock;
  struct sockaddr_in sourceaddr, sinkaddr, from;
  char buf[BUFSIZE];
  char *pbuf;
  unsigned int itemp;
  fd_set fds;
  struct timeval tvt, tvcur;
  ac_dat_t ac_dat[AC_NUM];
  int on = 1;

  memset(ac_dat, 0, sizeof(ac_dat));

  /* open sockets */
  sourcesock = socket(AF_INET, SOCK_DGRAM, 0);
  sinksock = socket(PF_INET, SOCK_STREAM, 0);
  consock = 0;
  if ((sourcesock < 0) || (sinksock < 0))
  {
    perror("socket");
    exit(1);
  }

  tolen = sizeof(struct sockaddr_in);
  length = sizeof(sourceaddr);
  memset(&sourceaddr, 0, sizeof(sourceaddr));
  memset(&sinkaddr, 0, sizeof(sinkaddr));

  sourceaddr.sin_family = AF_INET;
  sourceaddr.sin_addr.s_addr = INADDR_ANY;
  sourceaddr.sin_port = htons(PORT_IN);
  sinkaddr.sin_family = AF_INET;
  sinkaddr.sin_addr.s_addr = INADDR_ANY;
  sinkaddr.sin_port = htons(PORT_OUT);

  setsockopt(sinksock, SOL_SOCKET, SO_REUSEADDR, (void *)&on, sizeof(on)) ;

  if ((bind(sourcesock, (struct sockaddr *)&sourceaddr, length) < 0) ||
      (bind(sinksock, (struct sockaddr *)&sinkaddr, length) < 0))
  {
    perror("bind");
    exit(1);
  }

  if (listen(sinksock, 3))
  {
    perror("listen");
    exit(1);
  }

  fdmax = MAX(sourcesock, sinksock);
  FD_ZERO(&fds);

  while (1)
  {
    FD_SET(sourcesock, &fds);
    FD_SET(sinksock, &fds);

    /* loop every second */
    tvt.tv_sec = 1;
    tvt.tv_usec = 0;

    count = select(fdmax+1, &fds, NULL, NULL, &tvt);

    if (FD_ISSET(sourcesock, &fds))
    {
      gettimeofday(&tvcur, NULL);
      n = recvfrom(sourcesock, buf, sizeof(buf), 0, (struct sockaddr *)&sourceaddr, &fromlen);
      if (n < 0) perror("recvfrom");

      if (n == PACKET_SIZE*4)
      {
        ac = getac(ac_dat, *(unsigned char*)(buf), ntohl(sourceaddr.sin_addr.s_addr));
        if (ac >= 0)
        {
          ac_dat[ac].ac_id = *(unsigned char*)(buf);
          ac_dat[ac].addr = ntohl(sourceaddr.sin_addr.s_addr);
          ac_dat[ac].tvvalid.tv_sec = tvcur.tv_sec;

          for (count=1; count<PACKET_SIZE; count++)
          {
            itemp = 0;
            for (cnt=0; cnt<4; cnt++)
            {
              itemp |= (*(unsigned char*)(buf+count*4+cnt) << (cnt*8));
            }
            itemp = ntohl(itemp);
            ac_dat[ac].fval[count] = *(float*) &itemp;
          }
        }
      }
    }

    if (FD_ISSET(sinksock, &fds))
    {
      consock = accept(sinksock, (struct sockaddr *) &sinkaddr, &fromlen);
      if (consock < 0)
      {
        perror("socket");
        exit(1);
      }
      FD_SET(consock, &fds);
      fdmax=MAX(fdmax, consock);
    }

    if (FD_ISSET(consock, &fds))
    {
      n = recv(consock, buf, sizeof(buf), 0);
      if (n < 0) perror("recv");

      if (strncmp(ppua, buf, 4) == 0)
      {
        pbuf = buf;
        ac = getnumac(ac_dat);
        pbuf += sprintf(pbuf,"Cache-Control: no-cache\n");
        pbuf += sprintf(pbuf,"Pragma: no-cache\n");
        pbuf += sprintf(pbuf,"Content-type: text/xml\n\n");
        pbuf += sprintf(pbuf,"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n");
        pbuf += sprintf(pbuf,"<kml xmlns=\"http://earth.google.com/kml/2.1\">\n");
        pbuf += sprintf(pbuf,"<NetworkLinkControl>\n");
        pbuf += sprintf(pbuf," <Update>\n");
        pbuf += sprintf(pbuf,"  <targetHref>http://localhost/maps/fg_server_xml.cgi?ppac</targetHref>\n");

        for (count = 0; count < ac; count++)
        {
          if (ac_dat[count].ac_id > 0)
          {
            pbuf += sprintf(pbuf, "  <Change>\n");
            pbuf += sprintf(pbuf, "   <Placemark targetId=\"%i\">\n", count);
#if 0
            pbuf += sprintf(pbuf, "    <LookAt>\n");
            pbuf += sprintf(pbuf, "     <longitude>%f</longitude>\n", ac_dat[count].fval[4]);
            pbuf += sprintf(pbuf, "     <latitude>%f</latitude>\n", ac_dat[count].fval[3]);
            pbuf += sprintf(pbuf, "     <altitude>%f</altitude>\n", ac_dat[count].fval[7]);
            pbuf += sprintf(pbuf, "     <range>100</range>\n");
            pbuf += sprintf(pbuf, "     <tilt>55</tilt>\n");
            pbuf += sprintf(pbuf, "     <heading>%f</heading>\n", ac_dat[count].fval[6]);
            pbuf += sprintf(pbuf, "     <altitudeMode>absolute</altitudeMode>\n");
            pbuf += sprintf(pbuf, "    </LookAt>\n");
#endif
            pbuf += sprintf(pbuf, "    <Model>\n");
            pbuf += sprintf(pbuf, "     <Location>\n");
            pbuf += sprintf(pbuf, "      <latitude>%f</latitude>\n", ac_dat[count].fval[3]);
            pbuf += sprintf(pbuf, "      <longitude>%f</longitude>\n", ac_dat[count].fval[4]);
            pbuf += sprintf(pbuf, "      <altitude>%f</altitude>\n", ac_dat[count].fval[7]);
            pbuf += sprintf(pbuf, "     </Location>\n");
            pbuf += sprintf(pbuf, "     <Orientation>\n");
            pbuf += sprintf(pbuf, "      <heading>%f</heading>\n", ac_dat[count].fval[6]);
            pbuf += sprintf(pbuf, "      <tilt>%f</tilt>\n", -ac_dat[count].fval[2]);
            pbuf += sprintf(pbuf, "      <roll>%f</roll>\n", -ac_dat[count].fval[1]);
            pbuf += sprintf(pbuf, "     </Orientation>\n");
            pbuf += sprintf(pbuf, "    </Model>\n");
            pbuf += sprintf(pbuf, "   </Placemark>\n");
            pbuf += sprintf(pbuf, "  </Change>\n");
          }
        }
        pbuf += sprintf(pbuf, " </Update>\n");
        pbuf += sprintf(pbuf, "</NetworkLinkControl>\n");
        pbuf += sprintf(pbuf, "</kml>\n");
      }
      else if (strncmp(ppac, buf, 4) == 0)
      {
        pbuf = buf;
        ac = getnumac(ac_dat);
        pbuf += sprintf(pbuf,"Cache-Control: no-cache\n");
        pbuf += sprintf(pbuf,"Pragma: no-cache\n");
        pbuf += sprintf(pbuf,"Content-type: text/xml\n\n");
        pbuf += sprintf(pbuf,"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n");
        pbuf += sprintf(pbuf,"<kml xmlns=\"http://earth.google.com/kml/2.1\">\n");
        pbuf += sprintf(pbuf," <Document id=\"mpmap\">\n");
        pbuf += sprintf(pbuf," <name>Paparazzi live aircrafts</name>\n");
        pbuf += sprintf(pbuf," <visibility>1</visibility>\n");

        for (count = 0; count < ac; count++)
        {
          if (ac_dat[count].ac_id > 0)
          {
            pbuf += sprintf(pbuf, " <Placemark id=\"%i\">\n", count);
            pbuf += sprintf(pbuf, "  <name>%i</name>\n", ac_dat[count].ac_id);
#if 0
            pbuf += sprintf(pbuf, "  <LookAt>\n");
            pbuf += sprintf(pbuf, "   <longitude>%f</longitude>\n", ac_dat[count].fval[4]);
            pbuf += sprintf(pbuf, "   <latitude>%f</latitude>\n", ac_dat[count].fval[3]);
            pbuf += sprintf(pbuf, "   <altitude>%f</altitude>\n", ac_dat[count].fval[7]);
            pbuf += sprintf(pbuf, "   <range>500</range>\n");
            pbuf += sprintf(pbuf, "   <tilt>55</tilt>\n");
            pbuf += sprintf(pbuf, "   <heading>%f</heading>\n", ac_dat[count].fval[7]);
            pbuf += sprintf(pbuf, "   <altitudeMode>absolute</altitudeMode>\n");
            pbuf += sprintf(pbuf, "  </LookAt>\n");
#endif
            pbuf += sprintf(pbuf, "  <description>Paparazzi</description>\n");
            pbuf += sprintf(pbuf, "  <Model>\n");
            pbuf += sprintf(pbuf, "   <altitudeMode>absolute</altitudeMode>\n");
            pbuf += sprintf(pbuf, "   <Location>\n");
            pbuf += sprintf(pbuf, "    <latitude>%f</latitude>\n", ac_dat[count].fval[3]);
            pbuf += sprintf(pbuf, "    <longitude>%f</longitude>\n", ac_dat[count].fval[4]);
            pbuf += sprintf(pbuf, "    <altitude>%f</altitude>\n", ac_dat[count].fval[7]);
            pbuf += sprintf(pbuf, "   </Location>\n");
            pbuf += sprintf(pbuf, "   <Orientation>\n");
            pbuf += sprintf(pbuf, "    <heading>%f</heading>\n", ac_dat[count].fval[6]);
            pbuf += sprintf(pbuf, "    <tilt>%f</tilt>\n", -ac_dat[count].fval[2]);
            pbuf += sprintf(pbuf, "    <roll>%f</roll>\n", -ac_dat[count].fval[1]);
            pbuf += sprintf(pbuf, "   </Orientation>\n");
            pbuf += sprintf(pbuf, "   <Scale>\n");
            pbuf += sprintf(pbuf, "    <x>150.0</x>\n");
            pbuf += sprintf(pbuf, "    <y>150.0</y>\n");
            pbuf += sprintf(pbuf, "    <z>150.0</z>\n");
            pbuf += sprintf(pbuf, "   </Scale>\n");
            pbuf += sprintf(pbuf, "   <Link>\n");
            pbuf += sprintf(pbuf, "    <href>http://localhost/maps/c172p.dae</href>\n");
            pbuf += sprintf(pbuf, "    <refreshMode>onChange</refreshMode>\n");
            pbuf += sprintf(pbuf, "   </Link>\n");
            pbuf += sprintf(pbuf, "  </Model>\n");
            pbuf += sprintf(pbuf, " </Placemark>\n\n");
          }
        }
        pbuf += sprintf(pbuf, " <NetworkLink id=\"fgmap_update\">\n");
        pbuf += sprintf(pbuf, "  <name>Update</name>\n");
        pbuf += sprintf(pbuf, "  <Link>\n");
        pbuf += sprintf(pbuf, "   <href>http://localhost/maps/fg_server_xml.cgi?ppua</href>\n");
        pbuf += sprintf(pbuf, "   <refreshMode>onInterval</refreshMode>\n");
        pbuf += sprintf(pbuf, "   <refreshInterval>1</refreshInterval>\n");
        pbuf += sprintf(pbuf, "  </Link>\n");
        pbuf += sprintf(pbuf, " </NetworkLink>\n</Document>\n");
        pbuf += sprintf(pbuf, "</kml>\n");
      }
      else //if strncmp(ppfg, buf, 4) == 0)
      {
        pbuf=buf;
        ac = getnumac(ac_dat);

        pbuf += sprintf(pbuf,"Cache-Control: no-cache\n");
        pbuf += sprintf(pbuf,"Pragma: no-cache\n");
        pbuf += sprintf(pbuf, "Content-type: text/xml\n\n<fg_server pilot_cnt=\"%d\">\n", ac);

        for (count = 0; count < ac; count++)
        {
          if (ac_dat[count].ac_id > 0)
          {
            pbuf += sprintf(pbuf, "<marker callsign=\"%i\" server_ip=\"%u.%u.%u.%u\" model=\"c172p\" lat=\"%f\" lng=\"%f\" alt=\"%f\" heading=\"%f\" pitch=\"%f\" roll=\"%f\"/>\n",
                     ac_dat[count].ac_id,
                     (unsigned char)(ac_dat[count].addr >> 24),
                     (unsigned char)(ac_dat[count].addr >> 16),
                     (unsigned char)(ac_dat[count].addr >> 8),
                     (unsigned char)(ac_dat[count].addr & 0xff),
                     ac_dat[count].fval[3],
                     ac_dat[count].fval[4],
                     ac_dat[count].fval[7],
                     ac_dat[count].fval[6],
                     ac_dat[count].fval[2],
                     ac_dat[count].fval[1]);
          }
        }
        pbuf += sprintf(pbuf, "</fg_server>\n");
      }
//      else buf[0]=0;

      if (ntohl(sinkaddr.sin_addr.s_addr) == INADDR_LOOPBACK)
      {
        count = send(consock, buf, strlen(buf), 0);
      }
      close(consock);
      FD_CLR(consock, &fds);
      fdmax = MAX(sourcesock, sinksock);
    }

    cleanupac(ac_dat);
  }

  return 0;
}
