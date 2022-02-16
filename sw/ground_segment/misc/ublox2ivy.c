/*
 * Paparazzi UBLox to Ivy
 *
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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

#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include "ublox2ivy.h"

#define UDP_BUFFER_SIZE   1024

/* Endpoints */
struct endpoint_udp_t {
  char server_addr[128];
  uint16_t server_port;
  char client_addr[128];
  uint16_t client_port;

  int fd;
  pthread_t thread;
};

struct endpoint_uart_t {
  char devname[512];
  int baudrate;

  int fd;
  pthread_t thread;
};

enum endpoint_type_t {
  ENDPOINT_TYPE_NONE,
  ENDPOINT_TYPE_UDP,
  ENDPOINT_TYPE_UART
};

struct endpoint_t {
  enum endpoint_type_t type;
  union {
    struct endpoint_udp_t udp;
    struct endpoint_uart_t uart;
  } ep;
};

static bool verbose = false;
static struct endpoint_t gps_ep;
static struct gps_ubx_t gps_ubx;
static struct gps_rtcm_t gps_rtcm;
static uint8_t ac_id = 1;

void packet_handler(void *ep, uint8_t *data, uint16_t len);

static const unsigned int tbl_CRC24Q[] = {
  0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
  0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
  0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
  0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
  0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
  0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
  0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
  0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
  0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
  0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
  0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
  0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
  0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
  0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
  0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
  0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
  0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
  0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
  0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
  0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
  0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
  0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
  0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
  0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
  0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
  0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
  0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
  0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
  0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
  0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
  0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
  0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538
};

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
unsigned int crc24q(const unsigned char *buff, int len)
{
  unsigned int crc = 0;
  int i;
  for (i = 0; i < len; i++) { crc = ((crc << 8) & 0xFFFFFF)^tbl_CRC24Q[(crc >> 16)^buff[i]]; }
  return crc;
}

unsigned int RTCMgetbitu(unsigned char *buff, int pos, int lenb)
{
  unsigned int bits = 0;
  int i;
  for (i = pos; i < pos + lenb; i++) { bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u); }
  return bits;
}

/**
 * Create an UDP endpoint thread
 */
void *udp_endpoint(void *arg) {
  struct endpoint_udp_t *ep = (struct endpoint_udp_t *)arg;

  /* Create the socket */
	ep->fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(ep->fd < 0) {
		fprintf(stderr, "Could not create socket for %s:%d\r\n", ep->server_addr, ep->server_port);
    return NULL;
  }

  /* Enable broadcasting */
  int one = 1;
  setsockopt(ep->fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

  /* Create the input address */
	struct sockaddr_in server;
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = inet_addr(ep->server_addr);
	server.sin_port = htons(ep->server_port);

  /* Bind the socket with the server address  */
  if(bind(ep->fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    fprintf(stderr, "Could not bind to address %s:%d\r\n", ep->server_addr, ep->server_port);
    return NULL;
  }

  /* Wait for messages */
  while(true) {
    struct sockaddr_in client;
    uint8_t buffer[UDP_BUFFER_SIZE];
    socklen_t len = sizeof(client);
    int n = recvfrom(ep->fd, buffer, UDP_BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&client, &len);

    // Ignore errors
    if(n < 0)
      continue;

    if(verbose) printf("Got packet at endpoint [%s:%d] with length %d\r\n", ep->server_addr, ep->server_port, n);

    // Send the message to the handler
    packet_handler(ep, buffer, n);
  }
}

/**
 * Send a message out of an UDP endpoint
 */
void udp_endpoint_send(struct endpoint_udp_t *ep, uint8_t *buffer, uint16_t len) {
  // Check if file descriptor is valid
  if(ep->fd < 0)
    return;

  struct sockaddr_in client;
	client.sin_family = AF_INET;
	client.sin_addr.s_addr = inet_addr(ep->client_addr);
	client.sin_port = htons(ep->client_port);

  if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", ep->client_addr, ep->client_port, len);
  sendto(ep->fd, buffer, len, MSG_DONTWAIT, (struct sockaddr *)&client, sizeof(client));
}

/**
 * Create an UDP endpoint
 */
void udp_create(char *server_addr, uint16_t server_port, char *client_addr, uint16_t client_port) {
  /* Create the endpoint */
  gps_ep.type = ENDPOINT_TYPE_UDP;
  struct endpoint_udp_t *ep = &gps_ep.ep.udp;
  strncpy(ep->server_addr, server_addr, 127);
  ep->server_port = server_port;
  strncpy(ep->client_addr, client_addr, 127);
  ep->client_port = client_port;

  /* Start the endpoint thread */
  if(verbose) printf("Created UDP endpoint with server [%s:%d] and client [%s:%d]\r\n", ep->server_addr, ep->server_port, ep->client_addr, ep->client_port);
  pthread_create(&ep->thread, NULL, udp_endpoint, ep);
}

/**
 * Get the baudrate based ont he argument
 */
int get_baud(unsigned int baud_rate)
{
  int baud = 0;
  switch (baud_rate)
  {
#ifdef B921600
    case 921600:
      baud = B921600;
      break;
#endif
#ifdef B460800
    case 460800:
      baud = B460800;
      break;
#endif
    case 230400:
      baud = B230400;
      break;
    case 115200:
      baud = B115200;
      break;
    case 57600:
      baud = B57600;
      break;
    case 38400:
      baud = B38400;
      break;
    case 19200:
      baud  = B19200;
      break;
    case 9600:
      baud  = B9600;
      break;
    case 4800:
      baud  = B4800;
      break;
    case 2400:
      baud  = B2400;
      break;
    case 1800:
      baud  = B1800;
      break;
    case 1200:
      baud  = B1200;
      break;
    case 600:
      baud  = B600;
      break;
    case 300:
      baud  = B300;
      break;
    case 200:
      baud  = B200;
      break;
    case 150:
      baud  = B150;
      break;
    case 134:
      baud  = B134;
      break;
    case 110:
      baud  = B110;
      break;
    case 75:
      baud  = B75;
      break;
    case 50:
      baud  = B50;
      break;
    default:
      baud = -1;
  }  //end of switch baud_rate
  return baud;
}

/**
 * Create an UART endpoint thread
 */
void *uart_endpoint(void *arg) {
  struct endpoint_uart_t *ep = (struct endpoint_uart_t *)arg;

  /* Open de uart device in blocking Read/Write mode */
  ep->fd = open(ep->devname, O_RDWR | O_NOCTTY);
  if(ep->fd < 0) {
    fprintf(stderr, "Could not open uart for %s:%d\r\n", ep->devname, ep->baudrate);
    return NULL;
  }

  /* Conver the baudrate */
  int baud = get_baud(ep->baudrate);
  if(baud < 0) {
    fprintf(stderr, "Could set baudrate for %s:%d\r\n", ep->devname, ep->baudrate);
    return NULL;
  }

  /* Configure the UART */
  struct termios options;
	tcgetattr(ep->fd, &options);
	options.c_cflag = baud | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(ep->fd, TCIFLUSH);
	tcsetattr(ep->fd, TCSANOW, &options);

  /* Try to read bytes */
  while(true) {
    uint8_t buffer[2048];
    int len = read(ep->fd, buffer, 2048);

    // Read got an error
    if(len < 0) {
      if(verbose) printf("Got read error (%d) at endpoint [%s:%d]\r\n", len, ep->devname, ep->baudrate);
      usleep(20);
    }
    // No bytes to read
    else if(len == 0) {
      usleep(20);
    }
    // We succsfully received some bytes
    else if(len > 0) {
      if(verbose) printf("Got packet at endpoint [%s:%d] with length %d\r\n", ep->devname, ep->baudrate, len);

      // Send the message to the handler
      packet_handler(ep, buffer, len);
    }
  }
}

/**
 * Send a message out of an UART endpoint
 */
void uart_endpoint_send(struct endpoint_uart_t *ep, uint8_t *buffer, uint16_t len) {
  // Check if file descriptor is valid
  if(ep->fd < 0)
    return;

  if(verbose) printf("Send packet at endpoint [%s:%d] with length %d\r\n", ep->devname, ep->baudrate, len);
  if(write(ep->fd, buffer, len) < 0)
    fprintf(stderr, "Send packet at endpoint [%s:%d] with length %d errored\r\n", ep->devname, ep->baudrate, len);
}

/**
 * Create an UART endpoint
 */
void uart_create(char *devname, uint32_t baudrate) {
  /* Create the endpoint */
  gps_ep.type = ENDPOINT_TYPE_UART;
  struct endpoint_uart_t *ep = &gps_ep.ep.uart;
  strncpy(ep->devname, devname, 511);
  ep->baudrate = baudrate;

  /* Start the endpoint thread */
  if(verbose) printf("Created UART endpoint [%s:%d]\r\n", ep->devname, ep->baudrate);
  pthread_create(&ep->thread, NULL, uart_endpoint, ep);
}

/* RTCM parsing */
void gps_rtcm_parse(uint8_t c) {
  switch(gps_rtcm.status) {
    case UNINIT:
      if (c == GPS_RTCM_SYNC1) {
        gps_rtcm.msg_idx = 0;
        gps_rtcm.msg_buf[gps_rtcm.msg_idx++] = c;
        gps_rtcm.status++;
      }
      break;
    case GOT_SYNC1:
      gps_rtcm.len = (c << 8) & 0x3FF;
      gps_rtcm.msg_buf[gps_rtcm.msg_idx++] = c;
      gps_rtcm.status = GOT_LEN1;
      break;
    case GOT_LEN1:
      gps_rtcm.len = (gps_rtcm.len | c) & 0x3FF;
      gps_rtcm.msg_buf[gps_rtcm.msg_idx++] = c;
      gps_rtcm.status = GOT_LEN2;
      break;
    case GOT_LEN2:
      gps_rtcm.msg_buf[gps_rtcm.msg_idx++] = c;

      /* Received a full message */
      if (gps_rtcm.msg_idx >= (gps_rtcm.len + 6)) {
        gps_rtcm.status = UNINIT;

        /* Perform checksum check */
        if(crc24q(gps_rtcm.msg_buf, gps_rtcm.len + 3) ==  RTCMgetbitu(gps_rtcm.msg_buf, (gps_rtcm.len + 3) * 8, 24)) {
          gps_rtcm.msg_available = true;
        } else {
          if(verbose) printf("GPS RTCM checksum error\r\n");

          /* Try to resync */
          if(gps_ubx.len > 0) {
            const uint8_t *p = (const uint8_t *)memchr(&gps_rtcm.msg_buf[1], GPS_RTCM_SYNC1, (gps_rtcm.msg_idx-1));
            if(p != NULL) {
              if(verbose) printf("GPS RTCM parser resynced\r\n");
              uint16_t len_diff = p - gps_rtcm.msg_buf;
              uint16_t len = gps_rtcm.msg_idx - len_diff;
              uint8_t msg_buf[len];
              memcpy(msg_buf, p, len);

              /* Recursion (chances of extremely deep recursion are really low) */
              for(uint16_t i = 0; i < len; i++) {
                gps_rtcm_parse(msg_buf[i]);

                /* If two messages are embedded, the second one is discared */
                if(gps_rtcm.msg_available)
                  return;
              }
            }
          }
        }
        return;
      }
      break;
    default:
      /* Should not come here */
      break;
  }
}

/* UBX parsing */
void gps_ubx_parse(uint8_t c)
{
  if (gps_ubx.status < GOT_PAYLOAD) {
    gps_ubx.ck_a += c;
    gps_ubx.ck_b += gps_ubx.ck_a;
  }
  switch (gps_ubx.status) {
    case UNINIT:
      if (c == GPS_UBX_SYNC1) {
        gps_ubx.status++;
        gps_ubx.len = 0;
      }
      break;
    case GOT_SYNC1:
      if (c != GPS_UBX_SYNC2) {
        if(verbose) printf("GPS Ublox parser out of sync\r\n");
        goto error;
      }
      gps_ubx.ck_a = 0;
      gps_ubx.ck_b = 0;
      gps_ubx.status++;
      break;
    case GOT_SYNC2:
      if (gps_ubx.msg_available) {
        /* Previous message has not yet been parsed: discard this one */
        if(verbose) printf("GPS Ublox parser overrun\r\n");
        goto error;
      }
      gps_ubx.msg_class = c;
      gps_ubx.status++;
      break;
    case GOT_CLASS:
      gps_ubx.msg_id = c;
      gps_ubx.status++;
      break;
    case GOT_ID:
      gps_ubx.len = c;
      gps_ubx.status++;
      break;
    case GOT_LEN1:
      gps_ubx.len |= (c << 8);
      if (gps_ubx.len > GPS_UBX_MAX_PAYLOAD) {
        gps_ubx.len = 0;
        if(verbose) printf("GPS Ublox message to long\r\n");
        goto error;
      }
      gps_ubx.msg_idx = 0;
      gps_ubx.status++;
      break;
    case GOT_LEN2:
      gps_ubx.msg_buf[gps_ubx.msg_idx] = c;
      gps_ubx.msg_idx++;
      if (gps_ubx.msg_idx >= gps_ubx.len) {
        gps_ubx.status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != gps_ubx.ck_a) {
        if(verbose) printf("GPS Ublox parser checksum error\r\n");
        goto error;
      }
      gps_ubx.status++;
      break;
    case GOT_CHECKSUM1:
      if (c != gps_ubx.ck_b) {
        if(verbose) printf("GPS Ublox parser checksum error\r\n");
        goto error;
      }
      gps_ubx.msg_available = true;
      goto restart;
      break;
    default:
      if(verbose) printf("GPS Ublox parser unexpected error\r\n");
      goto error;
  }
  return;
error:
  gps_ubx.error_cnt++;

  /* Try to resync */
  if(gps_ubx.len > 0) {
    const uint8_t *p = (const uint8_t *)memchr(gps_ubx.msg_buf, GPS_UBX_SYNC1, gps_ubx.len);
    if(p != NULL) {
      if(verbose) printf("GPS Ublox parser resynced\r\n");
      uint16_t len_diff = p - gps_ubx.msg_buf;
      uint16_t len = gps_ubx.len - len_diff;
      uint8_t msg_buf[len];
      memcpy(msg_buf, p, len);

      /* Recursion (chances of extremely deep recursion are really low) */
      gps_ubx.status = UNINIT;
      for(uint16_t i = 0; i < len; i++) {
        gps_ubx_parse(msg_buf[i]);

        /* If two messages are embedded, the second one is discared */
        if(gps_ubx.msg_available)
          return;
      }
      return;
    }
  }
restart:
  gps_ubx.status = UNINIT;
  return;
}

static void send_gps_inject(uint8_t *buf, int32_t len) {
  const uint8_t max_len = 250; /* Remove PPRZLink overhead for maximum packet size */
  uint16_t send_len = len % max_len;
  uint16_t idx = 0;

  while(len > 0) {
    if(verbose) printf("Sending RTCM over ivy [%d, %d | %d, %d]\r\n", RTCMgetbitu(gps_rtcm.msg_buf, 24 + 0, 12), gps_rtcm.len+6, idx, send_len);
    char gps_packet[1024], number[5];
    snprintf(gps_packet, 1024, "ground GPS_INJECT %d %d %d", ac_id, 0, gps_rtcm.msg_buf[idx]);
    for(uint16_t i = idx+1; i < idx+send_len; i++) {
      snprintf(number, 5, ",%d", gps_rtcm.msg_buf[i]);
      strcat(gps_packet, number);
    }
    IvySendMsg("%s", gps_packet);

    len -= send_len;
    send_len = len % max_len;
  }
}

/**
 * Handle incoming packets from the GPS
 */
void packet_handler(void *ep, uint8_t *data, uint16_t len) {
  /* Try to parse it as UBX message */
  for(uint16_t i = 0; i < len; i++) {
    gps_ubx_parse(data[i]);
    /* Whenever we have a succesful message */
    if(gps_ubx.msg_available) {
      if(verbose) printf("Got a succesfull UBX message [%d]\r\n", gps_ubx.msg_id);

      /* Try to parse messages */
      switch(gps_ubx.msg_id) {
        case UBX_NAV_PVT_ID: {
          double lat = UBX_NAV_PVT_lat(gps_ubx.msg_buf) / 1e7;
          double lon = UBX_NAV_PVT_lon(gps_ubx.msg_buf) / 1e7;
          float gSpeed = UBX_NAV_PVT_gSpeed(gps_ubx.msg_buf) / 1000.;
          float headMot = UBX_NAV_PVT_headMot(gps_ubx.msg_buf) / 1e5;
          float alt = UBX_NAV_PVT_height(gps_ubx.msg_buf) / 1000.;
          float velD = UBX_NAV_PVT_velD(gps_ubx.msg_buf) / 1000.;
          uint32_t iTOW = UBX_NAV_PVT_iTOW(gps_ubx.msg_buf);

          if(verbose) printf("Got position %f %f (%f, %f, %f, %f, %d)\r\n", lat, lon, gSpeed, headMot, alt, velD, iTOW);
          IvySendMsg("ground FLIGHT_PARAM GCS %f %f %f %f %f %f %f %f %f %f %f %d %f",
                0.0, // roll,
                0.0, // pitch,
                0.0, // heading
                lat,
                lon,
                gSpeed,
                headMot, // course
                alt,
                -velD,
                0.0, // agl
                (float)time(NULL),
                iTOW, // itow
                0.0); // airspeed

            IvySendMsg("ground TARGET_POS %d %d %d %d %d %f %f %f",
                ac_id,
                ac_id,
                (int)(lat * 1e7),
                (int)(lon * 1e7),
                (int)(alt * 1000),
                gSpeed,
                -velD,
                headMot);
          break;
        }
        default:
          break;
      }
      gps_ubx.msg_available = false;
    }
  }

  /* Try to parse it as RTCM message */
    for(uint16_t i = 0; i < len; i++) {
    gps_rtcm_parse(data[i]);
    /* Whenever we have a succesful message */
    if(gps_rtcm.msg_available) {
      if(verbose) printf("Got a succesfull RTCM message [%d]\r\n", RTCMgetbitu(gps_rtcm.msg_buf, 24 + 0, 12));

      /* Forward the message to inject into the drone */
      send_gps_inject(gps_rtcm.msg_buf, gps_rtcm.len + 6);

      gps_rtcm.msg_available = false;
    }
  }
}

int main(int argc, char** argv) {
  /* Defaults */
  gps_ep.type = ENDPOINT_TYPE_NONE;
#ifdef __APPLE__
  char* ivy_bus = "224.255.255.255";
#else
  char *ivy_bus = "127.255.255.255";
#endif

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("UBLOX2Ivy", "UBLOX2Ivy READY", NULL, NULL, NULL, NULL);
  IvyStart(ivy_bus);

  /* Arguments options and usage information */
  static struct option long_options[] = {
    {"endpoint", required_argument, NULL, 'e'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {0, 0, 0, 0}
  };
  static const char* usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -e --endpoint [endpoint_str]           Endpoint address of the GPS\n"
    "   -h --help                              Display this help\n"
    "   -v --verbose                           Print verbose information\n";

  int c;
  int option_index = 0;
  while((c = getopt_long(argc, argv, "e:hv", long_options, &option_index)) != -1) {
    switch (c) {
      case 'e':
        // Parse the endpoint argument UDP
        if(!strncmp(optarg, "udp", 3)) {
          char serv_addr[125], cli_addr[125];
          uint16_t serv_port, cli_port;
          if(sscanf(optarg, "udp://%[^:]:%hu:%[^:]:%hu", serv_addr, &serv_port, cli_addr, &cli_port) != 4) {
            fprintf(stderr, "UDP endpoint %s has incorrect arguments\r\n", optarg);
            return 2;
          }
          udp_create(serv_addr, serv_port, cli_addr, cli_port);
        }
        // Parse the endpoint argument UART
        else if(!strncmp(optarg, "uart", 4)) {
          char devname[256];
          uint32_t baudrate;
          if(sscanf(optarg, "uart://%[^:]:%u", devname, &baudrate) != 2) {
            fprintf(stderr, "UART endpoint %s has incorrect arguments\r\n", optarg);
            return 2;
          }
          uart_create(devname, baudrate);
        }
        else {
          fprintf(stderr, "Endpoint %s has incorrect type only uart and udp are supported\r\n", optarg);
          return 2;
        }
        break;

      case 'v':
        verbose = true;
        break;

      case 'h':
        fprintf(stderr, usage, argv[0]);
        return 0;

      default: /* ’?’ */
        printf("?? getopt returned character code %c ??\r\n", c);
        fprintf(stderr, usage, argv[0]);
        return 1;
    }
  }

  g_main_loop_run(ml);

  /*while(true) {
    usleep(50000);
  }*/

  return 0;
}