/*
 * Paparazzi email attachment to udp handling for sat based telemetry
 *
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
 *
 * RFC 1521 base64 decoding
 * Copyright (C) 2006-2010, Brainspark B.V.
 * Lead Maintainer: Paul Bakker <polarssl_maintainer at polarssl.org>
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


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <stropts.h>


#define PORT 7023
#define HOSTADDR "localhost"

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

unsigned char base64_string[] = { "Content-Transfer-Encoding: base64" };

#define MAX_LINE_LEN    1024
#define MAX_DATA_LEN    65536

char line[MAX_LINE_LEN];
unsigned char buf[MAX_DATA_LEN];
unsigned char out[MAX_DATA_LEN];
char *eof;

static const unsigned char base64_dec_map[128] =
{
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127,  62, 127, 127, 127,  63,  52,  53,
     54,  55,  56,  57,  58,  59,  60,  61, 127, 127,
    127,  64, 127, 127, 127,   0,   1,   2,   3,   4,
      5,   6,   7,   8,   9,  10,  11,  12,  13,  14,
     15,  16,  17,  18,  19,  20,  21,  22,  23,  24,
     25, 127, 127, 127, 127, 127, 127,  26,  27,  28,
     29,  30,  31,  32,  33,  34,  35,  36,  37,  38,
     39,  40,  41,  42,  43,  44,  45,  46,  47,  48,
     49,  50,  51, 127, 127, 127, 127, 127
};

/*
 * Decode a base64-formatted buffer
 */
int base64_decode( unsigned char *dst, int *dlen,
                   const unsigned char *src, int  slen )
{
    int i, j, n;
    unsigned long x;
    unsigned char *p;

    for( i = j = n = 0; i < slen; i++ )
    {
        if( ( slen - i ) >= 2 &&
            src[i] == '\r' && src[i + 1] == '\n' )
            continue;

        if( src[i] == '\n' )
            continue;

        if( src[i] == '=' && ++j > 2 )
            return( -1 );

        if( src[i] > 127 || base64_dec_map[src[i]] == 127 )
            return( -2 );

        if( base64_dec_map[src[i]] < 64 && j != 0 )
            return( -3 );

        n++;
    }

    if( n == 0 )
        return( 0 );

    n = ((n * 6) + 7) >> 3;

    if( *dlen < n )
    {
        *dlen = n;
        return( -4 );
    }

   for( j = 3, n = x = 0, p = dst; i > 0; i--, src++ )
   {
        if( *src == '\r' || *src == '\n' )
            continue;

        j -= ( base64_dec_map[*src] == 64 );
        x  = (x << 6) | ( base64_dec_map[*src] & 0x3F );

        if( ++n == 4 )
        {
            n = 0;
            if( j > 0 ) *p++ = (unsigned char)( x >> 16 );
            if( j > 1 ) *p++ = (unsigned char)( x >>  8 );
            if( j > 2 ) *p++ = (unsigned char)( x       );
        }
    }

    *dlen = p - dst;

    return( 0 );
}

int base64_invalid(char *src, int  slen ) {
  int i;

  for( i=0; i < slen; i++ ) {
    if( ( slen - i ) >= 2 && src[i] == '\r' && src[i + 1] == '\n' )
      return 0;
    if( src[i] == '\n' )
      return 0;
    if(src[i] > 127 || base64_dec_map[(unsigned char)src[i]] == 127)
      return 1;
  }
  return 0;
}

int base64_crlf(char *src, int  slen ) {
  if (slen >= 2 && src[0] == '\r' && src[1] == '\n' )
    return 1;
  if( src[0] == '\n' )
    return 1;

  return 0;
}

int main( int argc, char** argv) {
  int len, len_out, total_len = 0;
  struct hostent *hent;
  int sock, length;
  struct sockaddr_in server;
  char hostaddr[4096] = {HOSTADDR};

  line[0] = '\0';
  line[sizeof(line)-1] = ~'\0';

  /* setup UDP */
  sock = socket(AF_INET, SOCK_DGRAM, 0);
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

  /* search for base64 start */
  do {
    eof = fgets(line, sizeof(line), stdin);
    if (strncmp((const char*)line, (const char*)base64_string, sizeof(base64_string)-1) == 0) break;
  } while (eof != NULL);
  if (eof == NULL) return 0;

  /* search for empty line */
  do {
    eof = fgets(line, sizeof(line), stdin);
    len = strnlen(line, sizeof(line));
    if (base64_crlf(line, len) != 0) break;
  } while (eof != NULL);
  if (eof == NULL) return 0;

  /* read base64 data */
  do {
    /* exit on empty line and invalid character */
    eof = fgets(line, sizeof(line), stdin);
    len = strnlen(line, sizeof(line));
    if ((base64_crlf(line, len) == 1) || (base64_invalid(line, len) == 1)) break;
    if ((total_len+len) > sizeof(buf)) return 0;

    /* fill buffer */
    memcpy(buf+total_len, line, len);
    total_len += len;
  } while (eof != NULL);
  if (eof == NULL) return 0;

  len_out = sizeof(out);

  if (base64_decode(out, &len_out, buf, total_len) != 0) return 0;
//  for (len=0; len<len_out; len++) printf("%c", out[i]);

  /* send data as binary through UDP */
  len = sendto(sock, out, len_out, 0, (struct sockaddr*) &server, length);
  if (len != len_out) perror("sendto");

  close (sock);

  return 0;
}
