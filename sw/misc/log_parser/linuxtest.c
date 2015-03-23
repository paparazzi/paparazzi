/*
 * Copyright (C) 2003-2009 Pascal Brisset, Antoine Drouin, Martin Mueller
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

  /* XBee-message: ABCDxxxxxxxE
     A XBEE_START (0x7E)
     B LENGTH_MSB (D->D)
     C LENGTH_LSB
     D XBEE_PAYLOAD
       0 XBEE_TX16 (0x01) / XBEE_RX16 (0x81)
       1 FRAME_ID (0)     / SRC_ID_MSB
       2 DEST_ID_MSB      / SRC_ID_LSB
       3 DEST_ID_LSB      / XBEE_RSSI
       4 TX16_OPTIONS (0) / RX16_OPTIONS
       5 PPRZ_DATA
         0 SENDER_ID
         1 MSG_ID
         2 MSG_PAYLOAD
         . DATA (messages.xml)
     E XBEE_CHECKSUM (sum[A->D])

    ID is AC_ID for aircraft, 0x100 for ground station
  */

  /* PPRZ-message: ABCxxxxxxxDE
     A PPRZ_STX (0x99)
     B LENGTH (A->E)
     C PPRZ_DATA
       0 SENDER_ID
       1 MSG_ID
       2 MSG_PAYLOAD
       . DATA (messages.xml)
     D PPRZ_CHECKSUM_A (sum[B->C])
     E PPRZ_CHECKSUM_B (sum[ck_a])
  */

  /* LOG-message: ABCDEFGHxxxxxxxI
     A PPRZ_STX (0x99)
     B LENGTH (H->H)
     C SOURCE (0=uart0, 1=uart1, 2=i2c0, ...)
     D TIMESTAMP_LSB (100 microsec raster)
     E TIMESTAMP
     F TIMESTAMP
     G TIMESTAMP_MSB
     H PPRZ_DATA
       0 SENDER_ID
       1 MSG_ID
       2 MSG_PAYLOAD
       . DATA (messages.xml)
     I CHECKSUM (sum[B->H])
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdarg.h>
#include <termios.h>
#include <unistd.h>

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

/** Constants for the API protocol */
#define XBEE_START 0x7e
#define XBEE_TX16_ID 0x01
#define TX16_OPTIONS 0x00
#define NO_FRAME_ID 0
#define XBEE_RFDATA_OFFSET 5
#define XBEE_RX16_ID 0x81

/** Status of the API packet receiver automata */
#define XBEE_UNINIT 0
#define XBEE_GOT_START 1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD 4
#define XBEE_PAYLOAD_LEN 256

/** Receiving pprz messages */
#define STX  0x99
#define UNINIT 0
#define GOT_STX 1
#define GOT_LENGTH 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4
#define PPRZ_PAYLOAD_LEN 256
#define PPRZ_DATA_OFFSET 2

/** logging messages **/
#define LOG_DATA_OFFSET 7
#define MSG_SIZE 256

unsigned char checksum(unsigned char start, unsigned char* data, int length);
void log_payload(int len, unsigned char source, unsigned int timestamp);
void log_xbee(unsigned char c);
void log_pprz(unsigned char c);

unsigned char xbeel_payload[XBEE_PAYLOAD_LEN];
unsigned char pprzl_payload[PPRZ_PAYLOAD_LEN];
volatile unsigned char xbeel_payload_len;
volatile unsigned char pprzl_payload_len;
unsigned char xbeel_error = 0;
unsigned char pprzl_error = 0;
unsigned char log_buffer[MSG_SIZE]  __attribute__ ((aligned));
static unsigned int xbeel_timestamp = 0;
static unsigned int pprzl_timestamp = 0;
unsigned int nb_messages = 0;
unsigned int nb_fail_write = 0;
int bytes_in = 0;
int bytes_out = 0;

//dummy
int fdw;
unsigned int filew;
int file_write (void* file, int size, unsigned char* buf)
{
    return write(fdw, buf, size);
}
//dummy

unsigned char checksum(unsigned char start, unsigned char* data, int length)
{
    int i;
    unsigned char retval = start;
    for (i=0;i<length;i++) retval += data[i];

    return retval;
}

/** Parsing a frame data and copy the payload to the log buffer */
void log_payload(int len, unsigned char source, unsigned int timestamp)
{
  unsigned char chk, i;

  /* start delimiter */
  log_buffer[0] = STX;

  /* length is just payload */
  log_buffer[1] = len & 0xFF;

  /* source */
  log_buffer[2] = source;

  /* add a 32bit timestamp */
  log_buffer[3] = (timestamp) & 0xFF;       // LSB first
  log_buffer[4] = (timestamp >> 8)  & 0xFF;
  log_buffer[5] = (timestamp >> 16) & 0xFF;
  log_buffer[6] = (timestamp >> 24) & 0xFF;

  /* data is already written */

  /* calculate checksum over start+length+timestamp+data */
  log_buffer[LOG_DATA_OFFSET+len] = checksum(0, &log_buffer[1], LOG_DATA_OFFSET+len-1);

    printf(">");
    printf("%02X:", log_buffer[1]);
    printf("SRC%02X|", log_buffer[2]);
    printf("TS%02X%02X%02X%02x|", log_buffer[3], log_buffer[4], log_buffer[5], log_buffer[6]);
    printf("S%02X:", log_buffer[7]);
    printf("M%02X:", log_buffer[8]);
    for (i=0; i<len-2; i++) printf("%02X", log_buffer[9+i]);
    printf(":%02X<\n", chk);

  /* write data, start+length+timestamp+data+checksum */
  chk = file_write(&filew, LOG_DATA_OFFSET+len+1, log_buffer);

  if (len != chk)
  {
    nb_fail_write++;
  }

  bytes_out += chk;
  nb_messages++;
//  dl_parse_msg();
}

/** Parsing a XBee API frame */
void log_xbee(unsigned char c)
{
  static unsigned char xbeel_status = XBEE_UNINIT;
  static unsigned char cs, payload_idx, i;

  switch (xbeel_status) {
  case XBEE_UNINIT:
    if (c == XBEE_START)
    {
      xbeel_timestamp++;
      xbeel_status++;
      printf(">");
    }
    break;
  case XBEE_GOT_START:
    xbeel_payload_len = c<<8;
    xbeel_status++;
    break;
  case XBEE_GOT_LENGTH_MSB:
    xbeel_payload_len |= c;
    xbeel_status++;
    payload_idx = 0;
    cs=0;
    printf("%04X:", xbeel_payload_len);
    break;
  case XBEE_GOT_LENGTH_LSB:
    xbeel_payload[payload_idx] = c;
    cs += c;
    payload_idx++;
    if (payload_idx == xbeel_payload_len)
      xbeel_status++;
    break;
  case XBEE_GOT_PAYLOAD:
    if (c + cs != 0xff)
      goto error;
    if ((xbeel_payload[0] != XBEE_RX16_ID) &&
        (xbeel_payload[0] != XBEE_TX16_ID))
      goto error;

    printf("ID%02X|", xbeel_payload[0]);
    printf("S%02X:", xbeel_payload[5]);
    printf("M%02X:", xbeel_payload[6]);
    for (i=7; i<xbeel_payload_len; i++) printf("%02X", xbeel_payload[i]);
    printf(":%02X<\n", c);

    /* copy the XBee message to the logger buffer */
    for (i = 0; i < xbeel_payload_len-XBEE_RFDATA_OFFSET; i++) {
      log_buffer[i+LOG_DATA_OFFSET] = xbeel_payload[i+XBEE_RFDATA_OFFSET];
    }
    log_payload(xbeel_payload_len-XBEE_RFDATA_OFFSET, 0, xbeel_timestamp);
    goto restart;
  }
  return;
 error:
  printf("?");
  xbeel_error++;
 restart:
  xbeel_status = XBEE_UNINIT;
  return;
}

void log_pprz(unsigned char c)
{
  static unsigned char pprzl_status = UNINIT;
  static unsigned char _ck_a, _ck_b, payload_idx, i;

  switch (pprzl_status) {
  case UNINIT:
    if (c == STX)
      pprzl_timestamp++;
      pprzl_status++;
      printf(">");
    break;
  case GOT_STX:
    pprzl_payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
    _ck_a = _ck_b = c;
    pprzl_status++;
    payload_idx = 0;
    printf("%02X:", pprzl_payload_len);
    break;
  case GOT_LENGTH:
    pprzl_payload[payload_idx] = c;
    _ck_a += c; _ck_b += _ck_a;
    payload_idx++;
    if (payload_idx == pprzl_payload_len)
      pprzl_status++;
    break;
  case GOT_PAYLOAD:
    if (c != _ck_a)
      goto error;
    pprzl_status++;
    break;
  case GOT_CRC1:
    if (c != _ck_b)
      goto error;

    printf("S%02X:", pprzl_payload[0]);
    printf("M%02X:", pprzl_payload[1]);
    for (i=2; i<pprzl_payload_len; i++) printf("%02X", pprzl_payload[i]);
    printf(":%02X%02X<\n", _ck_a, _ck_b);

    /* copy the pprz message to the logger buffer */
    for (i = 0; i < pprzl_payload_len; i++) {
      log_buffer[i+LOG_DATA_OFFSET] = pprzl_payload[i];
    }
    log_payload(pprzl_payload_len, 0, pprzl_timestamp);
    goto restart;
  }
  return;
 error:
  pprzl_error++;
 restart:
  pprzl_status = UNINIT;
  return;
}


int main(void)
{
    int fd;
    unsigned char inc;

    fd = open("xbee.bin", O_RDONLY);
//    fd = open("pprz.bin", O_RDONLY);
    if(fd < 0)
    {
        perror("open file");
        exit(1);
    }

    fdw = open("xbee.log", O_CREAT|O_WRONLY);
//    fdw = open("pprz.log", O_CREAT|O_WRONLY);
    if(fdw < 0)
    {
        perror("open write file");
        exit(1);
    }

    if(fd < 0)
    {
        perror("open file");
        exit(1);
    }

    while(read(fd, &inc, 1) == 1)
    {
//        printf("0x%02X\n", inc);
        log_xbee(inc);
//        log_pprz(inc);
        bytes_in++;
    }

    close(fdw);
    close(fd);

    printf("\nmsg detected %d\nbytes in %d, out %d\nerr xbee %d, pprz %d\n",
           nb_messages,
           bytes_in,
           bytes_out,
           xbeel_error,
           pprzl_error);

	return(0);
}

