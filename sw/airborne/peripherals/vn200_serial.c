/*
 * Copyright (C) 2015 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
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
 * @file vn200_serial.c
 *
 * Vectornav VN-200 INS subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "peripherals/vn200_serial.h"


/**
 * Calculates the 16-bit CRC for the given ASCII or binary message.
 * The CRC is calculated over the packet starting just after the sync byte (not including the sync byte)
 * and ending at the end of payload.
 */
static inline unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for (i = 0; i < length; i++) {
    crc = (unsigned char)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (unsigned char)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}


/**
 * Verify checksum
 */
static inline bool verify_chk(unsigned char data[], unsigned int length, uint16_t *calc_chk, uint16_t *rec_chk)
{
  unsigned short calc_crc = calculateCRC(data, length);
  unsigned short rec_crc = (unsigned short)(data[length] << 8 | data[length + 1]);
  *calc_chk = (uint16_t) calc_crc;
  *rec_chk = (uint16_t) rec_crc;
  if (calc_crc == rec_crc) {
    return 1;
  } else {
    return 0;
  }
}


static inline void vn200_read_buffer(struct VNPacket *vnp)
{
  while (uart_char_available(&VN_PORT) && !(vnp->msg_available)) {
    vn200_parse(vnp, uart_getch(&VN_PORT));
  }
}


void vn200_event(struct VNPacket *vnp)
{
  if (uart_char_available(&VN_PORT)) {
    vn200_read_buffer(vnp);
  }
}

/**
 *  Packet Collection & state machine
 */
void vn200_parse(struct VNPacket *vnp, uint8_t c)
{
  switch (vnp->status) {
    case VNMsgSync:
      // sync the header
      vnp->msg_idx = 0;
      if (c == VN_SYNC) {
        vnp->status = VNMsgHeader;
      } else {
        vnp->hdr_error++;
      }
      break;
    case VNMsgHeader:
      // read header data (we expect 0x39)
      if (c == VN_OUTPUT_GROUP) {
        // increment idx and save current byte for checksum
        vnp->status = VNMsgGroup;
        vnp->msg_buf[vnp->msg_idx] = c;
        vnp->msg_idx++;
      } else {
        vnp->hdr_error++;
        vnp->status = VNMsgSync;
      }
      break;
      break;
    case VNMsgGroup:
      // read header data
      vnp->msg_buf[vnp->msg_idx] = c;
      vnp->msg_idx++;
      if (vnp->msg_idx == VN_GROUP_BYTES) {
        vnp->datalength = VN_PAYLOAD_SIZE + VN_HEADER_SIZE;
        vnp->status = VNMsgData;
      }
      break;
    case VNMsgData:
      vnp->msg_buf[vnp->msg_idx] =  c;
      vnp->msg_idx++;
      if (vnp->msg_idx == (vnp->datalength + 2)) {
        if (verify_chk(vnp->msg_buf, vnp->datalength, &(vnp->calc_chk), &(vnp->rec_chk))) {
          vnp->msg_available = TRUE;
          vnp->counter++;
        } else {
          vnp->msg_available = FALSE;
          vnp->chksm_error++;
        }
        vnp->status = VNMsgSync;
      }
      break;
    default:
      vnp->status = VNMsgSync;
      vnp->msg_idx = 0;
      break;
  }
}
