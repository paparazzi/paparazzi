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
 * @file vn200_serial.h
 *
 * Vectornav VN-200 INS subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef VN200_SERIAl_H
#define VN200_SERIAl_H

#include "std.h"
#include "mcu_periph/uart.h"

/*
 * Defines for the serial communication
 */
#define VN_SYNC 0xFA
#define VN_OUTPUT_GROUP 0x39
#define VN_GROUP_BYTES 8

#define VN_BUFFER_SIZE 512
#define VN_HEADER_SIZE 9
#define VN_PAYLOAD_SIZE 144

#define DEG_TO_RAD 0.017453292519943


enum VNMsgStatus {
  VNMsgSync,
  VNMsgHeader,
  VNMsgGroup,
  VNMsgData,
  VNMsgCheck
};

struct VNPacket {
  bool_t  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[VN_BUFFER_SIZE];
  enum VNMsgStatus status;
  uint8_t  msg_idx;
  uint16_t datalength;
  uint16_t overrun_error;
  uint16_t noise_error;
  uint16_t framing_error;
  uint16_t calc_chk;
  uint16_t rec_chk;
  uint16_t counter;
};

enum VNStatus {
  VNNotTracking,
  VNOutOfSpecs,
  VNOK
};

void vn200_event(struct VNPacket *vnp);
void vn200_read_message(void);
void vn200_parse(struct VNPacket *vnp, uint8_t c);

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


#endif /* VN200_SERIAl_H */
