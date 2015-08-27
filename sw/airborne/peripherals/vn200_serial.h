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


#endif /* VN200_SERIAl_H */
