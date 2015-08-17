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
    default:;
      vnp->status = VNMsgSync;
      vnp->msg_idx = 0;
      break;
  }
}
