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
 * @file MsgPacket.h
 *
 * HITL demo version - general packet configuration
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef MSGPACKET_H_
#define MSGPACKET_H_

#include "../src/MsgConfig.h"

/**
 * Packet
 * contains variables for packet parsing
 */
class MsgPacket
{
public:
  bool  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[BUF_SIZE] = {0};
  uint16_t datalength;
  enum MsgStatus  status;
  uint16_t  msg_idx;

  MsgPacket() {
    msg_available = false;
    chksm_error = 0;
    hdr_error = 0;
    datalength = HITL_DATALENGTH;
    status = MsgSync0;
    msg_idx = 0;
  }
};


#endif /* MSGPACKET_H_ */
