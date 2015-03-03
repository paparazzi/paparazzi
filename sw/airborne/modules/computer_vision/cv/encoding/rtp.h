/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/cv/encoding/rtp.h
 *
 * Encodes a vide stream with RTP (JPEG)
 */

#ifndef _CV_ENCODING_RTP_H
#define _CV_ENCODING_RTP_H

#include "std.h"
#include "mcu_periph/udp.h"

void rtp_frame_send(
  struct udp_periph *udp,             // socket
  uint8_t *Jpeg, uint32_t JpegLen,    // jpeg data
  int w, int h,                       // width and height
  uint8_t format_code,                // 0=422, 1=421
  uint8_t quality_code,               // 0-99 of 128 for custom (include
  uint8_t has_dri_header,             // Does Jpeg data include Header Info?
  uint32_t delta_t                    // time step 90kHz
);

void rtp_frame_test(struct udp_periph *udp);

#endif /* _CV_ENCODING_RTP_H */
