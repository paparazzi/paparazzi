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
 * @file modules/computer_vision/lib/encoding/rtp.h
 *
 * Encodes a video stream with RTP Format 26 (Motion JPEG)
 */

#ifndef _CV_ENCODING_RTP_H
#define _CV_ENCODING_RTP_H

#include "std.h"
#include "lib/vision/image.h"
#include "udp_socket.h"

void rtp_frame_send(struct UdpSocket *udp, struct image_t *img, uint8_t format_code, uint8_t quality_code,
                    uint8_t has_dri_header, float average_frame_rate, uint16_t *packet_number, uint32_t *rtp_time_counter);
void rtp_frame_test(struct UdpSocket *udp);

#endif /* _CV_ENCODING_RTP_H */
