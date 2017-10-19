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
 * @file modules/computer_vision/lib/encoding/rtp.c
 *
 * Encodes a vide stream with RTP (JPEG)
 */

#include <stdint.h>
#include <string.h>
#include <sys/time.h>

#include "rtp.h"

static void rtp_packet_send(struct UdpSocket *udp, uint8_t *Jpeg, int JpegLen, uint16_t m_SequenceNumber,
                            uint32_t m_Timestamp, uint32_t m_offset, uint8_t marker_bit, int w, int h, uint8_t format_code, uint8_t quality_code,
                            uint8_t has_dri_header);

/*
 * RTP Protocol documentation
 *
 * Full description:
 * - http://www.ietf.org/rfc/rfc3550.txt
 *
 * Packet content:
 * - https://tools.ietf.org/html/rfc3550#section-5.1
 *
 * Format specific details:
 * - https://en.wikipedia.org/wiki/RTP_audio_video_profile
 * - protocol 26 (Jpeg) has a clock rate of 90 000
 *
 * The timestamp reflects the sampling instant of the first octet in
 * the RTP data packet. The sampling instant MUST be derived from a
 * clock that increments monotonically and linearly in time to allow
 * synchronization and jitter calculations (see Section 6.4.1). The
 * resolution of the clock MUST be sufficient for the desired
 * synchronization accuracy and for measuring packet arrival jitter
 * (one tick per video frame is typically not sufficient). The clock
 * frequency is dependent on the format of data carried as payload
 * and is specified statically in the profile or payload format
 * specification that defines the format, or MAY be specified
 * dynamically for payload formats defined through non-RTP means. If
 * RTP packets are generated periodically, the nominal sampling
 * instant as determined from the sampling clock is to be used, not a
 * reading of the system clock.
 */


#define KJpegCh1ScanDataLen 32
#define KJpegCh2ScanDataLen 56

// RGB JPEG images as RTP payload - 64x48 pixel
uint8_t JpegScanDataCh2A[KJpegCh2ScanDataLen] = {
  0xf8, 0xbe, 0x8a, 0x28, 0xaf, 0xe5, 0x33, 0xfd,
  0xfc, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x3f, 0xff, 0xd9
};
uint8_t JpegScanDataCh2B[KJpegCh2ScanDataLen] = {
  0xf5, 0x8a, 0x28, 0xa2, 0xbf, 0xca, 0xf3, 0xfc,
  0x53, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x0a, 0x28, 0xa2,
  0x80, 0x0a, 0x28, 0xa2, 0x80, 0x3f, 0xff, 0xd9
};

/**
 * Send a test RTP frame
 * @param[in] *udp The udp connection to send the test frame over
 */
void rtp_frame_test(struct UdpSocket *udp)
{
  static uint32_t framecounter = 0;
  static uint32_t timecounter = 0;
  static uint8_t toggle = 0;
  toggle = ! toggle;

  uint8_t format_code = 0x01;
  uint8_t quality_code = 0x54;

  if (toggle) {
    rtp_packet_send(udp, JpegScanDataCh2A, KJpegCh2ScanDataLen, framecounter, timecounter, 0, 1, 64, 48, format_code,
                    quality_code, 0);
  } else {
    rtp_packet_send(udp, JpegScanDataCh2B, KJpegCh2ScanDataLen, framecounter, timecounter, 0, 1, 64, 48, format_code,
                    quality_code, 0);
  }
  framecounter++;
  timecounter += 3600;
}

/**
 * Send an RTP frame
 * @param[in] *udp The UDP connection to send the frame over
 * @param[in] *img The image to send over the RTP connection
 * @param[in] format_code 0 for YUV422 and 1 for YUV421
 * @param[in] quality_code The JPEG encoding quality
 * @param[in] has_dri_header Whether we have an DRI header or not
 * @param[in] frame_time Time image was taken in usec (if set to 0 or less it is calculated)
 * @param[out] packet_number The frame number of the rtp stream
 * @param[out] rtp_time_counter The frame time counter of the rtp stream
 */
void rtp_frame_send(struct UdpSocket *udp, struct image_t *img, uint8_t format_code,
                    uint8_t quality_code, uint8_t has_dri_header, float average_frame_rate, uint16_t *packet_number, uint32_t *rtp_time_counter)
{
  uint32_t offset = 0;
  uint32_t jpeg_size = img->buf_size;
  uint8_t *jpeg_ptr = img->buf;

  *rtp_time_counter += ((uint32_t) (90000.0f / average_frame_rate));

#define MAX_PACKET_SIZE 1400

  // Split frame into packets
  for (; jpeg_size > 0;) {
    uint32_t len = MAX_PACKET_SIZE;
    uint8_t lastpacket = 0;

    if (jpeg_size <= len) {
      lastpacket = 1;
      len = jpeg_size;
    }

    rtp_packet_send(udp, jpeg_ptr, len, *packet_number, *rtp_time_counter, offset, lastpacket, img->w, img->h, format_code,
                    quality_code, has_dri_header);

    (*packet_number)++;
    jpeg_size -= len;
    jpeg_ptr  += len;
    offset    += len;
  }

}

/*
 * The same timestamp MUST appear in each fragment of a given frame.
 * The RTP marker bit MUST be set in the last packet of a frame.
 * Extra note: When the time difference between frames is non-constant,
   there seems to introduce some lag or jitter in the video streaming.
 * @param[in] *udp The UDP socket to send the RTP packet over
 * @param[in] *Jpeg JPEG encoded image byte buffer
 * @param[in] JpegLen The length of the byte buffer
 * @param[in] m_SequenceNumber RTP sequence number
 * @param[in] m_Timestamp Time counter: RTP requires monolitically lineraly increasing timecount. FMT26 uses 90kHz clock.
 * @param[in] m_offset 3 byte fragmentation offset for fragmented images
 * @param[in] marker_bit RTP marker bit: must be set in last packet of a frame.
 * @param[in] w The width of the JPEG image
 * @param[in] h The height of the image
 * @param[in] format_code 0 for YUV422 and 1 for YUV421
 * @param[in] quality_code The JPEG encoding quality
 * @param[in] has_dri_header Whether we have an DRI header or not
 */
static void rtp_packet_send(
  struct UdpSocket *udp,
  uint8_t *Jpeg, int JpegLen,
  uint16_t m_SequenceNumber, uint32_t m_Timestamp,
  uint32_t m_offset, uint8_t marker_bit,
  int w, int h,
  uint8_t format_code, uint8_t quality_code,
  uint8_t has_dri_header)
{

#define KRtpHeaderSize 12           // size of the RTP header
#define KJpegHeaderSize 8           // size of the special JPEG payload header

  uint8_t     RtpBuf[2048];
  int         RtpPacketSize = JpegLen + KRtpHeaderSize + KJpegHeaderSize;

  memset(RtpBuf, 0x00, sizeof(RtpBuf));

  /*
   The RTP header has the following format:

    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |V=2|P|X|  CC   |M|     PT      |       sequence number         |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                           timestamp                           |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |           synchronization source (SSRC) identifier            |
   +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
   |            contributing source (CSRC) identifiers             |
   |                             ....                              |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   * */

  // Prepare the 12 byte RTP header
  RtpBuf[0]  = 0x80;                               // RTP version
  RtpBuf[1]  = 0x1a + (marker_bit << 7);           // JPEG payload (26) and marker bit
  RtpBuf[2]  = m_SequenceNumber >> 8;
  RtpBuf[3]  = m_SequenceNumber & 0x0FF;           // each packet is counted with a sequence counter
  RtpBuf[4]  = (m_Timestamp & 0xFF000000) >> 24;   // each image gets a timestamp
  RtpBuf[5]  = (m_Timestamp & 0x00FF0000) >> 16;
  RtpBuf[6]  = (m_Timestamp & 0x0000FF00) >> 8;
  RtpBuf[7]  = (m_Timestamp & 0x000000FF);
  RtpBuf[8]  = 0x13;                               // 4 byte SSRC (sychronization source identifier)
  RtpBuf[9]  = 0xf9;                               // we just an arbitrary number here to keep it simple
  RtpBuf[10] = 0x7e;
  RtpBuf[11] = 0x67;

  /* JPEG header", are as follows:
   *
   * http://tools.ietf.org/html/rfc2435

    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   | Type-specific |              Fragment Offset                  |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |      Type     |       Q       |     Width     |     Height    |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   */

  // Prepare the 8 byte payload JPEG header
  RtpBuf[12] = 0x00;                               // type specific
  RtpBuf[13] = (m_offset & 0x00FF0000) >> 16;      // 3 byte fragmentation offset for fragmented images
  RtpBuf[14] = (m_offset & 0x0000FF00) >> 8;
  RtpBuf[15] = (m_offset & 0x000000FF);
  RtpBuf[16] = 0x00;                             // type: 0 422 or 1 421
  RtpBuf[17] = 60;                               // quality scale factor
  RtpBuf[16] = format_code;                      // type: 0 422 or 1 421
  if (has_dri_header) {
    RtpBuf[16] |= 0x40;  // DRI flag
  }
  RtpBuf[17] = quality_code;                     // quality scale factor
  RtpBuf[18] = w / 8;                            // width  / 8 -> 48 pixel
  RtpBuf[19] = h / 8;                            // height / 8 -> 32 pixel
  // append the JPEG scan data to the RTP buffer
  memcpy(&RtpBuf[20], Jpeg, JpegLen);

  udp_socket_send_dontwait(udp, RtpBuf, RtpPacketSize);
};
