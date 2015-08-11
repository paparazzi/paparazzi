/*
 * Copyright (C) 2014 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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
 * @file ins_vectornav.h
 *
 * Vectornav VN-200 INS subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef INS_VECTORNAV_H
#define INS_VECTORNAV_H

#include "subsystems/ins.h"

#ifndef DefaultInsImpl
#define DefaultInsImpl ins_vectornav
#endif

#ifdef InsEvent
#undef InsEvent
#endif

#define InsEvent ins_vectornav_event

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"

#include "led.h"

#define VN_SYNC 0xFA
#define VN_OUTPUT_GROUP 0x39
#define VN_GROUP_BYTES 8

#define VN_BUFFER_SIZE 512
#define VN_HEADER_SIZE 9
#define VN_PAYLOAD_SIZE 144

#define DEG_TO_RAD 0.017453292519943//0.0175


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
};

enum VNStatus {
  VNNotTracking,
  VNOutOfSpecs,
  VNOK
};

/** Ins implementation state (fixed point) */
struct InsInt {
  struct LtpDef_i  ltp_def; // initial position
  bool_t           ltp_initialized; // status indicator

  /* output LTP NED for telemetry messages*/
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  struct LlaCoor_f lla_pos;

  struct NedCoor_f ltp_accel_f;

  /* baro [height above ground]*/
  float baro_z;  ///< z-position calculated from baro in meters (z-down)
  float qfe;

  // Packet data
  struct VNPacket vn_packet;///< Packet struct
  enum VNStatus vn_status;  ///< VN status
  float vn_freq;            ///< data frequency
  uint16_t vn_chksm;         ///< aux variable for checksum
  uint32_t vn_time;          ///< VN time stamp
  uint32_t vn_ltime;         ///< aux time stamp

  // Auxilliary data fields
  float timestamp; // System time [s]
  struct FloatEulers attitude; // Attitude, float, [degrees], yaw, pitch, roll
  // rates -> imu
  double pos_lla[3]; // Lla [deg, deg, m above elipsoid]
  struct NedCoor_f vel_ned; // The estimated velocity in the North East Down (NED) frame, given in m/s.
  // accel -> imu
  // num sats -> GPS
  // GPS fix -> GPS
  float posU[3]; // The current GPS position uncertainty in the North East Down (NED) coordinate frame, given in meters.
  float velU; // NED velocity uncertainty [m/s]
  struct FloatVect3 lin_accel; // Linear acceleration in imu frame [m/s^2]
  struct FloatEulers YprU; // Attitude uncertainty, 1sigma, float, [degrees], yaw, pitch, roll
  uint16_t ins_status; // see page 122 of VN-200 datasheet
  uint8_t mode; // 0-not tracking, 1 - poor performance, 2- OK
  uint8_t err; // see page 122 of VN-200 datasheet
  struct FloatVect3 vel_body; //The estimated velocity in the imu frame, given in m/s.
};



/** global INS state */
extern struct InsInt ins_impl;

extern void ins_vectornav_init(void);
extern void ins_vectornav_register(void);

void ins_vectornav_propagate(void);
void ins_vectornav_read_message(void);
void ins_vectornav_parse(uint8_t c);
void ins_init_origin_from_flightplan(void);
void ins_ned_to_state(void);

/**
 * Calculates the 16-bit CRC for the given ASCII or binary message.
 * The CRC is calculated over the packet starting just after the sync byte (not including the sync byte)
 * and ending at the end of payload.
 */
static inline unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for (i=0; i<length; i++){
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
static inline bool verify_chk(unsigned char data[], unsigned int length, uint16_t* calc_chk, uint16_t* rec_chk) {
  unsigned short calc_crc = calculateCRC(data, length);
  unsigned short rec_crc = (unsigned short) (data[length] << 8 | data[length+1]);
  *calc_chk = (uint16_t) calc_crc;
  *rec_chk = (uint16_t) rec_crc;
  if (calc_crc == rec_crc) {
    return 1;
  }
  else {
    return 0;
  }
}


static inline void ReadVnBuffer(void) {
  while (uart_char_available(&VN_PORT) && !ins_impl.vn_packet.msg_available)
    ins_vectornav_parse(uart_getch(&VN_PORT));
}

static inline void ins_vectornav_event(void) {
  if (uart_char_available(&VN_PORT)) {
    ReadVnBuffer();
  }
  if (ins_impl.vn_packet.msg_available) {
    ins_vectornav_read_message();
    ins_impl.vn_packet.msg_available = FALSE;
  }
}

#endif /* INS_VECTORNAV_H */
