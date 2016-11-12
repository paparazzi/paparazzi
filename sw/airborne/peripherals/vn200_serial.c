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


void vn200_check_status(struct VNData *vn_data);
void vn200_yaw_pitch_roll_to_attitude(struct FloatEulers *vn_attitude);


/**
 * Check INS status
 */
void vn200_check_status(struct VNData *vn_data)
{
  vn_data->mode = (uint8_t)(vn_data->ins_status & 0x03);
  vn_data->err = (uint8_t)((vn_data->ins_status >> 3) & 0x0F);
}


/**
 * Convert yaw, pitch, and roll data from VectorNav
 * to correct attitude
 * yaw(0), pitch(1), roll(2) -> phi, theta, psi
 * [deg] -> rad
 */
void vn200_yaw_pitch_roll_to_attitude(struct FloatEulers *vn_attitude)
{
  static struct FloatEulers att_rad;
  att_rad.phi = RadOfDeg(vn_attitude->psi);
  att_rad.theta = RadOfDeg(vn_attitude->theta);
  att_rad.psi = RadOfDeg(vn_attitude->phi);

  vn_attitude->phi = att_rad.phi;
  vn_attitude->theta = att_rad.theta;
  vn_attitude->psi = att_rad.psi;
}


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
          vnp->msg_available = true;
          vnp->counter++;
        } else {
          vnp->msg_available = false;
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


/**
 * Read received message and populate data struct with new measurements
 */
void vn200_read_message(struct VNPacket *vn_packet, struct VNData *vn_data)
{
  uint16_t idx = VN_HEADER_SIZE;

  // Timestamp [nanoseconds] since startup
  memcpy(&vn_data->nanostamp, &vn_packet->msg_buf[idx], sizeof(uint64_t));
  idx += sizeof(uint64_t);

  // Timestamp [s]
  vn_data->timestamp = ((float)vn_data->nanostamp / 1000000000); // [nanoseconds to seconds]

  //Attitude, float, [degrees], yaw, pitch, roll, NED frame
  memcpy(&vn_data->attitude, &vn_packet->msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // correctly rearrange YawPitchRoll to attitude (plus scale to radians)
  vn200_yaw_pitch_roll_to_attitude(&vn_data->attitude); // convert to correct units and axis [rad]

  // Rates (imu frame), float, [rad/s]
  memcpy(&vn_data->gyro, &vn_packet->msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //Pos LLA, double,[deg, deg, m]
  //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
  memcpy(&vn_data->pos_lla, &vn_packet->msg_buf[idx], 3 * sizeof(double));
  idx += 3 * sizeof(double);

  //VelNed, float [m/s]
  //The estimated velocity in the North East Down (NED) frame, given in m/s.
  memcpy(&vn_data->vel_ned, &vn_packet->msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // Accel (imu-frame), float, [m/s^-2]
  memcpy(&vn_data->accel, &vn_packet->msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // tow (in nanoseconds), uint64
  memcpy(&vn_data->tow, &vn_packet->msg_buf[idx], sizeof(uint64_t));
  idx += sizeof(uint64_t);
  vn_data->tow = vn_data->tow / 1000000; // nanoseconds to miliseconds

  //num sats, uint8
  vn_data->num_sv = vn_packet->msg_buf[idx];
  idx++;

  //gps fix, uint8
  vn_data->gps_fix = vn_packet->msg_buf[idx];
  idx++;

  //posU, float[3]
  memcpy(&vn_data->pos_u, &vn_packet->msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //velU, float
  memcpy(&vn_data->vel_u, &vn_packet->msg_buf[idx], sizeof(float));
  idx += sizeof(float);

  //linear acceleration imu-body frame, float [m/s^2]
  //The estimated linear acceleration (without gravity) reported in m/s^2, and given in the body frame. The
  //acceleration measurement has been bias compensated by the onboard INS filter, and the gravity
  //component has been removed using the current gravity reference vector model. This measurement is
  //attitude dependent, since the attitude solution is required to map the gravity reference vector (known
  //in the inertial NED frame), into the body frame so that it can be removed from the measurement. If the
  //device is stationary and the onboard INS filter is tracking, the measurement nominally will read 0 in all
  //three axes.
  memcpy(&vn_data->lin_accel, &vn_packet->msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //YprU, float[3]
  memcpy(&vn_data->ypr_u, &vn_packet->msg_buf[idx], 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //instatus, uint16
  memcpy(&vn_data->ins_status, &vn_packet->msg_buf[idx], sizeof(uint16_t));
  idx += sizeof(uint16_t);

  vn200_check_status(vn_data); // Check INS status

  //Vel body, float [m/s]
  // The estimated velocity in the body (i.e. imu) frame, given in m/s.
  memcpy(&vn_data->vel_body, &vn_packet->msg_buf[idx], sizeof(float));
  idx += sizeof(float);
}
