/*
 * Copyright (C) 2013 Michal Podhradsky
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
* @file imu_um6.c
*
* Driver for CH Robotics UM6 IMU/AHRS subsystem
*
* Takes care of configuration of the IMU, communication and parsing
* the received packets. See UM6 datasheet for configuration options.
* Should be used with ahrs_extern_euler AHRS subsystem.
*
* @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
*/
#include "subsystems/imu/imu_um6.h"
#include "subsystems/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"

struct UM6Packet UM6_packet;
uint8_t buf_out[IMU_UM6_BUFFER_LENGTH];
uint16_t data_chk;

uint8_t PacketLength;
uint8_t PacketType;
uint8_t PacketAddr;
enum UM6Status UM6_status;

uint16_t chk_calc;
uint16_t chk_rec;

struct FloatVect3 UM6_accel;
struct FloatRates UM6_rate;
struct FloatVect3 UM6_mag;
struct FloatEulers UM6_eulers;
struct FloatQuat UM6_quat;

inline void UM6_imu_align(void);
inline void UM6_send_packet(uint8_t *packet_buffer, uint8_t packet_length);
inline uint16_t UM6_calculate_checksum(uint8_t packet_buffer[], uint8_t packet_length);
inline bool UM6_verify_chk(uint8_t packet_buffer[], uint8_t packet_length);

inline bool UM6_verify_chk(uint8_t packet_buffer[], uint8_t packet_length)
{
  chk_rec = (packet_buffer[packet_length - 2] << 8) | packet_buffer[packet_length - 1];
  chk_calc = UM6_calculate_checksum(packet_buffer, packet_length - 2);
  return (chk_calc == chk_rec);
}

inline uint16_t UM6_calculate_checksum(uint8_t packet_buffer[], uint8_t packet_length)
{
  uint16_t chk = 0;
  for (int i = 0; i < packet_length; i++) {
    chk += packet_buffer[i];
  }
  return chk;
}

inline void UM6_send_packet(uint8_t *packet_buffer, uint8_t packet_length)
{
  for (int i = 0; i < packet_length; i++) {
    uart_put_byte(&(UM6_LINK), 0, packet_buffer[i]);
  }
}

void UM6_imu_align(void)
{
  UM6_status = UM6Uninit;

  // Acceleration vector realign
  buf_out[0] = 's';
  buf_out[1] = 'n';
  buf_out[2] = 'p';
  buf_out[3] = 0;
  buf_out[4] = IMU_UM6_SET_MAG_REF;
  data_chk = UM6_calculate_checksum(buf_out, 5);
  buf_out[5] = (uint8_t)(data_chk >> 8);
  buf_out[6] = (uint8_t)data_chk;
  UM6_send_packet(buf_out, 7);

  // Magnetic realign
  buf_out[0] = 's';
  buf_out[1] = 'n';
  buf_out[2] = 'p';
  buf_out[3] = 0;
  buf_out[4] = IMU_UM6_SET_ACCEL_REF;
  data_chk = UM6_calculate_checksum(buf_out, 5);
  buf_out[5] = (uint8_t)(data_chk >> 8);
  buf_out[6] = (uint8_t)data_chk;
  UM6_send_packet(buf_out, 7);

  // Zero gyros 0xAC, takes around 3s
  buf_out[0] = 's';
  buf_out[1] = 'n';
  buf_out[2] = 'p';
  buf_out[3] = 0;
  buf_out[4] = IMU_UM6_ZERO_GYROS_CMD;
  data_chk = UM6_calculate_checksum(buf_out, 5);
  buf_out[5] = (uint8_t)(data_chk >> 8);
  buf_out[6] = (uint8_t)data_chk;
  UM6_send_packet(buf_out, 7);

  // Reset EKF 0xAD
  buf_out[0] = 's';
  buf_out[1] = 'n';
  buf_out[2] = 'p';
  buf_out[3] = 0;
  buf_out[4] = IMU_UM6_RESET_EKF_CMD;
  data_chk = UM6_calculate_checksum(buf_out, 5);
  buf_out[5] = (uint8_t)(data_chk >> 8);
  buf_out[6] = (uint8_t)data_chk;
  UM6_send_packet(buf_out, 7);

  UM6_status = UM6Running;
}

void imu_um6_init(void)
{
  // Initialize variables
  UM6_status = UM6Uninit;

  // Initialize packet
  UM6_packet.status = UM6PacketWaiting;
  UM6_packet.msg_idx = 0;
  UM6_packet.msg_available = false;
  UM6_packet.chksm_error = 0;
  UM6_packet.hdr_error = 0;

  // Communication register 0x00
  buf_out[0] = 's';
  buf_out[1] = 'n';
  buf_out[2] = 'p';
  buf_out[3] = 0x80;        // has data, zeros
  buf_out[4] = IMU_UM6_COMMUNICATION_REG;        // communication register address
  buf_out[5] = 0b01000110;  // B3 - broadcast enabled, processed data enabled (acc, gyro, mag)
  buf_out[6] = 0b10000000;  // B2 - no euler angles, quaternions enabled
  buf_out[7] = 0b00101101;  // B1 - baud 115200
  buf_out[8] = 100;        // B0 - broadcast rate, 1=20Hz, 100=130Hz, 255=300Hz
  data_chk = UM6_calculate_checksum(buf_out, 9);
  buf_out[9] = (uint8_t)(data_chk >> 8);
  buf_out[10] = (uint8_t)data_chk;
  UM6_send_packet(buf_out, 11);

  // Config register 0x01
  buf_out[0] = 's';
  buf_out[1] = 'n';
  buf_out[2] = 'p';
  buf_out[3] = 0x80;        // has data, zeros
  buf_out[4] = IMU_UM6_MISC_CONFIG_REG;  // config register address
  buf_out[5] = 0b11110000;  // B3 - mag, accel updates enabled, use quaternions
  buf_out[6] = 0;  // B2 - RES
  buf_out[7] = 0;  // B1 - RES
  buf_out[8] = 0;        // B0 - RES
  data_chk = UM6_calculate_checksum(buf_out, 9);
  buf_out[9] = (uint8_t)(data_chk >> 8);
  buf_out[10] = (uint8_t)data_chk;
  UM6_send_packet(buf_out, 11);

  /*
  // Get firmware
  buf_out[0] = 's';
  buf_out[1] = 'n';
  buf_out[2] = 'p';
  buf_out[3] = 0;
  buf_out[4] = IMU_UM6_GET_FIRMWARE_CMD;
  data_chk = UM6_calculate_checksum(buf_out, 5);
  buf_out[5] = (uint8_t)(data_chk >> 8);
  buf_out[6] = (uint8_t)data_chk;
  UM6_send_packet(buf_out, 7);
  */

  UM6_imu_align();
}



void imu_um6_periodic(void)
{
  /* We would request for data here - optional
  //GET_DATA command 0xAE
  buf_out[0] = 's';
  buf_out[1] = 'n';
  buf_out[2] = 'p';
  buf_out[3] = 0;           // zeros
  buf_out[4] = 0xAE;        // GET_DATA command
  data_chk = UM6_calculate_checksum(buf_out, 5);
  buf_out[5] = (uint8_t)(data_chk >> 8);
  buf_out[6] = (uint8_t)data_chk;
  UM6_send_packet(buf_out, 7);
  */
}

void UM6_packet_read_message(void)
{
  if ((UM6_status == UM6Running) && PacketLength > 11) {
    switch (PacketAddr) {
      case IMU_UM6_GYRO_PROC:
        UM6_rate.p =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 0] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 1]))) * 0.0610352;
        UM6_rate.q =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 2] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 3]))) * 0.0610352;
        UM6_rate.r =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 4] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 5]))) * 0.0610352;
        RATES_SMUL(UM6_rate, UM6_rate, 0.0174532925); //Convert deg/sec to rad/sec
        RATES_BFP_OF_REAL(imu.gyro, UM6_rate);
        break;
      case IMU_UM6_ACCEL_PROC:
        UM6_accel.x =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 0] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 1]))) * 0.000183105;
        UM6_accel.y =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 2] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 3]))) * 0.000183105;
        UM6_accel.z =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 4] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 5]))) * 0.000183105;
        VECT3_SMUL(UM6_accel, UM6_accel, 9.80665); //Convert g into m/s2
        ACCELS_BFP_OF_REAL(imu.accel, UM6_accel); // float to int
        break;
      case IMU_UM6_MAG_PROC:
        UM6_mag.x =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 0] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 1]))) * 0.000305176;
        UM6_mag.y =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 2] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 3]))) * 0.000305176;
        UM6_mag.z =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 4] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 5]))) * 0.000305176;
        // Assume the same units for magnetic field
        // Magnitude at the Earth's surface ranges from 25 to 65 microteslas (0.25 to 0.65 gauss).
        MAGS_BFP_OF_REAL(imu.mag, UM6_mag);
        break;
      case IMU_UM6_EULER:
        UM6_eulers.phi =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 0] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 1]))) * 0.0109863;
        UM6_eulers.theta =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 2] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 3]))) * 0.0109863;
        UM6_eulers.psi =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 4] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 5]))) * 0.0109863;
        EULERS_SMUL(UM6_eulers, UM6_eulers, 0.0174532925); //Convert deg to rad
        EULERS_BFP_OF_REAL(ahrs_impl.ltp_to_imu_euler, UM6_eulers);
        break;
      case IMU_UM6_QUAT:
        UM6_quat.qi =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 0] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 1]))) * 0.0000335693;
        UM6_quat.qx =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 2] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 3]))) * 0.0000335693;
        UM6_quat.qy =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 4] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 5]))) * 0.0000335693;
        UM6_quat.qz =
          ((float)((int16_t)
                   ((UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 6] << 8) | UM6_packet.msg_buf[IMU_UM6_DATA_OFFSET + 7]))) * 0.0000335693;
        QUAT_BFP_OF_REAL(ahrs_impl.ltp_to_imu_quat, UM6_quat);
        break;
      default:
        break;
    }
  }
}

/* UM6 Packet Collection */
void UM6_packet_parse(uint8_t c)
{
  switch (UM6_packet.status) {
    case UM6PacketWaiting:
      UM6_packet.msg_idx = 0;
      if (c == 's') {
        UM6_packet.status = UM6PacketReadingS;
        UM6_packet.msg_buf[UM6_packet.msg_idx] = c;
        UM6_packet.msg_idx++;
      } else {
        UM6_packet.hdr_error++;
        UM6_packet.status = UM6PacketWaiting;
      }
      break;
    case UM6PacketReadingS:
      if (c == 'n') {
        UM6_packet.status = UM6PacketReadingN;
        UM6_packet.msg_buf[UM6_packet.msg_idx] = c;
        UM6_packet.msg_idx++;
      } else {
        UM6_packet.hdr_error++;
        UM6_packet.status = UM6PacketWaiting;
      }
      break;
    case UM6PacketReadingN:
      if (c == 'p') {
        UM6_packet.status = UM6PacketReadingPT;
        UM6_packet.msg_buf[UM6_packet.msg_idx] = c;
        UM6_packet.msg_idx++;
      } else {
        UM6_packet.hdr_error++;
        UM6_packet.status = UM6PacketWaiting;
      }
      break;
    case UM6PacketReadingPT:
      PacketType = c;
      UM6_packet.msg_buf[UM6_packet.msg_idx] =  c;
      UM6_packet.msg_idx++;
      UM6_packet.status = UM6PacketReadingAddr;
      if ((PacketType & 0xC0) == 0xC0) {
        PacketLength = 4 * ((PacketType >> 2) & 0xF) + 7; // Batch, has 4*BatchLength bytes of data
      } else if ((PacketType & 0xC0) == 0x80) {
        PacketLength = 11; // Not batch, has 4 bytes of data
      } else if ((PacketType & 0xC0) == 0x00) {
        PacketLength = 7; // Not batch, no data
      }
      break;
    case UM6PacketReadingAddr:
      PacketAddr = c;
      UM6_packet.msg_buf[UM6_packet.msg_idx] =  c;
      UM6_packet.msg_idx++;
      UM6_packet.status = UM6PacketReadingData;
      break;
    case UM6PacketReadingData:
      UM6_packet.msg_buf[UM6_packet.msg_idx] =  c;
      UM6_packet.msg_idx++;
      if (UM6_packet.msg_idx == PacketLength) {
        if (UM6_verify_chk(UM6_packet.msg_buf, PacketLength)) {
          UM6_packet.msg_available = true;
        } else {
          UM6_packet.msg_available = false;
          UM6_packet.chksm_error++;
        }
        UM6_packet.status = UM6PacketWaiting;
      }
      break;
    default:
      UM6_packet.status = UM6PacketWaiting;
      UM6_packet.msg_idx = 0;
      break;
  }
}

/* no scaling */
void imu_scale_gyro(struct Imu *_imu __attribute__((unused))) {}
void imu_scale_accel(struct Imu *_imu __attribute__((unused))) {}
void imu_scale_mag(struct Imu *_imu __attribute__((unused))) {}


void imu_um6_publish(void)
{
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgIMU_GYRO_INT32(IMU_UM6_ID, now_ts, &imu.gyro);
  AbiSendMsgIMU_ACCEL_INT32(IMU_UM6_ID, now_ts, &imu.accel);
  AbiSendMsgIMU_MAG_INT32(IMU_UM6_ID, now_ts, &imu.mag);
}
