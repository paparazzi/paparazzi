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
 * @file Sender.h
 *
 * HITL demo version - VectorNav class simulates Vectornav VN-200 INS
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef SRC_VECTORNAV_H_
#define SRC_VECTORNAV_H_

#include <stdio.h>     // Standard input/output definitions
#include <string.h>    // String function definitions

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include "VectorNavData.h"

#include "LogTime.h"

const uint8_t VN_DATA_START = 10;
const short VN_BUFFER_SIZE = 512;
const int VN_UPDATE_RATE = 16;//2;

const uint8_t VN_SYNC = 0xFA;
const uint8_t VN_OUTPUT_GROUP = 0x39;
const uint16_t VN_GROUP_FIELD_1 = 0x01E9;
const uint16_t VN_GROUP_FIELD_2 = 0x061A;
const uint16_t VN_GROUP_FIELD_3 = 0x0140;
const uint16_t VN_GROUP_FIELD_4 = 0x0009;

using namespace std;

class VectorNav
{
private:
  boost::asio::serial_port &port_;
  uint8_t buffer_[VN_BUFFER_SIZE] = {0};
  uint8_t work_; // flag
  bool new_data_;
  boost::mutex datalock_; // mutex
  VectorNavData data_; // data to be sent.
  timeval start_time_;

  /**
   * Sleep in ms
   */
  void msleep(int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
  }

  /**
   * Calculates the 16-bit CRC for the given ASCII or binary message.
   * The CRC is calculated over the packet starting just after the sync byte (not including the sync byte)
   * and ending at the end of payload.
   */
  unsigned short calculateCRC(unsigned char data[], unsigned int length) {
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

public:
  VectorNav(boost::asio::serial_port &port_n, timeval start_t)
    : port_(port_n) {
    start_time_ = start_t;
    new_data_ = false;
    data_ = VectorNavData(); // data to be sent.
    std::cout << "Binding VN \n";
    work_ = 1;
  }

  /*
   * Looks like we are not using the worker function...
   */
  void workerFunc() {
    std::cout << "VN Thread: Starting to work \n";
    while (work_) {
      datalock_.lock();
      send_data();
      //new_data_ = false;
      datalock_.unlock();
      // eventually we wont need to sleep - we will just wait for a signal from another thread
      // this is just an example
      msleep(VN_UPDATE_RATE);
    }
  }

  /**
   * Check whether there are data to send
   * @return True if ready to send a message
   */
  bool hasNewData() {
    datalock_.lock();
    bool nd = new_data_;
    datalock_.unlock();
    return nd;
  }

  /**
   * Set new data to be sent
   * Also triggers the flag for
   * new data available
   * @param tx_data Data to be sent
   */
  void setNewData(VectorNavData tx_data) {
    datalock_.lock();
    data_ = tx_data;
    new_data_ = true;
    datalock_.unlock();
  }

  /**
    * Send data to port
    */
  void send_data() {
    static uint8_t cksum0, cksum1;
    static uint16_t idx;

    buffer_[0] = VN_SYNC;
    buffer_[1] = VN_OUTPUT_GROUP;
    buffer_[2] = (uint8_t)(VN_GROUP_FIELD_1 >> 8);
    buffer_[3] = (uint8_t)(VN_GROUP_FIELD_1);
    buffer_[4] = (uint8_t)(VN_GROUP_FIELD_2 >> 8);
    buffer_[5] = (uint8_t)(VN_GROUP_FIELD_2);
    buffer_[6] = (uint8_t)(VN_GROUP_FIELD_3 >> 8);
    buffer_[7] = (uint8_t)(VN_GROUP_FIELD_3);
    buffer_[8] = (uint8_t)(VN_GROUP_FIELD_4 >> 8);
    buffer_[9] = (uint8_t)(VN_GROUP_FIELD_4);

    idx = VN_DATA_START;

    // Timestamp
    memcpy(&buffer_[idx], &data_.TimeStartup, sizeof(uint64_t));
    idx += sizeof(uint64_t);

    //Attitude, float, [degrees], yaw, pitch, roll, NED frame
    memcpy(&buffer_[idx], &data_.YawPitchRoll, 3 * sizeof(float));
    idx += 3 * sizeof(float);

    // Rates (imu frame), float, [rad/s]
    memcpy(&buffer_[idx], &data_.AngularRate, 3 * sizeof(float));
    idx += 3 * sizeof(float);

    //Pos LLA, double,[beg, deg, m]
    //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
    memcpy(&buffer_[idx], &data_.Position, 3 * sizeof(double));
    idx += 3 * sizeof(double);

    //VelNed, float [m/s]
    //The estimated velocity in the North East Down (NED) frame, given in m/s.
    memcpy(&buffer_[idx], &data_.Velocity, 3 * sizeof(float));
    idx += 3 * sizeof(float);

    // Accel (imu-frame), float, [m/s^-2]
    memcpy(&buffer_[idx], &data_.Accel, 3 * sizeof(float));
    idx += 3 * sizeof(float);

    // tow (in nanoseconds), uint64
    memcpy(&buffer_[idx], &data_.Tow, sizeof(uint64_t));
    idx += sizeof(uint64_t);

    //num sats, uint8
    buffer_[idx] = data_.NumSats;
    idx++;

    //gps fix, uint8
    buffer_[idx] = data_.Fix;
    idx++;

    //posU, float[3]
    memcpy(&buffer_[idx], &data_.PosU, 3 * sizeof(float));
    idx += 3 * sizeof(float);

    //velU, float
    memcpy(&buffer_[idx], &data_.VelU, sizeof(float));
    idx += sizeof(float);

    //linear acceleration imu-body frame, float [m/s^2]
    memcpy(&buffer_[idx], &data_.LinearAccelBody, 3 * sizeof(float));
    idx += 3 * sizeof(float);

    //YprU, float[3]
    memcpy(&buffer_[idx], &data_.YprU, 3 * sizeof(float));
    idx += 3 * sizeof(float);

    //instatus, uint16
    memcpy(&buffer_[idx], &data_.InsStatus, sizeof(uint16_t));
    idx += sizeof(uint16_t);

    //Vel body, float [m/s]
    // The estimated velocity in the body (i.e. imu) frame, given in m/s.
    memcpy(&buffer_[idx], &data_.VelBody, 3 * sizeof(float));
    idx += 3 * sizeof(float);

    // calculate checksum & send
    uint16_t chk = calculateCRC(&buffer_[1], idx - 1);
    buffer_[idx] = (uint8_t)(chk >> 8);
    idx++;
    buffer_[idx] = (uint8_t)(chk & 0xFF);
    idx++;

    //cout << "Going to write " << idx << " bytes\n";
    int len = port_.write_some(boost::asio::buffer(buffer_, idx));
    //cout << "Written " << len << " bytes\n";
  }
};

#endif /* SRC_VECTORNAV_H_ */
