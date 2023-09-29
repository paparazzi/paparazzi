/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 The Paparazzi Team
 * Copyright (C) 2016 Michal Podhradsky <http://github.com/podhrmic>
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

#include "nps_ins.h"
#include <sys/time.h>
#include "nps_fdm.h"
#include <time.h>
#include <stdio.h>
#include "nps_sensors.h"
#include <stdlib.h>     /* srand, rand */

/*
 * Vectornav info
 */
#define VN_DATA_START 10
#define VN_BUFFER_SIZE 512
#define GPS_SEC_IN_DAY 86400

static uint8_t VN_SYNC = 0xFA;
static uint8_t VN_OUTPUT_GROUP = 0x39;
static uint16_t VN_GROUP_FIELD_1 = 0x01E9;
static uint16_t VN_GROUP_FIELD_2 = 0x061A;
static uint16_t VN_GROUP_FIELD_3 = 0x0140;
static uint16_t VN_GROUP_FIELD_4 = 0x0009;

uint8_t vn_buffer[VN_BUFFER_SIZE];

uint8_t *ins_buffer;

struct VectornavData {
  uint64_t TimeStartup;
  float YawPitchRoll[3];
  float AngularRate[3];
  double Position[3];
  float Velocity[3];
  float Accel[3];
  uint64_t Tow;
  uint8_t NumSats;
  uint8_t Fix;
  float PosU[3];
  float VelU;
  float LinearAccelBody[3];
  float YprU[3];
  uint16_t InsStatus;
  float VelBody[3];
};

struct VectornavData vn_data;

void ins_vectornav_init(void);
void ins_vectornav_init(void) {}
void ins_vectornav_event(void);
void ins_vectornav_event(void) {}

/**
 * Calculates the 16-bit CRC for the given ASCII or binary message.
 * The CRC is calculated over the packet starting just after the sync byte (not including the sync byte)
 * and ending at the end of payload.
 */
static short vn_calculate_crc(unsigned char data[], unsigned int length)
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

void nps_ins_init(void)
{
  ins_buffer = &vn_buffer[0];
}


/**
 * @return GPS TOW
 */
static uint64_t vn_get_time_of_week(void)
{
  struct timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;
  struct tm t_res;
  localtime_r(&curTime.tv_sec, &t_res);
  struct tm *tt = &t_res;

  uint64_t tow = (uint64_t)GPS_SEC_IN_DAY * tt->tm_wday + (uint64_t)3600 * tt->tm_hour + (uint64_t)60 * tt->tm_min + tt->tm_sec; // sec
  tow = tow * 1000; // tow to ms
  tow = tow + milli; // tow with added ms
  tow = tow * 1e6; // tow in nanoseconds

  return tow;
}

/**
 * Fetch data from FDM and store them into vectornav packet
 * NOTE: some noise is being added, see Vectornav specifications
 * for details about the precision: http://www.vectornav.com/products/vn-200/specifications
 */
void nps_ins_fetch_data(struct NpsFdm *fdm_ins)
{
  struct NpsFdm fdm_data;
  memcpy(&fdm_data, fdm_ins, sizeof(struct NpsFdm));

  // Timestamp
  vn_data.TimeStartup = (uint64_t)(fdm_data.time * 1000000000.0);

  //Attitude, float, [degrees], yaw, pitch, roll, NED frame
  vn_data.YawPitchRoll[0] = DegOfRad((float)fdm_data.ltp_to_body_eulers.psi); // yaw
  vn_data.YawPitchRoll[1] = DegOfRad((float)fdm_data.ltp_to_body_eulers.theta); // pitch
  vn_data.YawPitchRoll[2] = DegOfRad((float)fdm_data.ltp_to_body_eulers.phi); // roll

  // Rates (imu frame), float, [rad/s]
  vn_data.AngularRate[0] = (float)fdm_data.body_ecef_rotvel.p;
  vn_data.AngularRate[1] = (float)fdm_data.body_ecef_rotvel.q;
  vn_data.AngularRate[2] = (float)fdm_data.body_ecef_rotvel.r;

  //Pos LLA, double,[beg, deg, m]
  //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
  vn_data.Position[0] = DegOfRad(sensors.gps.lla_pos.lat);
  vn_data.Position[1] = DegOfRad(sensors.gps.lla_pos.lon);
  vn_data.Position[2] = sensors.gps.lla_pos.alt;

  //VelNed, float [m/s]
  //The estimated velocity in the North East Down (NED) frame, given in m/s.
  vn_data.Velocity[0] = (float)fdm_data.ltp_ecef_vel.x;
  vn_data.Velocity[1] = (float)fdm_data.ltp_ecef_vel.y;
  vn_data.Velocity[2] = (float)fdm_data.ltp_ecef_vel.z;

  // Accel (imu-frame), float, [m/s^-2]
  vn_data.Accel[0] = (float)fdm_data.body_ecef_accel.x;
  vn_data.Accel[1] = (float)fdm_data.body_ecef_accel.y;
  vn_data.Accel[2] = (float)fdm_data.body_ecef_accel.z;

  // tow (in nanoseconds), uint64
  vn_data.Tow = vn_get_time_of_week();

  //num sats, uint8
  vn_data.NumSats = 8; // random number

  //gps fix, uint8
  // TODO: add warm-up time
  vn_data.Fix = 3; // 3D fix

  //posU, float[3]
  // TODO: use proper sensor simulation
  vn_data.PosU[0] = 2.5+(((float)rand())/RAND_MAX)*0.1;
  vn_data.PosU[1] = 2.5+(((float)rand())/RAND_MAX)*0.1;
  vn_data.PosU[2] = 2.5+(((float)rand())/RAND_MAX)*0.1;

  //velU, float
  // TODO: use proper sensor simulation
  vn_data.VelU = 5.0+(((float)rand())/RAND_MAX)*0.1;

  //linear acceleration imu-body frame, float [m/s^2]
  vn_data.LinearAccelBody[0] = (float)fdm_data.ltp_ecef_vel.x;
  vn_data.LinearAccelBody[1] = (float)fdm_data.ltp_ecef_vel.y;
  vn_data.LinearAccelBody[2] = (float)fdm_data.ltp_ecef_vel.z;

  //YprU, float[3]
  // TODO: use proper sensor simulation
  vn_data.YprU[0] = 2.5+(((float)rand())/RAND_MAX)*0.1;
  vn_data.YprU[1] = 0.5+(((float)rand())/RAND_MAX)*0.1;
  vn_data.YprU[2] = 0.5+(((float)rand())/RAND_MAX)*0.1;

  //instatus, uint16
  vn_data.InsStatus = 0x02;

  //Vel body, float [m/s]
  // The estimated velocity in the body (i.e. imu) frame, given in m/s.
  vn_data.VelBody[0] = (float)fdm_data.body_accel.x;
  vn_data.VelBody[1] = (float)fdm_data.body_accel.y;
  vn_data.VelBody[2] = (float)fdm_data.body_accel.z;
}


uint16_t nps_ins_fill_buffer(void)
{
  static uint16_t idx;

  vn_buffer[0] = VN_SYNC;
  vn_buffer[1] = VN_OUTPUT_GROUP;
  vn_buffer[2] = (uint8_t)(VN_GROUP_FIELD_1 >> 8);
  vn_buffer[3] = (uint8_t)(VN_GROUP_FIELD_1);
  vn_buffer[4] = (uint8_t)(VN_GROUP_FIELD_2 >> 8);
  vn_buffer[5] = (uint8_t)(VN_GROUP_FIELD_2);
  vn_buffer[6] = (uint8_t)(VN_GROUP_FIELD_3 >> 8);
  vn_buffer[7] = (uint8_t)(VN_GROUP_FIELD_3);
  vn_buffer[8] = (uint8_t)(VN_GROUP_FIELD_4 >> 8);
  vn_buffer[9] = (uint8_t)(VN_GROUP_FIELD_4);

  idx = VN_DATA_START;

  // Timestamp
  memcpy(&vn_buffer[idx], &vn_data.TimeStartup, sizeof(uint64_t));
  idx += sizeof(uint64_t);

  //Attitude, float, [degrees], yaw, pitch, roll, NED frame
  memcpy(&vn_buffer[idx], &vn_data.YawPitchRoll, 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // Rates (imu frame), float, [rad/s]
  memcpy(&vn_buffer[idx], &vn_data.AngularRate, 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //Pos LLA, double,[beg, deg, m]
  //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
  memcpy(&vn_buffer[idx], &vn_data.Position, 3 * sizeof(double));
  idx += 3 * sizeof(double);

  //VelNed, float [m/s]
  //The estimated velocity in the North East Down (NED) frame, given in m/s.
  memcpy(&vn_buffer[idx], &vn_data.Velocity, 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // Accel (imu-frame), float, [m/s^-2]
  memcpy(&vn_buffer[idx], &vn_data.Accel, 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // tow (in nanoseconds), uint64
  memcpy(&vn_buffer[idx], &vn_data.Tow, sizeof(uint64_t));
  idx += sizeof(uint64_t);

  //num sats, uint8
  vn_buffer[idx] = vn_data.NumSats;
  idx++;

  //gps fix, uint8
  vn_buffer[idx] = vn_data.Fix;
  idx++;

  //posU, float[3]
  memcpy(&vn_buffer[idx], &vn_data.PosU, 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //velU, float
  memcpy(&vn_buffer[idx], &vn_data.VelU, sizeof(float));
  idx += sizeof(float);

  //linear acceleration imu-body frame, float [m/s^2]
  memcpy(&vn_buffer[idx], &vn_data.LinearAccelBody, 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //YprU, float[3]
  memcpy(&vn_buffer[idx], &vn_data.YprU, 3 * sizeof(float));
  idx += 3 * sizeof(float);

  //instatus, uint16
  memcpy(&vn_buffer[idx], &vn_data.InsStatus, sizeof(uint16_t));
  idx += sizeof(uint16_t);

  //Vel body, float [m/s]
  // The estimated velocity in the body (i.e. imu) frame, given in m/s.
  memcpy(&vn_buffer[idx], &vn_data.VelBody, 3 * sizeof(float));
  idx += 3 * sizeof(float);

  // calculate checksum & send
  uint16_t chk = vn_calculate_crc(&vn_buffer[1], idx - 1);
  vn_buffer[idx] = (uint8_t)(chk >> 8);
  idx++;
  vn_buffer[idx] = (uint8_t)(chk & 0xFF);
  idx++;

  return idx;
}

/**
 *  HITL link to autopilot
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "termios.h"
#include <fcntl.h>
#include <unistd.h>

#include "nps_main.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"
#include "pprzlink/pprz_transport.h"

#define NPS_MAX_MSG_SIZE 512

void *nps_ins_data_loop(void *data __attribute__((unused)));
void *nps_ap_data_loop(void *data __attribute__((unused)));

pthread_t th_ins_data; // sends INS packets to the autopilot
pthread_t th_ap_data; // receives commands from the autopilot

void nps_hitl_impl_init(void)
{
  pthread_create(&th_ins_data, NULL, nps_ins_data_loop, NULL);
  pthread_create(&th_ap_data, NULL, nps_ap_data_loop, NULL);
}

void *nps_ins_data_loop(void *data __attribute__((unused)))
{
  struct timespec requestStart;
  struct timespec requestEnd;
  struct timespec waitFor;
  long int period_ns = (1. / INS_FREQUENCY) * 1000000000L; // thread period in nanoseconds
  long int task_ns = 0; // time it took to finish the task in nanoseconds

  nps_ins_init(); // initialize ins variables and pointers

  // configure port
  int fd = open(STRINGIFY(INS_DEV), O_WRONLY | O_NOCTTY | O_SYNC);//open(INS_DEV, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    printf("INS THREAD: data loop error opening port %i\n", fd);
    return(NULL);
  }

  struct termios new_settings;
  tcgetattr(fd, &new_settings);
  memset(&new_settings, 0, sizeof(new_settings));
  new_settings.c_iflag = 0;
  new_settings.c_cflag = 0;
  new_settings.c_lflag = 0;
  new_settings.c_cc[VMIN] = 0;
  new_settings.c_cc[VTIME] = 0;
  cfsetispeed(&new_settings, (speed_t)INS_BAUD);
  cfsetospeed(&new_settings, (speed_t)INS_BAUD);
  tcsetattr(fd, TCSANOW, &new_settings);

  struct NpsFdm fdm_ins;

  while (TRUE) {
    // lock mutex
    pthread_mutex_lock(&fdm_mutex);

    // start timing
    clock_get_current_time(&requestStart);

    // make a copy of fdm struct to speed things up
    memcpy(&fdm_ins, &fdm, sizeof(fdm));

    // unlock mutex
    pthread_mutex_unlock(&fdm_mutex);

    // fetch data
    nps_ins_fetch_data(&fdm_ins);

    // send ins data here
    static uint16_t idx;
    idx = nps_ins_fill_buffer();

    static int wlen;
    wlen = write(fd, ins_buffer, idx);
    if (wlen != idx) {
      printf("INS THREAD: Warning - sent only %u bytes to the autopilot, instead of expected %u\n", wlen, idx);
    }

    clock_get_current_time(&requestEnd);

    // Calculate time it took
    task_ns = (requestEnd.tv_sec - requestStart.tv_sec) * 1000000000L + (requestEnd.tv_nsec - requestStart.tv_nsec);

    // task took less than one period, sleep for the rest of time
    if (task_ns < period_ns) {
      waitFor.tv_sec = 0;
      waitFor.tv_nsec = period_ns - task_ns;
      nanosleep(&waitFor, NULL);
    } else {
      // task took longer than the period
#ifdef PRINT_TIME
      printf("INS THREAD: task took longer than one period, exactly %f [ms], but the period is %f [ms]\n",
             (double)task_ns / 1E6, (double)period_ns / 1E6);
#endif
    }
  }
  return(NULL);
}



void *nps_ap_data_loop(void *data __attribute__((unused)))
{
  // configure port
  int fd = open(STRINGIFY(AP_DEV), O_RDONLY | O_NOCTTY);
  if (fd < 0) {
    printf("AP data loop error opening port %i\n", fd);
    return(NULL);
  }

  struct termios new_settings;
  tcgetattr(fd, &new_settings);
  memset(&new_settings, 0, sizeof(new_settings));
  new_settings.c_iflag = 0;
  new_settings.c_cflag = 0;
  new_settings.c_lflag = 0;
  new_settings.c_cc[VMIN] = 1;
  new_settings.c_cc[VTIME] = 5;
  cfsetispeed(&new_settings, (speed_t)AP_BAUD);
  cfsetospeed(&new_settings, (speed_t)AP_BAUD);
  tcsetattr(fd, TCSANOW, &new_settings);

  int rdlen;
  uint8_t buf[NPS_MAX_MSG_SIZE];
  uint8_t cmd_len;
  pprz_t  cmd_buf[NPS_COMMANDS_NB];

  struct pprz_transport pprz_tp_logger;

  pprz_transport_init(&pprz_tp_logger);
  uint32_t rx_msgs = 0;
  uint32_t rx_commands = 0;
  uint32_t rx_motor_mixing = 0;
  uint8_t show_stats = 1;

  while (TRUE) {
    if ((rx_msgs % 100 == 0) && show_stats && (rx_msgs > 0) ) {
      printf("AP data loop received %i messages.\n", rx_msgs);
      printf("From those, %i were COMMANDS messages, and %i were MOTOR_MIXING messages.\n",
          rx_commands, rx_motor_mixing);
      show_stats = 0;
    }

    // receive messages from the autopilot
    // Note: read function might wait until the buffer is full, in which case
    // it could contain several messages. That might lead to delay in processing,
    // for example if we send COMMANDS at 100Hz, and each read() will hold 10 COMMANDS
    // it means a delay of 100ms before the message is processed.
    rdlen = read(fd, buf, sizeof(buf) - 1);

    for (int i = 0; i < rdlen; i++) {
      // parse data
      parse_pprz(&pprz_tp_logger, buf[i]);

      // if msg_available then read
      if (pprz_tp_logger.trans_rx.msg_received) {
        rx_msgs++;
        for (int k = 0; k < pprz_tp_logger.trans_rx.payload_len; k++) {
          buf[k] = pprz_tp_logger.trans_rx.payload[k];
        }
        //Parse message
        uint8_t sender_id = SenderIdOfPprzMsg(buf);
        uint8_t msg_id = IdOfPprzMsg(buf);

        // parse telemetry messages coming from other AC
        if (sender_id != AC_ID) {
          switch (msg_id) {
            default: {
              break;
            }
          }
        } else {
          /* parse telemetry messages coming from the correct AC_ID */
          switch (msg_id) {
            case DL_COMMANDS:
              // parse commands message
              rx_commands++;
              cmd_len = DL_COMMANDS_values_length(buf);
              memcpy(&cmd_buf, DL_COMMANDS_values(buf), cmd_len * sizeof(int16_t));
              pthread_mutex_lock(&fdm_mutex);
              // update commands
              for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
                nps_autopilot.commands[i] = (double)cmd_buf[i] / MAX_PPRZ;
              }
              // hack: invert pitch to fit most JSBSim models
              nps_autopilot.commands[COMMAND_PITCH] = -(double)cmd_buf[COMMAND_PITCH] / MAX_PPRZ;
              pthread_mutex_unlock(&fdm_mutex);
              break;
            case DL_MOTOR_MIXING:
              // parse actuarors message
              rx_motor_mixing++;
              cmd_len = DL_MOTOR_MIXING_values_length(buf);
              // check for out-of-bounds access
              if (cmd_len > NPS_COMMANDS_NB) {
                cmd_len = NPS_COMMANDS_NB;
              }
              memcpy(&cmd_buf, DL_MOTOR_MIXING_values(buf), cmd_len * sizeof(int16_t));
              pthread_mutex_lock(&fdm_mutex);
              // update commands
              for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
                nps_autopilot.commands[i] = (double)cmd_buf[i] / MAX_PPRZ;
              }
              pthread_mutex_unlock(&fdm_mutex);
              break;
            default:
              break;
          }
        }
        pprz_tp_logger.trans_rx.msg_received = false;
      }
    }
  }
  return(NULL);
}

