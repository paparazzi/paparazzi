/*
 * Copyright (C) 2013 Gautier Hattenberger
 *
 * Mixed with modified version of MNAV
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
 * @file nps_fdm_crrcsim.c
 * Flight Dynamics Model (FDM) for NPS using CRRCSIM.
 *
 */

#include "nps_fdm.h"

#include <stdlib.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#ifndef NPS_CRRCSIM_HOST_IP
#define NPS_CRRCSIM_HOST_IP "127.0.0.1"
#endif

#ifndef NPS_CRRCSIM_HOST_PORT
#define NPS_CRRCSIM_HOST_PORT 9002
#endif

#ifndef NPS_CRRCSIM_ROLL_NEUTRAL
#define NPS_CRRCSIM_ROLL_NEUTRAL 0.
#endif

#ifndef NPS_CRRCSIM_PITCH_NEUTRAL
#define NPS_CRRCSIM_PITCH_NEUTRAL 0.
#endif

/* blocking */
#define UDP_BLOCKING 0
#define UDP_NONBLOCKING 1

// type
#define word unsigned short
#define byte unsigned char

// uNAV packet length definition
#define IMU_PACKET_LENGTH 51
#define GPS_PACKET_LENGTH	86
#define AHRS_PACKET_LENGTH 93
#define FULL_PACKET_SIZE 93


// NpsFdm structure
struct NpsFdm fdm;

#define INPUT_BUFFER_SIZE (3*FULL_PACKET_SIZE)
// Input buffer
struct inputbuf {
   byte buf[INPUT_BUFFER_SIZE];
   byte start;
   int  length;
};

// Socket structure
struct _crrcsim {
  int socket;
  struct sockaddr_in addr;
  struct inputbuf buf;
  byte data_buffer[FULL_PACKET_SIZE];
};

static struct _crrcsim crrcsim;

// Reference point
static struct LtpDef_d ltpdef;

// static functions declaration
static void open_udp(char* host, int port, int blocking);
static void inputbuf_init(struct inputbuf* c);
static void read_into_buffer(struct _crrcsim* io);
static void init_ltp(void);
static int get_msg(struct _crrcsim* io, byte* data_buffer);
static void decode_imupacket(struct NpsFdm * fdm, byte* buffer);
static void decode_gpspacket(struct NpsFdm * fdm, byte* buffer);
static void decode_ahrspacket(struct NpsFdm * fdm, byte* buffer);
static void send_servo_cmd(struct _crrcsim* io, double* commands);

// NPS FDM interface
void nps_fdm_init(double dt) {
  fdm.init_dt = dt;
  fdm.curr_dt = dt;
  fdm.nan_count = 0;

  init_ltp();

  // init circfuf
  inputbuf_init(&crrcsim.buf);

  printf("Starting to connect to CRRCsim server.\n");
  open_udp((char*)(NPS_CRRCSIM_HOST_IP), NPS_CRRCSIM_HOST_PORT, UDP_NONBLOCKING);

  if (crrcsim.socket < 0) {
    printf("Connection to CRRCsim failed\n");
    exit(0);
  }
  else {
    printf("Connection to CRRCsim succed\n");
  }

  // Send something to let crrcsim that we are here
  double zero[] = { 0., 0., 0. };
  send_servo_cmd(&crrcsim, zero);
}

void nps_fdm_run_step(bool_t launch __attribute__((unused)), double* commands, int commands_nb) {
  // read state
  if (get_msg(&crrcsim, crrcsim.data_buffer) <= 0) {
    return; // nothing on the socket
  }

  // send commands
  send_servo_cmd(&crrcsim, commands);

  //printf("new data %c %c\n", crrcsim.data_buffer[2], crrcsim.data_buffer[33]);
  switch (crrcsim.data_buffer[2])
  {
    case 'S': /* IMU packet without GPS */
      decode_imupacket(&fdm, crrcsim.data_buffer);
      break;

    case 'N': /* IMU packet with GPS */

      decode_imupacket(&fdm, crrcsim.data_buffer);

      /******************************************
       *check GPS data packet
       ******************************************/
      //if(data_buffer[31]==(byte)0x55 && data_buffer[32]==(byte)0x55 && data_buffer[33]=='G')
      if(crrcsim.data_buffer[33]=='G') {
        decode_gpspacket(&fdm, &crrcsim.data_buffer[31]);
      }
      break;

    case 'I': /* IMU packet with GPS and AHRS */

      decode_imupacket(&fdm, crrcsim.data_buffer);

      /******************************************
       *check GPS data packet
       ******************************************/
      if(crrcsim.data_buffer[33]=='G') {
        decode_gpspacket(&fdm, &crrcsim.data_buffer[31]);
      }
      if(crrcsim.data_buffer[66]=='A') {
        decode_ahrspacket(&fdm, &crrcsim.data_buffer[66]);
      }
      break;

    default :
      printf("invalid data packet...!\n");
  } /* end case  */

}

void nps_fdm_set_wind(double speed __attribute__((unused)), double dir __attribute__((unused)), int turbulence_severity __attribute__((unused))) {
}

/***************************************************************************
 ** Open and configure UDP connection
 ****************************************************************************/

static void open_udp(char* host, int port, int blocking)
{
  int    flags;

  bzero((char *) &crrcsim.addr, sizeof(crrcsim.addr));
  crrcsim.addr.sin_family      = AF_INET;
  crrcsim.addr.sin_addr.s_addr = inet_addr(host);
  crrcsim.addr.sin_port        = htons(port);
  crrcsim.socket = socket(AF_INET, SOCK_DGRAM, 0);

  //make a nonblocking connection
  flags = fcntl(crrcsim.socket, F_GETFL, 0);
  fcntl(crrcsim.socket, F_SETFL, flags | O_NONBLOCK);

  if (connect(crrcsim.socket,(struct sockaddr*)&crrcsim.addr,sizeof(crrcsim.addr)) < 0) {
    close(crrcsim.socket);
    crrcsim.socket = -1;
  }

  if(crrcsim.socket != -1 && blocking == UDP_BLOCKING)
  {
    //restore
    fcntl(crrcsim.socket, F_SETFL, flags);
  }
}

static void inputbuf_init(struct inputbuf* c)
{
  c->start = 0;
  c->length = 0;
}

static void read_into_buffer(struct _crrcsim* io)
{
  struct inputbuf *c = &io->buf;
  int res;

  if (io->socket >= 0) {
    // get latest data from the buffer (crapy but working)
    while ((res = recv(io->socket, c->buf, INPUT_BUFFER_SIZE, 0)) > 0) {
      c->start = 0;
      c->length = res;
    }
  }
}

static void init_ltp(void) {

  struct LlaCoor_d llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = RadOfDeg((double)NAV_LAT0/1e7);
  llh_nav0.lon = RadOfDeg((double)NAV_LON0/1e7);
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = (NAV_ALT0 + NAV_MSL0)/1000.;

  struct EcefCoor_d ecef_nav0;
  ecef_of_lla_d(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_d(&ltpdef, &ecef_nav0);

  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 0.; // accel data are already with the correct format

#ifdef AHRS_H_X
#pragma message "Using magnetic field as defined in airframe file."
  fdm.ltp_h.x = AHRS_H_X;
  fdm.ltp_h.y = AHRS_H_Y;
  fdm.ltp_h.z = AHRS_H_Z;
#else
  fdm.ltp_h.x = 0.4912;
  fdm.ltp_h.y = 0.1225;
  fdm.ltp_h.z = 0.8624;
#endif

}

static int get_msg(struct _crrcsim* io, byte* data_buffer)
{
  struct inputbuf *c = &io->buf;
  int count = 0;
  int packet_len;
  int i;

  read_into_buffer(io);

  while (1) {
    /*********************************************************************
     * Find start of packet: the header (2 bytes) starts with 0x5555
     *********************************************************************/
    while(c->length >= 4 && (c->buf[c->start] != (byte)0x55 || c->buf[(byte)(c->start + 1)] != (byte)0x55)) {
      c->start++;
      c->length--;
    }
    if(c->length < 4)
      return count;

    /*********************************************************************
     * Read packet contents
     *********************************************************************/
    packet_len = 0;
    switch (c->buf[(byte)(c->start + 2)])
    {
      case 'S':
        packet_len = IMU_PACKET_LENGTH;
        break;

      case 'N':
        packet_len = GPS_PACKET_LENGTH;
        break;

      case 'I':
        packet_len = AHRS_PACKET_LENGTH;
        break;

      default:
        break;
    }

    if(packet_len > 0 && c->length < packet_len)
      return count; // not enough data
    if(packet_len > 0) {
      byte ib;
      word rcvchecksum = 0;
      word sum = 0;

      for(i = 2, ib = c->start + (byte)2; i < packet_len - 2; i++, ib++)
        sum += c->buf[ib];
      rcvchecksum = c->buf[ib++] << 8;
      rcvchecksum = rcvchecksum | c->buf[ib++];

      if(rcvchecksum != sum) {
        packet_len = 0;
        printf("checksum error\n");
      }
    }
    // fill data buffer or go to next bytes
    if(packet_len > 0) {
      for(i = 0; i < packet_len; i++) {
        data_buffer[i] = c->buf[c->start];
        c->start++;
        c->length--;
      }
      count++;
    }
    else {
      c->start += 3;
      c->length -= 3;
    }
  }

  return count;
}

// Long (32bits) in buffer are little endian in gps message
#define LongOfBuf(_buf,_idx) (int32_t)(((uint32_t)_buf[_idx+3]<<24)|((uint32_t)_buf[_idx+2]<<16)|((uint32_t)_buf[_idx+1]<<8)|((uint32_t)_buf[_idx]))
// Unsigned short (16bits) in buffer are little endian in gps message
#define UShortOfBuf(_buf,_idx) (uint16_t)(((uint16_t)_buf[_idx+1]<<8)|((uint16_t)_buf[_idx]))
// Short (16bits) in buffer are big endian in other messages
#define ShortOfBuf(_buf,_idx) (int16_t)(((uint16_t)_buf[_idx]<<8)|((uint16_t)_buf[_idx+1]))

/***************************************************************************************
 *decode the gps data packet
 ***************************************************************************************/
static void decode_gpspacket(struct NpsFdm * fdm, byte* buffer)
{
  /* gps velocity (1e2 m/s to  m/s */
  struct NedCoor_d vel;
  vel.x = (double)LongOfBuf(buffer,3)*1.0e-2;
  vel.y = (double)LongOfBuf(buffer,7)*1.0e-2;
  vel.z = (double)LongOfBuf(buffer,11)*1.0e-2;
  fdm->ltp_ecef_vel = vel;
  ecef_of_ned_vect_d(&fdm->ecef_ecef_vel, &ltpdef, &vel);

  /* gps position (1e7 deg to rad and 1e3 m to m) */
  struct LlaCoor_d pos;
  pos.lon=(double)LongOfBuf(buffer,15)*1.74533e-9;
  pos.lat=(double)LongOfBuf(buffer,19)*1.74533e-9;
  pos.alt=(double)LongOfBuf(buffer,23)*1.0e-3;

  pos.lat += ltpdef.lla.lat;
  pos.lon += ltpdef.lla.lon;
  pos.alt += ltpdef.lla.alt;

  fdm->lla_pos = pos;
  ecef_of_lla_d(&fdm->ecef_pos, &pos);
  fdm->hmsl = pos.alt - NAV_MSL0/1000.;

  /* gps time */
  fdm->time = (double)UShortOfBuf(buffer,27);

  /* in LTP pprz */
  ned_of_ecef_point_d(&fdm->ltpprz_pos, &ltpdef, &fdm->ecef_pos);
  fdm->lla_pos_pprz = pos;
  ned_of_ecef_vect_d(&fdm->ltpprz_ecef_vel, &ltpdef, &fdm->ecef_ecef_vel);

#if NPS_CRRCSIM_DEBUG
  printf("decode gps | pos %f %f %f | vel %f %f %f | time %f\n",
      57.3*fdm->lla_pos.lat,
      57.3*fdm->lla_pos.lon,
      fdm->lla_pos.alt,
      fdm->ltp_ecef_vel.x,
      fdm->ltp_ecef_vel.y,
      fdm->ltp_ecef_vel.z,
      fdm->time);
#endif
}

/***************************************************************************************
 *decode the ahrs data packet
 ***************************************************************************************/
static void decode_ahrspacket(struct NpsFdm * fdm, byte* buffer)
{
  /* euler angles (0.9387340515702713e04 rad to rad) */
  fdm->ltp_to_body_eulers.phi = (double)ShortOfBuf(buffer,1)*0.000106526 - NPS_CRRCSIM_ROLL_NEUTRAL;
  fdm->ltp_to_body_eulers.theta = (double)ShortOfBuf(buffer,3)*0.000106526 - NPS_CRRCSIM_PITCH_NEUTRAL;
  fdm->ltp_to_body_eulers.psi = (double)ShortOfBuf(buffer,5)*0.000106526;
  DOUBLE_QUAT_OF_EULERS(fdm->ltp_to_body_quat, fdm->ltp_to_body_eulers);

#if NPS_CRRCSIM_DEBUG
  printf("decode ahrs %f %f %f\n",
      fdm->ltp_to_body_eulers.phi*57.3,
      fdm->ltp_to_body_eulers.theta*57.3,
      fdm->ltp_to_body_eulers.psi*57.3);
#endif
}

/***************************************************************************************
 *decode the imu data packet
 ***************************************************************************************/
void decode_imupacket(struct NpsFdm * fdm, byte* buffer)
{
  /* acceleration (0.1670132517315938e04 m/s^2 to m/s^2) */
  fdm->body_accel.x = (double)ShortOfBuf(buffer,3)*5.98755e-04;
  fdm->body_accel.y = (double)ShortOfBuf(buffer,5)*5.98755e-04;
  fdm->body_accel.z = (double)ShortOfBuf(buffer,7)*5.98755e-04;

  /* since we don't get acceleration in ecef frame, use ECI for now */
  fdm->body_ecef_accel.x = fdm->body_accel.x;
  fdm->body_ecef_accel.y = fdm->body_accel.y;
  fdm->body_ecef_accel.z = fdm->body_accel.z;


  /* angular rate (0.9387340515702713e4 rad/s to rad/s) */
  fdm->body_inertial_rotvel.p = (double)ShortOfBuf(buffer,9)*1.06526e-04;
  fdm->body_inertial_rotvel.q = (double)ShortOfBuf(buffer,11)*1.06526e-04;
  fdm->body_inertial_rotvel.r = (double)ShortOfBuf(buffer,13)*1.06526e-04;

  /* since we don't get angular velocity in ECEF frame, use the rotvel in ECI frame for now */
  fdm->body_ecef_rotvel.p = fdm->body_inertial_rotvel.p;
  fdm->body_ecef_rotvel.q = fdm->body_inertial_rotvel.q;
  fdm->body_ecef_rotvel.r = fdm->body_inertial_rotvel.r;

  /* magnetic field in Gauss */
  //fdm->mag.x = (double)ShortOfBuf(buffer,15)*6.10352e-05;
  //fdm->mag.y = (double)ShortOfBuf(buffer,17)*6.10352e-05;
  //fdm->mag.z = (double)ShortOfBuf(buffer,19)*6.10352e-05;

  /* pressure in m and m/s */
  //data->Ps = (double)ShortOfBuf(buffer,27)*3.05176e-01;
  //data->Pt = (double)ShortOfBuf(buffer,29)*2.44141e-03;

#if NPS_CRRCSIM_DEBUG
  printf("decode imu | accel %f %f %f | gyro %f %f %f\n",
      fdm->body_accel.x,
      fdm->body_accel.y,
      fdm->body_accel.z,
      fdm->body_inertial_rotvel.p,
      fdm->body_inertial_rotvel.q,
      fdm->body_inertial_rotvel.r);
#endif
}

// compatibility with OSX
#ifdef __APPLE__
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif

/***************************************************************************************
 * send servo command over udp
 ***************************************************************************************/
static void send_servo_cmd(struct _crrcsim* io, double* commands)
{
  //cnt_cmd[1] = ch1:elevator, cnt_cmd[0] = ch0:aileron, cnt_cmd[2] = ch2:throttle
  word cnt_cmd[3];
  byte  data[24]={0,};
  short i = 0;
  word  sum=0;

  word roll = (word)((65535/4)*commands[NPS_CRRCSIM_COMMAND_ROLL] + (65536/2));
  word pitch = (word)((65535/4)*commands[NPS_CRRCSIM_COMMAND_PITCH] + (65536/2));

  cnt_cmd[0] = roll;
  cnt_cmd[1] = -pitch;
  cnt_cmd[2] = (word)(65535*commands[NPS_CRRCSIM_COMMAND_THROTTLE]);

#if NPS_CRRCSIM_DEBUG
  printf("send servo %f %f %f | %d %d %d | %d %d\n",
      commands[0],
      commands[1],
      commands[2],
      cnt_cmd[0],
      cnt_cmd[1],
      cnt_cmd[2],
      roll, pitch);
#endif

  data[0] = 0x55;
  data[1] = 0x55;
  data[2] = 0x53;
  data[3] = 0x53;

  //aileron ch#0,elevator ch#1,throttle ch#2
  //aileron
  data[4] = (byte)(cnt_cmd[0] >> 8);
  data[5] = (byte)cnt_cmd[0];
  //elevator
  data[6] = (byte)(cnt_cmd[1] >> 8);
  data[7] = (byte)cnt_cmd[1];
  //throttle
  data[8] = (byte)(cnt_cmd[2] >> 8);
  data[9] = (byte)cnt_cmd[2];

  //checksum
  sum = 0xa6; //0x53+0x53
  for(i=4;i<22;i++) sum += data[i];

  data[22] = (byte)(sum >> 8);
  data[23] = (byte)sum;

  //sendout the command packet
  if (io->socket >= 0) {
    send(io->socket, (char*)data, 24, MSG_NOSIGNAL);
  }
}

