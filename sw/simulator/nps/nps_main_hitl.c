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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "termios.h"
#include <fcntl.h>
#include <unistd.h>


#include "paparazzi.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"
#include "pprzlink/pprz_transport.h"
/* Message id helpers */
#define SenderIdOfPprzMsg(x) (x[0])
#define IdOfPprzMsg(x) (x[1])

#include "nps_main.h"
//#include "nps_fdm.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"
#include "nps_autopilot.h"
#include "nps_ivy.h"
#include "nps_flightgear.h"
#include "mcu_periph/sys_time.h"

void* nps_ins_data_loop(void* data __attribute__((unused)));
void* nps_ap_data_loop(void* data __attribute__((unused)));

pthread_t th_ins_data; // sends INS packets to the autopilot
pthread_t th_ap_data; // receives commands from the autopilot

/*
 * Vectornav info
 */
#define VN_DATA_START 10
#define VN_BUFFER_SIZE 512

static uint8_t VN_SYNC = 0xFA;
static uint8_t VN_OUTPUT_GROUP = 0x39;
static uint16_t VN_GROUP_FIELD_1 = 0x01E9;
static uint16_t VN_GROUP_FIELD_2 = 0x061A;
static uint16_t VN_GROUP_FIELD_3 = 0x0140;
static uint16_t VN_GROUP_FIELD_4 = 0x0009;

uint8_t vn_buffer[VN_BUFFER_SIZE];

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

/**
 * Calculates the 16-bit CRC for the given ASCII or binary message.
 * The CRC is calculated over the packet starting just after the sync byte (not including the sync byte)
 * and ending at the end of payload.
 */
unsigned short vn_calculate_crc(unsigned char data[], unsigned int length)
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


int main(int argc, char **argv)
{
  nps_main_init(argc, argv);

  pthread_create(&th_flight_gear,NULL,nps_flight_gear_loop,NULL);
  pthread_create(&th_display_ivy,NULL,nps_main_display,NULL);
  pthread_create(&th_main_loop,NULL,nps_main_loop,NULL);
  pthread_create(&th_ins_data,NULL,nps_ins_data_loop,NULL);
  pthread_create(&th_ap_data,NULL,nps_ap_data_loop,NULL);
  pthread_join(th_main_loop, NULL);

  return 0;
}

void nps_main_run_sim_step(void)
{
  // TODO: fix mutexes, then enable again
  //nps_atmosphere_update(SIM_DT);
  nps_fdm_run_step(autopilot.launch, autopilot.commands, NPS_COMMANDS_NB);
}

#define GPS_SEC_IN_DAY 86400
/**
 * @return GPS TOW
 */
static uint64_t vn_get_time_of_week(void){
  struct timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;
  struct tm t_res;
  localtime_r(&curTime.tv_sec, &t_res);
  struct tm * tt = &t_res;

  uint64_t tow = GPS_SEC_IN_DAY*tt->tm_wday + 3600*tt->tm_hour + 60*tt->tm_min + tt->tm_sec; // sec
  tow = tow * 1000; // tow to ms
  tow = tow + milli; // tow with added ms
  tow = tow * 1e6; // tow in nanoseconds

  return tow;
}



void* nps_ins_data_loop(void* data __attribute__((unused)))
{
  // configure port
  int fd = open(INS_DEV, O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
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
  cfsetispeed(&new_settings, (speed_t)INS_BAUD);
  cfsetospeed(&new_settings, (speed_t)INS_BAUD);
  tcsetattr(fd, TCSANOW, &new_settings);

  //gint64 end_time;
  struct timespec requestStart, requestEnd, waitFor;

  while (TRUE)
  {
    //g_mutex_lock(&fdm_mutex);
    pthread_mutex_lock(&fdm_mutex);
    //end_time = g_get_monotonic_time () + (1./100.) * G_TIME_SPAN_MILLISECOND;
    clock_gettime(CLOCK_REALTIME, &requestStart);

    // fetch data
    // Timestamp
    vn_data.TimeStartup = (uint64_t)(fdm.time * 1000000000.0);

    //Attitude, float, [degrees], yaw, pitch, roll, NED frame
    vn_data.YawPitchRoll[0] = DegOfRad((float)fdm.ltp_to_body_eulers.psi); // yaw
    vn_data.YawPitchRoll[1] = DegOfRad((float)fdm.ltp_to_body_eulers.theta); // pitch
    vn_data.YawPitchRoll[2] = DegOfRad((float)fdm.ltp_to_body_eulers.phi); // roll

    // Rates (imu frame), float, [rad/s]
    vn_data.AngularRate[0] = (float)fdm.body_ecef_rotvel.p;
    vn_data.AngularRate[1] = (float)fdm.body_ecef_rotvel.q;
    vn_data.AngularRate[2] = (float)fdm.body_ecef_rotvel.r;

    //Pos LLA, double,[beg, deg, m]
    //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
    vn_data.Position[0] = DegOfRad(fdm.lla_pos.lat);
    vn_data.Position[1] = DegOfRad(fdm.lla_pos.lon);
    vn_data.Position[2] = fdm.lla_pos.alt; // TODO: make sure it shows the correct starting point

    //VelNed, float [m/s]
    //The estimated velocity in the North East Down (NED) frame, given in m/s.
    vn_data.Velocity[0] = (float)fdm.ltp_ecef_vel.x;
    vn_data.Velocity[1] = (float)fdm.ltp_ecef_vel.y;
    vn_data.Velocity[2] = (float)fdm.ltp_ecef_vel.z;

    // Accel (imu-frame), float, [m/s^-2]
    vn_data.Accel[0] = (float)fdm.body_ecef_accel.x;
    vn_data.Accel[1] = (float)fdm.body_ecef_accel.y;
    vn_data.Accel[2] = (float)fdm.body_ecef_accel.z;

    // tow (in nanoseconds), uint64
    vn_data.Tow = vn_get_time_of_week();

    //num sats, uint8
    vn_data.NumSats = 8; // random number

    //gps fix, uint8
    vn_data.Fix = 3; // 3D fix

    //posU, float[3]
    // TODO

    //velU, float
    // TODO

    //linear acceleration imu-body frame, float [m/s^2]
    vn_data.LinearAccelBody[0] = (float)fdm.ltp_ecef_vel.x;
    vn_data.LinearAccelBody[1] = (float)fdm.ltp_ecef_vel.y;
    vn_data.LinearAccelBody[2] = (float)fdm.ltp_ecef_vel.z;

    //YprU, float[3]
    // TODO

    //instatus, uint16
    vn_data.InsStatus = 0x02;

    //Vel body, float [m/s]
    // The estimated velocity in the body (i.e. imu) frame, given in m/s.
    vn_data.VelBody[0] = (float)fdm.body_accel.x;
    vn_data.VelBody[1] = (float)fdm.body_accel.y;
    vn_data.VelBody[2] = (float)fdm.body_accel.z;

    // unlock mutex
    //g_mutex_unlock(&fdm_mutex);
    pthread_mutex_unlock(&fdm_mutex);

    // lock ins mutex
    //g_mutex_lock(&ins_mutex);
    // send ins data here
    static uint16_t idx;

    vn_buffer[0] = VN_SYNC;
    vn_buffer[1] = VN_OUTPUT_GROUP;
    vn_buffer[2] = (uint8_t) (VN_GROUP_FIELD_1 >> 8);
    vn_buffer[3] = (uint8_t) (VN_GROUP_FIELD_1);
    vn_buffer[4] = (uint8_t) (VN_GROUP_FIELD_2 >> 8);
    vn_buffer[5] = (uint8_t) (VN_GROUP_FIELD_2);
    vn_buffer[6] = (uint8_t) (VN_GROUP_FIELD_3 >> 8);
    vn_buffer[7] = (uint8_t) (VN_GROUP_FIELD_3);
    vn_buffer[8] = (uint8_t) (VN_GROUP_FIELD_4 >> 8);
    vn_buffer[9] = (uint8_t) (VN_GROUP_FIELD_4);

    idx = VN_DATA_START;

    // Timestamp
    memcpy(&vn_buffer[idx], &vn_data.TimeStartup, sizeof(uint64_t));
    idx += sizeof(uint64_t);

    //Attitude, float, [degrees], yaw, pitch, roll, NED frame
    memcpy(&vn_buffer[idx], &vn_data.YawPitchRoll, 3*sizeof(float));
    idx += 3*sizeof(float);

    // Rates (imu frame), float, [rad/s]
    memcpy(&vn_buffer[idx], &vn_data.AngularRate, 3*sizeof(float));
    idx += 3*sizeof(float);

    //Pos LLA, double,[beg, deg, m]
    //The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectfully.
    memcpy(&vn_buffer[idx], &vn_data.Position, 3*sizeof(double));
    idx += 3*sizeof(double);

    //VelNed, float [m/s]
    //The estimated velocity in the North East Down (NED) frame, given in m/s.
    memcpy(&vn_buffer[idx], &vn_data.Velocity, 3*sizeof(float));
    idx += 3*sizeof(float);

    // Accel (imu-frame), float, [m/s^-2]
    memcpy(&vn_buffer[idx], &vn_data.Accel, 3*sizeof(float));
    idx += 3*sizeof(float);

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
    memcpy(&vn_buffer[idx], &vn_data.PosU, 3*sizeof(float));
    idx += 3*sizeof(float);

    //velU, float
    memcpy(&vn_buffer[idx], &vn_data.VelU, sizeof(float));
    idx += sizeof(float);

    //linear acceleration imu-body frame, float [m/s^2]
    memcpy(&vn_buffer[idx], &vn_data.LinearAccelBody, 3*sizeof(float));
    idx += 3*sizeof(float);

    //YprU, float[3]
    memcpy(&vn_buffer[idx], &vn_data.YprU, 3*sizeof(float));
    idx += 3*sizeof(float);

    //instatus, uint16
    memcpy(&vn_buffer[idx], &vn_data.InsStatus, sizeof(uint16_t));
    idx += sizeof(uint16_t);

    //Vel body, float [m/s]
    // The estimated velocity in the body (i.e. imu) frame, given in m/s.
    memcpy(&vn_buffer[idx], &vn_data.VelBody, 3*sizeof(float));
    idx += 3*sizeof(float);

    // calculate checksum & send
    uint16_t chk = vn_calculate_crc(&vn_buffer[1], idx-1);
    vn_buffer[idx] = (uint8_t) (chk >> 8);
    idx++;
    vn_buffer[idx] = (uint8_t) (chk & 0xFF);
    idx++;


    static int wlen;
    wlen = write(fd, &vn_buffer, idx);
    if (wlen != idx){
      printf("vectornav Warning: sent only %u bytes to VN, instead of expected %u\n",wlen,idx);
    }

    tcdrain(fd); // delay for output

    // TODO: this doesn't seem to be waiting long enough, I have a constant rate of about 240 packets/second
    // no matter what...
    //g_cond_wait_until (&ins_cond, &ins_mutex, end_time);
    //g_mutex_unlock (&ins_mutex);

    clock_gettime(CLOCK_REALTIME, &requestEnd);

    // Calculate time it took
    //double accum = (requestEnd.tv_sec - requestStart.tv_sec) + (requestEnd.tv_nsec - requestStart.tv_nsec)/ 1E9;
    long int accum_ns = (requestEnd.tv_sec - requestStart.tv_sec)*1000000000L + (requestEnd.tv_nsec - requestStart.tv_nsec);

    if (accum_ns > 0) {
      waitFor.tv_sec = 0;
      waitFor.tv_nsec = (1./100.)*1000000000L - accum_ns;

      //printf("INS THREAD: Worked for %f ms, waiting for another %f ms\n", (double)accum_ns/1E6, waitFor.tv_nsec/1E6);

      nanosleep(&waitFor,NULL);
    }
    else {
      printf("INS THREAD: took too long, exactly %f ms\n", (double)accum_ns/1E6);
    }
  }

  return(NULL);
}



void* nps_ap_data_loop(void* data __attribute__((unused)))
{
	int cnt = 0;

  // configure port
  int fd = open(AP_DEV, O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    printf("AP data loop error opening port %i\n", fd);
    return(NULL);
  }

  // TODO replace with normal serial port?
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
  uint8_t buf[90];

  //bool dl_msg_available = FALSE;
  struct pprz_transport pprz_tp_logger;


  while (TRUE)
  {
    // receive and update the ap commands
    rdlen = read(fd, buf, sizeof(buf) - 1);

    for(int i=0;i<rdlen;i++){
      // parse
      parse_pprz(&pprz_tp_logger, buf[i]);

      // if msg_available read
      if (pprz_tp_logger.trans_rx.msg_received) {
        for (int k = 0; k < pprz_tp_logger.trans_rx.payload_len; k++) {
          buf[k] = pprz_tp_logger.trans_rx.payload[k];
        }
        //Parse message;
        //uint8_t sender_id = SenderIdOfPprzMsg(buf); // TODO: check sender ID
        uint8_t msg_id = IdOfPprzMsg(buf);
        //printf("Received message, sender_id=%u, msg_id=%u\n",sender_id,msg_id);

        uint8_t cmd_len;
        int16_t cmd_buf[5]; // TODO: replace with varible number of commands
        /*
        int16_t cmd_throttle;
        int16_t cmd_roll;
        int16_t cmd_pitch;
        int16_t cmd_yaw;
        int16_t cmd_flap;
        */



        // process readings
        switch(msg_id){
          case DL_COMMANDS:
            // parse commands message
            cmd_len = DL_COMMANDS_values_length(buf);
            //printf("Commands legth: %u\n",cmd_len);

            memcpy(&cmd_buf, DL_COMMANDS_values(buf), cmd_len*sizeof(int16_t));

            // display commands
            /*
            cmd_throttle = cmd_buf[0];
            cmd_roll = cmd_buf[1];
            cmd_pitch = cmd_buf[2];
            cmd_yaw = cmd_buf[3];
            cmd_flap = cmd_buf[4];
            */

            /*
            printf("cmd_throttle = %d\n",cmd_throttle);
            printf("cmd_roll = %d\n",cmd_roll);
            printf("cmd_pitch = %d\n",cmd_pitch);
            printf("cmd_yaw = %d\n",cmd_yaw);
            printf("cmd_flap = %d\n",cmd_flap);
            */

            //g_mutex_lock(&fdm_mutex);
            pthread_mutex_lock(&fdm_mutex);
            cnt++;
            // update commands
            for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
              autopilot.commands[i] = (double)cmd_buf[i] / MAX_PPRZ;
            }
            // hack: invert pitch to fit most JSBSim models
            autopilot.commands[COMMAND_PITCH] = -(double)cmd_buf[COMMAND_PITCH] / MAX_PPRZ;
            //g_mutex_unlock(&fdm_mutex);

//            if( ( cnt % 100 ) == 0){
//            	printf("AP: Got another 100 commands, total of %i\n",cnt);
//            }


            pthread_mutex_unlock(&fdm_mutex);



            break;
          default:
            break;
        }
        pprz_tp_logger.trans_rx.msg_received = false;
      }
    }
  }

  return(NULL);
}




void* nps_main_loop(void* data __attribute__((unused)))
{
  struct timespec requestStart, requestEnd, waitFor;
  int cnt = 0;

  // check the sim time difference from the realtime
  // fdm.time - simulation time
  //
  struct timespec startTime;
  struct timespec realTime;
  clock_gettime(CLOCK_REALTIME, &startTime);
  double start_secs = ntime_to_double(&startTime);
  double real_secs = 0;
  //double time_diff = 0;
  double real_time = 0;


  while (TRUE)
  {
    //g_mutex_lock(&fdm_mutex);

    //end_time = g_get_monotonic_time () + (1./100.) * G_TIME_SPAN_MILLISECOND;
    clock_gettime(CLOCK_REALTIME, &requestStart);

    pthread_mutex_lock(&fdm_mutex);

    // check the current simulation time
    clock_gettime(CLOCK_REALTIME, &realTime);
    real_secs = ntime_to_double(&realTime);
    real_time = real_secs - start_secs; // real time elapsed
    //time_diff = real_time - fdm.time;

    static int guard;

    guard = 0;
    while ((real_time - fdm.time) > SIM_DT) {
      //printf("MAIN SIML Ouch! (real_time - fdm.time) = %f[s], sim_dt=%f\n",(real_time - fdm.time), SIM_DT);
      nps_main_run_sim_step();
      cnt++;
      guard++;
      if (guard>2){
        //printf("MAIN SIML Ouch! Too much behind!\n");
        break;
      }
    }
    //printf("MAIN SIML Now it is OK! (real_time - fdm.time) = %f[s]\n",(real_time - fdm.time));

//    if( ( cnt % 100 ) == 0){
//    	printf("MAIN SIML Got another 100 sim steps, total of %i\n",cnt);
//    }
    pthread_mutex_unlock(&fdm_mutex);

    clock_gettime(CLOCK_REALTIME, &requestEnd);

    // Calculate time it took
    //double accum = (requestEnd.tv_sec - requestStart.tv_sec) + (requestEnd.tv_nsec - requestStart.tv_nsec)/ 1E9;
    long int accum_ns = (requestEnd.tv_sec - requestStart.tv_sec)*1000000000L + (requestEnd.tv_nsec - requestStart.tv_nsec);

    if (accum_ns > 0) {
      waitFor.tv_sec = 0;
      waitFor.tv_nsec = SIM_DT*1000000000L - accum_ns;

      //printf("MAIN THREAD: Worked for %f ms, waiting for another %f ms\n", (double)accum_ns/1E6, waitFor.tv_nsec/1E6);

      nanosleep(&waitFor,NULL);
    }
    else {
      printf("MAIN THREAD: took too long, exactly %f ms\n", (double)accum_ns/1E6);
    }

  }

  return(NULL);
}


