/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <event.h>

#include "messages2.h"

#include "fms_periodic.h"
#include "fms_debug.h"
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"
#include "booz/booz_imu.h"

#include "overo_file_logger.h"
#include "overo_gcs_com.h"
#include "overo_estimator.h"
#include "overo_controller.h"


static void main_periodic(int);
static void main_parse_cmd_line(int argc, char *argv[]);
static void main_exit(int sig);
static void main_talk_with_stm32(void);


static struct AutopilotMessageBethUp   msg_in;
static struct AutopilotMessageBethDown msg_out;

struct BoozImu booz_imu;
struct BoozImuFloat booz_imu_float;


static uint32_t foo = 0;


int main(int argc, char *argv[]) {
  
  (void) signal(SIGINT, main_exit);
  
  RATES_ASSIGN(booz_imu.gyro_neutral,  IMU_GYRO_P_NEUTRAL,  IMU_GYRO_Q_NEUTRAL,  IMU_GYRO_R_NEUTRAL);
  VECT3_ASSIGN(booz_imu.accel_neutral, IMU_ACCEL_X_NEUTRAL, IMU_ACCEL_Y_NEUTRAL, IMU_ACCEL_Z_NEUTRAL);
  VECT3_ASSIGN(booz_imu.mag_neutral,   IMU_MAG_X_NEUTRAL,   IMU_MAG_Y_NEUTRAL,   IMU_MAG_Z_NEUTRAL);
  
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }
  
  /* Initalize the event library */
  event_init();
  
  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return -1; 
  }

  //  file_logger_init("my_log.data");

  gcs_com_init();

  main_parse_cmd_line(argc, argv);
  
  event_dispatch();
 
  printf("goodbye! (%d)\n",foo);

  return 0;
}



static void main_periodic(int my_sig_num) {
#if 0
DOWNLINK_SEND_ALIVE(gcs_com.udp_transport, 16, MD5SUM);
main_talk_with_stm32();
estimator_run(msg_in.bench_sensor.z);
control_run();
BoozImuScaleGyro();
gcs_com_periodic();
#endif

#if 01 
  RunOnceEvery(50, {DOWNLINK_SEND_ALIVE(gcs_com.udp_transport, 16, MD5SUM);});
 
  main_talk_with_stm32();

  RunOnceEvery(50, {DOWNLINK_SEND_BETH(gcs_com.udp_transport,&msg_in.bench_sensor.x,&msg_in.bench_sensor.y,
				       &msg_in.bench_sensor.z,&foo);});
  
  estimator_run(msg_in.bench_sensor.z);
 
  control_run();

  //  file_logger_periodic();


  RunOnceEvery(10, {DOWNLINK_SEND_IMU_GYRO_RAW(gcs_com.udp_transport,
			     //&msg_in.gyro.p,&msg_in.gyro.q,&msg_in.gyro.r)
				&booz_imu.gyro_unscaled.p,&booz_imu.gyro_unscaled.q,&booz_imu.gyro_unscaled.r);});
    

  RunOnceEvery(10, {DOWNLINK_SEND_IMU_ACCEL_RAW(gcs_com.udp_transport,
			     //&msg_in.accel.x,&msg_in.accel.y,&msg_in.accel.z
				&booz_imu.accel_unscaled.x,&booz_imu.accel_unscaled.y,&booz_imu.accel_unscaled.z);});

  BoozImuScaleGyro();
  RunOnceEvery(50, {DOWNLINK_SEND_BOOZ2_GYRO(gcs_com.udp_transport,
			     //&msg_in.gyro.p,&msg_in.gyro.q,&msg_in.gyro.r)
				&booz_imu.gyro.p,&booz_imu.gyro.q,&booz_imu.gyro.r);});
    

/*  RunOnceEvery(50, {DOWNLINK_SEND_BOOZ2_ACCEL(DefaultChannel,
			     //&msg_in.accel.x,&msg_in.accel.y,&msg_in.accel.z
				&booz_imu.accel.x,&booz_imu.accel.y,&booz_imu.accel.z);});*/

  //RunOnceEvery(33, {UdpTransportPeriodic();});
  RunOnceEvery(33, gcs_com_periodic());
#endif

}


static void main_parse_cmd_line(int argc, char *argv[]) {

  if (argc>1){
    controller.kp = atof(argv[1]);
    //    printf("kp set to %f\n",kp);
    if (argc>2) {
      controller.kd = atof(argv[2]);
      //      printf("kd set to %f\n",kd);
    } else {
      controller.kd=1.0;
      //      printf("using default value of kd %f\n",kd);
    }
  } else {
    controller.kp = 0.05;
    //    printf("using default value of kp %f\n",kp);
  }

}


static void main_exit(int sig) {
  printf("Closing down\n");
  //file_logger_exit()
  exit(EXIT_SUCCESS);
}


static void main_talk_with_stm32() {

  static int8_t adder = 1;
  //uint8_t *fooptr;

  msg_out.thrust = 10;

  if (!(foo%100)) { 
    /*if (pitchval == 15 ) adder=-1;
    if (pitchval == -15 ) adder=1;
    pitchval = pitchval + adder;*/
    printf("cmd now %f\n",controller.cmd);
  }
  msg_out.pitch = (int8_t)controller.cmd;
  //msg_out.pitch = 1;
  //msg_out.cksum = msg_out.thrust + msg_out.pitch;

  spi_link_send(&msg_out, sizeof(union AutopilotMessage) , &msg_in);

  /*if (msg_in.bench_sensor.x == 0){
    fooptr = &msg_in;
    for (int i=0; i<sizeof(msg_in); i++)
      printf("%02x ",  fooptr[i]);
    printf("\n");
  }*/

  /*if (!(foo%10)) { 
    printf("%d %d %d %x\r\n",msg_in.bench_sensor.x,msg_in.bench_sensor.y,
    msg_in.bench_sensor.z,msg_in.bench_sensor.x);   
  }*/
  foo++;

  booz_imu.gyro_unscaled.p = (msg_in.gyro.p&0xFFFF);
  booz_imu.gyro_unscaled.q = (msg_in.gyro.q&0xFFFF);
  booz_imu.gyro_unscaled.r = (msg_in.gyro.r&0xFFFF);
  booz_imu.accel_unscaled.x = (msg_in.accel.x&0xFFFF);
  booz_imu.accel_unscaled.y = (msg_in.accel.y&0xFFFF);
  booz_imu.accel_unscaled.z = (msg_in.accel.z&0xFFFF);

}




