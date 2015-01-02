/*
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
#include "generated/airframe.h"

#include "fms_periodic.h"
#include "fms_debug.h"
#include "fms_spi_link.h"
#include "fms_autopilot_msg.h"
#include "subsystems/imu.h"

#include "overo_file_logger.h"
#include "overo_gcs_com.h"
#include "overo_estimator.h"
#include CONTROLLER_H


static void main_periodic(int);
//static void main_parse_cmd_line(int argc, char *argv[]);
static void drive_output(void);
static void main_exit(int sig);
static void main_talk_with_stm32(void);


static struct AutopilotMessageCRCFrame   msg_in;
static struct AutopilotMessageCRCFrame   msg_out;

struct Imu imu;
struct ImuFloat imu_float;


static uint32_t foo = 0;
static uint8_t spi_crc_ok = 1;
static uint8_t last_state = 1;

int main(int argc, char *argv[])
{

  (void) signal(SIGINT, main_exit);

  //set IMU neutrals
  RATES_ASSIGN(imu.gyro_neutral,  IMU_GYRO_P_NEUTRAL,  IMU_GYRO_Q_NEUTRAL,  IMU_GYRO_R_NEUTRAL);
  VECT3_ASSIGN(imu.accel_neutral, IMU_ACCEL_X_NEUTRAL, IMU_ACCEL_Y_NEUTRAL, IMU_ACCEL_Z_NEUTRAL);
  VECT3_ASSIGN(imu.mag_neutral,   IMU_MAG_X_NEUTRAL,   IMU_MAG_Y_NEUTRAL,   IMU_MAG_Z_NEUTRAL);

  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }

  /* Initalize the event library */
  event_init();

  control_init();
  estimator_init();

  //  file_logger_init("my_log.data");

  gcs_com_init();

  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return -1;
  }

  //main_parse_cmd_line(argc, argv);

  event_dispatch();
  //should never occur!
  printf("goodbye! (%d)\n", foo);

  return 0;
}

#define PITCH_MAGIC_NUMBER (121)

static void main_periodic(int my_sig_num)
{

  /*  static int bar=0;
    if (!(foo%2000)) {
      if (bar) {
        controller.tilt_sp = -0.4; bar=0;
      }else{
        controller.tilt_sp = 0.4; bar=1;
      }
    }
  */
  //if (foo >2000 ) { controller.armed=1;}

  RunOnceEvery(50, {DOWNLINK_SEND_ALIVE(gcs_com.udp_transport, 16, MD5SUM);});

  main_talk_with_stm32();

  imu_scale_gyro(&imu);

  RunOnceEvery(50, {DOWNLINK_SEND_BETH(gcs_com.udp_transport,
                                       &msg_in.payload.msg_up.bench_sensor.x, &msg_in.payload.msg_up.bench_sensor.y,
                                       &msg_in.payload.msg_up.bench_sensor.z, &msg_in.payload.msg_up.cnt,
                                       &msg_in.payload.msg_up.can_errs, &msg_in.payload.msg_up.spi_errs,
                                       &msg_in.payload.msg_up.thrust_out, &msg_in.payload.msg_up.pitch_out);
                   });

  estimator_run(msg_in.payload.msg_up.bench_sensor.z, msg_in.payload.msg_up.bench_sensor.y,
                msg_in.payload.msg_up.bench_sensor.x);

  if (msg_in.payload.msg_up.cnt == 0) printf("STM indicates overo link is lost! %d %d\n",
        msg_in.payload.msg_up.cnt, msg_in.payload.msg_up.can_errs);
  if (msg_in.payload.msg_up.cnt == 1) printf("STM indicates overo link is regained. %d %d\n",
        msg_in.payload.msg_up.cnt, msg_in.payload.msg_up.can_errs);

  //If the stm32 cut the motors due to an error, we force the state machine into spinup mode.
  //when the stm32 resumes after the error, the system will need to be rearmed by the user.
  if ((controller.armed != 0) && (msg_in.payload.msg_up.pitch_out == PITCH_MAGIC_NUMBER)) {
    controller.armed = 0; last_state = 1;
    printf("STM cut motor power. %d %d\n",
           msg_in.payload.msg_up.cnt, msg_in.payload.msg_up.can_errs);
  }


  drive_output();

  control_send_messages();

  estimator_send_messages();

  //file_logger_periodic();

  /*  RunOnceEvery(10, {DOWNLINK_SEND_IMU_GYRO_RAW(gcs_com.udp_transport,
             //&msg_in.payload.msg_up.gyro.p,&msg_in.payload.msg_up.gyro.q,&msg_in.payload.msg_up.gyro.r)
          &imu.gyro_unscaled.p,&imu.gyro_unscaled.q,&imu.gyro_unscaled.r);});
    RunOnceEvery(10, {DOWNLINK_SEND_IMU_ACCEL_RAW(gcs_com.udp_transport,
             //&msg_in.payload.msg_up.accel.x,&msg_in.payload.msg_up.accel.y,&msg_in.payload.msg_up.accel.z
          &imu.accel_unscaled.x,&imu.accel_unscaled.y,&imu.accel_unscaled.z);})
    RunOnceEvery(50, {DOWNLINK_SEND_IMU_GYRO_SCALED(gcs_com.udp_transport,
             //&msg_in.payload.msg_up.gyro.p,&msg_in.payload.msg_up.gyro.q,&msg_in.payload.msg_up.gyro.r)
          &imu.gyro.p,&imu.gyro.q,&imu.gyro.r);});

    RunOnceEvery(50, {DOWNLINK_SEND_AHRS_EULER(gcs_com.udp_transport,
        &estimator.tilt, &estimator.elevation, &estimator.azimuth );});
    RunOnceEvery(50, {DOWNLINK_SEND_IMU_ACCEL_SCALED(DefaultChannel,
             //&msg_in.payload.msg_up.accel.x,&msg_in.payload.msg_up.accel.y,&msg_in.payload.msg_up.accel.z
          &imu.accel.x,&imu.accel.y,&imu.accel.z);});*/

  RunOnceEvery(33, gcs_com_periodic());

}

#if 0
static void main_parse_cmd_line(int argc, char *argv[])
{

  if (argc > 1) {
    controller.kp = atof(argv[1]);
    //    printf("kp set to %f\n",kp);
    if (argc > 2) {
      controller.kd = atof(argv[2]);
      //      printf("kd set to %f\n",kd);
    } else {
      controller.kd = 1.0;
      //      printf("using default value of kd %f\n",kd);
    }
  } else {
    controller.kp = 0.05;
    //    printf("using default value of kp %f\n",kp);
  }
  /*
    if (argc>1){
      printf("args not currently supported\n");
    }*/
}
#endif

static void drive_output()
{
  switch (controller.armed) {
    case 0:
      if (last_state == 2) {
        controller.armed = 2;
        printf("Not allowed. Enter standby first.\n");
        //controller.elevation_sp = ((int32_t)(foo/2048.)%2) ? RadOfDeg(-20) : RadOfDeg(10);
        control_run();
      } else {
        if (last_state == 1) {
          printf("Entering spinup mode.\n");
        }
        controller.cmd_pitch = 1;
        controller.cmd_thrust = 1;
        last_state = 0;
      }
      break;
    case 1:
      if (last_state != 1) {
        printf("Entering standby mode.\n");
        last_state = 1;
        if (last_state == 0) {
          controller.elevation_ref = estimator.elevation;
          controller.elevation_dot_ref = estimator.elevation_dot;
        }
      }
      controller.elevation_sp = RadOfDeg(-28);
      controller.tilt_sp = 0;
      control_run();
      break;
    case 2:
      if (last_state == 0) {
        printf("Not allowed. Enter standby first.\n");
        controller.armed = 0;
      } else {
        if (last_state == 1) {
          printf("Entering flight mode.\n");
        }
        //controller.elevation_sp = ((int32_t)(foo/2048.)%2) ? RadOfDeg(-20) : RadOfDeg(10);
        control_run();
        last_state = 2;
      }
      break;
    default:
      break;
  }
}

static void main_exit(int sig)
{
  printf("Initiating BETH shutdown...\n");

//since the periodic event is no longer running when we get here right now,
//this code doesn't do anything so removed for now.
#if 0
  printf("Zeroing setpoints...");
  uint32_t startfoo = foo;
  while (foo < (startfoo + 2000)) {
    controller.tilt_sp = 0;
    controller.elevation_sp = RadOfDeg(-25);
  }
  printf("done\n");
#endif

  //If a logfile is being used, close it.
  //file_logger_exit()
  printf("Main Overo Application Exiting...\n");
  exit(EXIT_SUCCESS);
}



static void main_talk_with_stm32()
{

  //static int8_t adder = 1;
  //uint8_t *fooptr;

  //msg_out.payload.msg_down.thrust = 0;
  msg_out.payload.msg_down.thrust = (int8_t)controller.cmd_thrust;

  //TODO: change motor config to remove minus here.
  msg_out.payload.msg_down.pitch = -(int8_t)controller.cmd_pitch;
  //msg_out.payload.msg_down.pitch = 0;


  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame) , &msg_in, &spi_crc_ok);

  /* for debugging overo/stm spi link
  if (msg_in.payload.msg_up.bench_sensor.x == 0){
    fooptr = &msg_in;
    for (int i=0; i<sizeof(msg_in); i++)
      printf("%02x ",  fooptr[i]);
    printf("\n");
  }*/

  foo++;

  imu.gyro_unscaled.p = (msg_in.payload.msg_up.gyro.p & 0xFFFF);
  imu.gyro_unscaled.q = (msg_in.payload.msg_up.gyro.q & 0xFFFF);
  imu.gyro_unscaled.r = (msg_in.payload.msg_up.gyro.r & 0xFFFF);
  imu.accel_unscaled.x = (msg_in.payload.msg_up.accel.x & 0xFFFF);
  imu.accel_unscaled.y = (msg_in.payload.msg_up.accel.y & 0xFFFF);
  imu.accel_unscaled.z = (msg_in.payload.msg_up.accel.z & 0xFFFF);

}




