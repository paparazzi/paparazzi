/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"

#define NPS_COMMANDS_NB 4
struct gazebo_debug_t
{
  float sp[NPS_COMMANDS_NB];
  float u[NPS_COMMANDS_NB];
  float udot[NPS_COMMANDS_NB];
  float spinup_out[NPS_COMMANDS_NB];
  float thrust_out[NPS_COMMANDS_NB];
  float torque_out[NPS_COMMANDS_NB];
};
extern struct gazebo_debug_t gazebo_debug;

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,
      "counter,gyro_p,gyro_q,gyro_r,accel_x,accel_y,accel_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz,sp0,sp1,sp2,sp3,u0,u1,u2,u3,udot0,udot1,udot2,udot3,spinup0,spinup1,spinup2,spinup3,thr0,thr1,thr2,thr3,T0,T1,T2,T3\n"
    );
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();

fprintf(file_logger,
    "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
          counter,
    imu.gyro.p,
    imu.gyro.q,
    imu.gyro.r,
    imu.accel.x,
    imu.accel.y,
    imu.accel.z,
    imu.mag_unscaled.x,
    imu.mag_unscaled.y,
    imu.mag_unscaled.z,
    stabilization_cmd[COMMAND_THRUST],
    stabilization_cmd[COMMAND_ROLL],
    stabilization_cmd[COMMAND_PITCH],
    stabilization_cmd[COMMAND_YAW],
    quat->qi,
    quat->qx,
    quat->qy,
    quat->qz,
    gazebo_debug.sp[0],
    gazebo_debug.sp[1],
    gazebo_debug.sp[2],
    gazebo_debug.sp[3],
    gazebo_debug.u[0],
    gazebo_debug.u[1],
    gazebo_debug.u[2],
    gazebo_debug.u[3],
    gazebo_debug.udot[0],
    gazebo_debug.udot[1],
    gazebo_debug.udot[2],
    gazebo_debug.udot[3],
    gazebo_debug.spinup_out[0],
    gazebo_debug.spinup_out[1],
    gazebo_debug.spinup_out[2],
    gazebo_debug.spinup_out[3],
    gazebo_debug.thrust_out[0],
    gazebo_debug.thrust_out[1],
    gazebo_debug.thrust_out[2],
    gazebo_debug.thrust_out[3],
    gazebo_debug.torque_out[0],
    gazebo_debug.torque_out[1],
    gazebo_debug.torque_out[2],
    gazebo_debug.torque_out[3]
    );
  counter++;
}
