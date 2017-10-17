/*
 * Copyright (C) K. N. McGuire
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * @file "modules/range_forcefield/range_forcefield.h"
 * @author K. N. McGuire
 * This module generates a forcefield based on range sensor measurements the use of single point range sensors.
 */


#include "modules/range_forcefield/range_forcefield.h"
#include "subsystems/abi.h"


float inner_border_FF;
float outer_border_FF;
float min_vel_command;
float max_vel_command;
float vel_body_x_guided;
float vel_body_y_guided;

//abi for range sensors
#ifndef RANGE_MODULE_RECIEVE_ID
#define RANGE_MODULE_RECIEVE_ID ABI_BROADCAST
#endif


static abi_event range_sensor_array_ev;
static void range_sensor_array_cb(uint8_t sender_id, int16_t range_array_size, uint16_t *range_array, int32_t *orientation_array);
static void range_sensor_array_cb(uint8_t sender_id  __attribute__((unused)), int16_t range_array_size, uint16_t *range_array, int32_t *orientation_array)
{

  struct FloatEulers body_to_sensors_eulers = {0, 0, 0};
  struct FloatVect3 vel_avoid_body = {0, 0, 0};
  float range = 0;

  // loop through the range sensor array
  for (int i = 0; i < range_array_size; i++) {
    // Get the orientation  and value of the range array measurement (per sensor)
    body_to_sensors_eulers.phi = ANGLE_FLOAT_OF_BFP(orientation_array[3 * i]);
    body_to_sensors_eulers.theta = ANGLE_FLOAT_OF_BFP(orientation_array[3 * i + 1]);
    body_to_sensors_eulers.psi = ANGLE_FLOAT_OF_BFP(orientation_array[3 * i + 2]);
    range = (float)range_array[i] / 1000.;

    // Calculate forcefield for that one measurement
    range_sensor_single_velocity_force_field(&vel_avoid_body,  range, &body_to_sensors_eulers,
        inner_border_FF,  outer_border_FF,  min_vel_command,  max_vel_command);
  }

  //Send the range velocity forcefield through abi
  AbiSendMsgRANGE_FORCEFIELD(RANGE_FORCEFIELD_ID, vel_avoid_body.x, vel_avoid_body.y, vel_avoid_body.z);
}


void range_forcefield_init(void)
{
  inner_border_FF = 1.0f;
  outer_border_FF = 1.4f;
  min_vel_command = 0.0f;
  max_vel_command = 0.5f;
  vel_body_x_guided = 0.0f;
  vel_body_y_guided = 0.0f;

  AbiBindMsgRANGE_SENSORS_ARRAY(RANGE_MODULE_RECIEVE_ID, &range_sensor_array_ev, range_sensor_array_cb);
}

/*  range_sensor_single_velocity_force_field: This function adjusts the intended velocity commands in the horizontal plane
 * to move away from obstacles and walls as detected by single ray range sensors. An simple linear equation with the distance
 * measured by the range sensors and the given minimum and maximum avoid velocity, it will create a velocity forcefield which
 * can be used during a guided flight.
 *
 *
 * @param[in] vel_body_x, intended body fixed velocity in the x direction in [m/s]
 * @param[in] vel_body_y, intended body fixed velocity in the y direction in [m/s]
 * @param[in] body_to_sensors_eulers, euler angles of the orientation of the range sensor in question[rad]
 * @param[in] avoid_inner_border, minimum allowable distance from the obstacle in [m]
 * @param[in] avoid_outer_border, maximum allowable distance from the obstacle in [m]
 * @param[in] min_vel_command_lc, minimum velocity for the forcefield generation [m/s]
 * @param[in] max_vel_command_lc, maximum velocity for the forcefield generation [m/s]
 * @param[out] vel_body_x, adjusted body velocity in x direction [m/s]
 * @param[out] vel_body_y, adjusted body velocity in y direction [m/s]
 * */

void range_sensor_single_velocity_force_field(struct FloatVect3 *vel_avoid_body, float range, struct FloatEulers *body_to_sensors_eulers,
    float avoid_inner_border, float avoid_outer_border, float min_vel_command_lc, float max_vel_command_lc)
{
  static const float max_sensor_range = 2;
  float difference_inner_outer = avoid_outer_border - avoid_inner_border;

  // Velocity commands
  float avoid_command = 0;

  // Calculate avoidance velocity
  if (range < 0.001  || range > max_sensor_range) {
    //do nothing
  } else if (range < avoid_inner_border) {
    avoid_command -= max_vel_command_lc;
  } else if (range < avoid_outer_border) {
    // Linear
    avoid_command -= (max_vel_command_lc - min_vel_command_lc) *
                     (range - range)
                     / difference_inner_outer;
  }

  struct FloatRMat range_sensor_to_body;
  float_rmat_of_eulers(&range_sensor_to_body, body_to_sensors_eulers);
  struct FloatVect3 quad_sensor_avoid_vel = {avoid_command, 0, 0};


  struct FloatVect3 quad_body_avoid_vel;
  float_rmat_transp_vmult(&quad_body_avoid_vel, &range_sensor_to_body, &quad_sensor_avoid_vel);

  // Add velocity forcefield to previous values (for balancing the command)
  vel_avoid_body->x += quad_body_avoid_vel.x;
  vel_avoid_body->y += quad_body_avoid_vel.y;
  vel_avoid_body->z += quad_body_avoid_vel.z;

}


