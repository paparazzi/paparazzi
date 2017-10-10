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
 * @file "modules/range_module/range_module.c"
 * @author K. N. McGuire
 * This module contains functions to accommodate the use of single point range sensors.
 */

#include "modules/range_sensor/range_sensor_forcefield.h"
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

static abi_event range_sensors_ev;
static void range_sensors_cb(uint8_t sender_id,
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left, int16_t range_bottom, int16_t range_top);
struct range_finders_ range_finders;
static void range_sensors_cb(uint8_t UNUSED(sender_id),
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left, int16_t range_bottom, int16_t range_top)
{
  range_finders.front = (float)range_front / 1000;
  range_finders.right = (float)range_right / 1000;
  range_finders.back = (float)range_back / 1000;
  range_finders.left = (float)range_left / 1000;
  range_finders.top = (float)range_top / 1000;
  range_finders.bottom = (float)range_bottom / 1000;


}

void range_init(void)
{
  inner_border_FF = 1.0f;
  outer_border_FF = 1.4f;
  min_vel_command = 0.0f;
  max_vel_command = 0.5f;
  vel_body_x_guided = 0.0f;
  vel_body_y_guided = 0.0f;

  AbiBindMsgRANGE_SENSORS(RANGE_MODULE_RECIEVE_ID, &range_sensors_ev, range_sensors_cb);

}

void range_run(void)
{
  float vel_body_x  = 0;
  float vel_body_y  = 0;
  float vel_body_z  = 0;

  range_sensor_horizontal_velocity_force_field(&vel_body_x, &vel_body_y,
      inner_border_FF, outer_border_FF, min_vel_command, max_vel_command);

  range_sensor_vertical_velocity_force_field(&vel_body_z,
      inner_border_FF, outer_border_FF, min_vel_command, max_vel_command);

  AbiSendMsgRANGE_FORCEFIELD(RANGE_FORCEFIELD_ID, vel_body_x, vel_body_y, vel_body_z);
}

/*  range_sensor_horizontal_force_field: This function adjusts the intended velocity commands in the horizontal plane
 * to move away from obstacles and walls as detected by single ray range sensors. An simple linear equation with the distance
 * measured by the range sensors and the given minimum and maximum avoid velocity, it will create a velocity forcefield which
 * can be used during a guided flight.
 *
 *
 * @param[in] vel_body_x, intended body fixed velocity in the x direction in [m/s]
 * @param[in] vel_body_y, intended body fixed velocity in the y direction in [m/s]
 * @param[in] avoid_inner_border, minimum allowable distance from the obstacle in [m]
 * @param[in] avoid_outer_border, maximum allowable distance from the obstacle in [m]
 * @param[in] min_vel_command_lc, minimum velocity for the forcefield generation [m/s]
 * @param[in] max_vel_command_lc, maximum velocity for the forcefield generation [m/s]
 * @param[out] vel_body_x, adjusted body velocity in x direction [m/s]
 * @param[out] vel_body_y, adjusted body velocity in y direction [m/s]
 * */

void range_sensor_horizontal_velocity_force_field(float *vel_body_x, float *vel_body_y, float avoid_inner_border, float avoid_outer_border,
     float min_vel_command_lc, float max_vel_command_lc)
{
  static const float max_sensor_range = 2;

  float difference_inner_outer = avoid_outer_border - avoid_inner_border;

  // Velocity commands
  float avoid_x_command = *vel_body_x;
  float avoid_y_command = *vel_body_y;

  // Balance avoidance command for y direction (sideways)
  if (range_finders.right < 0.001 || range_finders.right > max_sensor_range) {
    //do nothing
  } else if (range_finders.right < avoid_inner_border) {
    avoid_y_command -= max_vel_command_lc;
  } else if (range_finders.right < avoid_outer_border) {
    // Linear
    avoid_y_command -= (max_vel_command_lc - min_vel_command_lc) *
                       ((float)avoid_outer_border - (float)range_finders.right)
                       / (float)difference_inner_outer;
  } else {}

  if (range_finders.left < 0.001  || range_finders.left > max_sensor_range) {
    //do nothing
  } else if (range_finders.left < avoid_inner_border) {
    avoid_y_command += max_vel_command_lc;
  } else if (range_finders.left < avoid_outer_border) {
    // Linear
    avoid_y_command += (max_vel_command_lc - min_vel_command_lc) *
                       ((float)avoid_outer_border - (float)range_finders.left)
                       / (float)difference_inner_outer;
  } else {}

  // balance avoidance command for x direction (forward/backward)
  if (range_finders.front < 0.001  || range_finders.front > max_sensor_range) {
    //do nothing
  } else if (range_finders.front < avoid_inner_border) {
    avoid_x_command -= max_vel_command_lc;
  } else if (range_finders.front < avoid_outer_border) {
    // Linear
    avoid_x_command -= (max_vel_command_lc - min_vel_command_lc) *
                       ((float)avoid_outer_border - (float)range_finders.front)
                       / (float)difference_inner_outer;
  } else {}


  if (range_finders.back < 0.001  || range_finders.back > max_sensor_range) {
    //do nothing
  } else if (range_finders.back < avoid_inner_border) {
    avoid_x_command += max_vel_command_lc;
  } else if (range_finders.back < avoid_outer_border) {
    // Linear
    avoid_x_command += (max_vel_command_lc - min_vel_command_lc) *
                       ((float)avoid_outer_border - (float)range_finders.back)
                       / (float)difference_inner_outer;
  } else {}

  *vel_body_x = avoid_x_command;
  *vel_body_y = avoid_y_command;

}

/*  range_sensor_vertical_force_field: Same function as for horizontal plane (see range_sensor_vertical_force_field), but then for the top and bottom range sensor
 *
 * @param[in] vel_body_z, intended body fixed velocity in the z direction in [m/s]
 * @param[in] avoid_inner_border, minimum allowable distance from the obstacle in [m]
 * @param[in] avoid_outer_border, maximum allowable distance from the obstacle in [m]
 * @param[in] min_vel_command_lc, minimum velocity for the forcefield generation [m/s]
 * @param[in] max_vel_command_lc, maximum velocity for the forcefield generation [m/s]
 * @param[out] vel_body_x, adjusted body velocity in x direction [m/s]
 * @param[out] vel_body_y, adjusted body velocity in y direction [m/s]
 * */
void range_sensor_vertical_velocity_force_field(float *vel_body_z, float avoid_inner_border, float avoid_outer_border,
    float min_vel_command_lc, float max_vel_command_lc)
{
  static const float max_sensor_range = 2;

  float difference_inner_outer = avoid_outer_border - avoid_inner_border;

  // Velocity commands
  float avoid_z_command = *vel_body_z;

  // Balance avoidance command for y direction (sideways)
  if (range_finders.top < 0.001 || range_finders.top > max_sensor_range) {
    //do nothing
  } else if (range_finders.top < avoid_inner_border) {
    avoid_z_command -= max_vel_command_lc;
  } else if (range_finders.top < avoid_outer_border) {
    // Linear
    avoid_z_command -= (max_vel_command_lc - min_vel_command_lc) *
                       ((float)avoid_outer_border - (float)range_finders.top)
                       / (float)difference_inner_outer;
  } else {}

  if (range_finders.bottom < 0.001 || range_finders.left > max_sensor_range) {
    //do nothing
  } else if (range_finders.bottom < avoid_inner_border) {
    avoid_z_command += max_vel_command_lc;
  } else if (range_finders.bottom < avoid_outer_border) {
    // Linear
    avoid_z_command += (max_vel_command_lc - min_vel_command_lc) *
                       ((float)avoid_outer_border - (float)range_finders.bottom)
                       / (float)difference_inner_outer;
  } else {}


  *vel_body_z = avoid_z_command;
}

