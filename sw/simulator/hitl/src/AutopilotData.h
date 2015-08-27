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
 * @file AutopilotData.h
 *
 * HITL demo version - class containing data sent from the autopilot
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef AUTOPILOTDATA_H_
#define AUTOPILOTDATA_H_

#include "MsgConfig.h"

/**
 * Autopilot Data
 */
class AutopilotData
{
public:
  uint32_t msg_cnt; // Pckg counter
  uint8_t actuators_nb; // NB actuators aka COMMANDS_NB
  int16_t actuators[ACTUATORS_NB] = {0}; // Actuators value, int16
  double commands[ACTUATORS_NB] = {0.0}; // scaled actuators for JSBSim

  float time_startup = 0; // [s] VN Timestamp
  float YawPitchRoll[3] = {0}; // [deg] attitude
  float angularRate[3] = {0}; // [rad/s] angular rate in body coordinates
  double position[3] = {0}; // [deg,deg,m] LLA
  float velocity_NED[3] = {0}; // [m/s] NED Velocity
  float accel[3] = {0}; // [m/s^2] accel in body
  uint32_t tow = 0; // [ms]
  uint8_t numSats = 0;
  uint8_t gpsFix = 0;
  float posU[3] = {0}; // [m] pos.unc.in NED frame
  float velU = 0; // [m/s] velocity unc.
  float linAccelBody[3] = {0}; // [m/s^2] Body acceleration without gravity
  float yprU[3] = {0}; //[deg]
  uint16_t insStatus = 0;
  float velBody[3] = {0}; // [m/s]
  float ap_time = 0; // [s]

  // COMMANDS
  int16_t cmd_throttle = 0;
  int16_t cmd_roll = 0;
  int16_t cmd_pitch = 0;
  int16_t cmd_yaw = 0;
  int16_t cmd_flaps = 0;

  // Normalized commands
  double cmd_throttle_norm = 0;
  double cmd_roll_norm = 0;
  double cmd_pitch_norm = 0;
  double cmd_yaw_norm = 0;
  double cmd_flaps_norm = 0;

  float alpha = 0; // [rad]
  float beta = 0; // [rad]
  float airspeed = 0; // [m/s]
  uint16_t ap_settings = 0;

  AutopilotData() {
    msg_cnt = 0;
    actuators_nb = ACTUATORS_NB;
  }
};

#endif /* AUTOPILOTDATA_H_ */
