/*
 * Copyright (C) 2016 - IMAV 2016
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file modules/computer_vision/flight_plan_guided.c
 *
 * Functions to use directly in a flight plan during an AP_GUIDED flight.
 *
 * @author MAVLab IMAV 2016
 * @author K. N. McGuire
 *
 */

#include "modules/flight_plan_guided/flight_plan_guided.h"
#include "subsystems/ins.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>
#include "math/pprz_algebra_float.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "mcu_periph/uart.h"

// start and stop modules
#include "generated/modules.h"

#define NOM_FLIGHT_ALT 1.7  // nominal flight altitude
float nom_flight_alt; // nominal flight altitude


// TODO: add abi messages for variables from other modules, like stop triggers for
//   obstacle avoidance ?

//#include "subsystems/datalink/telemetry.h"
//#include "subsystems/abi.h"


// Module functions
void flight_plan_guided_init(void)
{
  nom_flight_alt = NOM_FLIGHT_ALT;
}


/********************************************//**
 *  Start functions for guided flight plan
 ***********************************************/

/**
 * KillEngines: Kill engines from flight plan directly
 * @param[return] Falseboolean for flight plan to only run once
 */
uint8_t KillEngines(void)
{
  autopilot_set_motors_on(FALSE);
  return false;
}

/**
 * StartEngines: Start engines from flight plan directly
 * @param[return] False boolean for flight plan to only run once
 */
uint8_t StartEngines(void)
{
  autopilot_set_motors_on(TRUE);
  return false;
}

/**
 * ResetAlt: Reset the altitude reference to the current GPS alt if GPS is used
 * @param[return] False boolean for flight plan to only run once
 */
uint8_t ResetAlt(void) {
	ins_reset_altitude_ref();
	return false;
}

/**
 * ChangeHorizontalMode: Change horizontal mode in autopilot guided mode
 * @param[in] mode, number of the specific horizontal mode that you want to change to
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
bool ChangeHorizontalMode(uint8_t mode)
{
  guidance_h_mode_changed(mode);
  return false;
}

/**
 * ChangeVerticalMode: Change vertical mode in autopilot guided mode
 * @param[in] mode, number of the specific horizontal mode that you want to change to
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
bool ChangeVerticalMode(uint8_t mode)
{
  guidance_v_mode_changed(mode);
  return false;
}


/********************************************//**
 *  Guided commands during flight
 ***********************************************/

/**
 * TakeOff: Take off with a certain climb rate
 * @param[in] climb_rate, climb_rate in [m/s] (positive z-axis is up)
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 *
 */
bool TakeOff(float climb_rate)
{
	// Waits untill autopilot mode is AP_MODE_GUIDED before taking off
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  guidance_v_set_guided_vz(-climb_rate);
  guidance_h_set_guided_body_vel(0, 0);

  return false;
}

/**
 * Hover: MAV will hover on spot in guided mode
 * @param[in] alt, altitude at where the hover should be
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
uint8_t Hover(float alt)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }
  // Horizontal velocities are set to zero and heading should be fixed
  guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
  guidance_h_set_guided_body_vel(0, 0);
  guidance_v_set_guided_z(-alt);

  return false;
}

/**
 * MoveForward: Move forward in body fixed coordinates, trying to keep sideways drift at 0
 * @param[in] vx, Forward velocity in [m/s]
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
uint8_t MoveForward(float vx)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (autopilot.mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(vx, 0);
  }
  return false;
}

/**
 * MoveSideways: Move sideways in body fixed coordinates, trying to keep sideways drift at 0
 * @param[in] vy, Sideways velocity in [m/s] (to the right is positive)
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */uint8_t MoveSideways(float vy)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (autopilot.mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(0, vy);
  }
  return false;
}


 /********************************************//**
  *  Rotating functions
  ***********************************************/

 /**
  * RotateToHeading: Rotate to a given heading in guided mode
  * @param[in] heading, rotate to the wanted heading in [rad] (clockwise is positive)
  * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
  */
 bool RotateToHeading(float heading)
 {
   if (autopilot.mode != AP_MODE_GUIDED) { return true; }

   guidance_h_set_guided_heading(heading);
   return false;
 }

 /**
  * SpeedRotateToHeading: Will slowly turn to a given heading with a fixed speed
  * @param[in] roation_speed, speed to slowly rotate the heading(clockwise is positive)
  * @param[in] heading, here the MAV will stop spinning [rad] (clockwise is positive)
  *  @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
  */
 bool SpeedRotateToHeading(float rotation_speed, float heading)
 {
   guidance_h_set_guided_heading_rate(rotation_speed);
   if (fabs(heading - stateGetNedToBodyEulers_f()->psi) < 0.1) {
     guidance_h_set_guided_heading(heading);
     return false;
   }
   return true;
 }

 /*NOTE: THe following rotating in ATT mode functions are necessary when no exact position (GPS or Optitrack) information is available
  * From experience, the guided rotating functions are very unstable and will cause the drone to drift significantly during the turn
  *   which can compromise the flight plane's mission. Hopefully in the future when the guided control in yaw has improved, these
  *   will not be necessary anymore
  */

 /**
  * RotateToHeading_ATT: Rotate to heading in ATT mode
  * @param[in] heading, to turn to in a guided flight [rad] (clockwise is positive)
  * @param[in] trim_phi, during turn, apply a small trim to the roll if sideways drift is noticeable [rad]
  * @param[in] trim_theta, during turn, apply a small trim to the pitch if forward/backwards drift is noticeable
  * @param[in] in_flight, bool necessary for stabilization loop to check if the MAV is flying
  * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
  */
 bool RotateToHeading_ATT(float heading, float trim_phi, float trim_theta, bool in_flight)
 {
   struct Int32Eulers cmd;

   if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
     cmd.phi = ANGLE_BFP_OF_REAL(trim_phi); //trim?
     cmd.theta = ANGLE_BFP_OF_REAL(trim_theta);
     cmd.psi = ANGLE_BFP_OF_REAL(heading);

     stabilization_attitude_set_rpy_setpoint_i(&cmd);
     stabilization_attitude_run(in_flight);
   }
   return false;
 }

 /**
  * ResetAngles_ATT: Reset all angles while in ATT mode
  * @param[in] current_heading, indicated the current heading of the MAV [rad] (clockwise is positive)
  * @param[in] in_flight, bool necessary for stabilization loop to check if the MAV is flying
  * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
  */
 bool ResetAngles_ATT(float current_heading, bool in_flight)
 {
   struct Int32Eulers cmd;
   if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
     cmd.phi = ANGLE_BFP_OF_REAL(0.0f);
     cmd.theta = ANGLE_BFP_OF_REAL(0.0f);
     cmd.psi = ANGLE_BFP_OF_REAL(current_heading);

     stabilization_attitude_set_rpy_setpoint_i(&cmd);
     stabilization_attitude_run(in_flight);
   }
   return false;
 }

/********************************************//**
 *  Blocking conditions
 ***********************************************/

 /**
  * WaitUntilAltitude: Blocks while waiting until a certain altitude is reached
  * @param[in] altitude in [m] (positive z-axis is up)
  * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
  */
 bool WaitUntilAltitude(float altitude)
 {
   if (autopilot.mode != AP_MODE_GUIDED) { return true; }

   if (stateGetPositionEnu_f()->z < -altitude) { return true; }

   return false;
 }

 /**
  * WaitUntilSpeedOrAltitude: Blocks while waiting until a certain altitude or speed is reached
  * @param[in] fail_altitude, wait for altitude in [m] (positive z-axis is up)
  * @param[in] speed, speed in [m/s] (positive z-axis is up)
  * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
  */
 bool WaitUntilSpeedOrAltitude(float speed, float fail_altitude)
 {
   if (autopilot.mode != AP_MODE_GUIDED) { return true; }

   if (stateGetPositionEnu_f()->z > -fail_altitude) { return false; }
   if (stateGetSpeedEnu_f()->z < -speed) { return true; }

   return false;
 }

 /**
  * WaitforHeading: Blocks untill wanted heading is achieved
  * @param[in] heading, waits untill MAV reached certain heading [rad] (clockwise is positive)
  *  @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
  *
  * TODO: Wrap the angle around -pi and pi
  */
 bool WaitforHeading(float heading)
 {
   if (fabs(heading - stateGetNedToBodyEulers_f()->psi) < 0.1) {
     return false;
   }
   return true;
 }

/**
 * WaitForHMode: Blocks until specified horizontal mode is achieved
 * @param[in] mode, number of the specific horizontal mode that you want to wait for
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
bool WaitForHMode(uint8_t mode)
{
  if (guidance_h.mode == mode) {
    return false;
  }
  return true;
}


/********************************************//**
 *  Timer functions
 ***********************************************/

/**
 * ResetSpecialTimer: Reset a special timer, that works outside of the block and stage time
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
float specialtimer = 0;
bool ResetSpecialTimer(void)
{
  specialtimer = 0;
  return false;
}

/**
 * WaitUntilTimer: Blocks while waiting until until the special timer reaches the specified amount of seconds
 * @param[in] sec, wait for amount of seconds [sec]
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
bool WaitUntilTimer(float sec)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  specialtimer += 1.0f / ((float)NAV_FREQ);
  if (specialtimer < sec) { return true; }

  return false;
}

/**
 * WaitUntilTimerOrAltitude: Blocks while waiting until the timer reaches the specified amount of seconds or an altitude has been reached
 * @param[in] sec, wait for amount of seconds [sec]
 * @param[in] fail_altitude, wait for altitude in [m] (positive z-axis is up)
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
bool WaitUntilTimerOrAltitude(float sec, float fail_altitude)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z > fail_altitude) { return false; }
  specialtimer += 1.0f / ((float)NAV_FREQ);
  if (specialtimer < sec) { return true; }

  return false;
}

/**
 * ResetCounter: Resets a special counter
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
int32_t counter = 0;
bool ResetCounter(void)
{
  counter = 0;
  return false;
}

/**
 * WaitUntilCounter: Blocks while waiting until until the special counter reaches the specified amount
 * @param[in] end_counter, condition for the counter to stop
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
bool WaitUntilCounter(int32_t end_counter)
{
	  if (autopilot.mode != AP_MODE_GUIDED) { return true; }
  counter++;
  if (counter > end_counter) {
    return false;
  }

  return true;
}
