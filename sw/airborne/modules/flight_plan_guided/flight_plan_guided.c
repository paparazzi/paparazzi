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
 * @author K. N. McGuire 2017
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

float nominal_alt; // nominal flight altitude

//TODO: Find equivelant function in math (couldn't find it yet...)

/* Wraps an angle in radians between -PI and +PI */
float wrapToPi_guided(float ang)
{
  if (ang > M_PI) {
    while (ang > M_PI) {
      ang = ang - 2 * M_PI;
    }
  } else if (ang < -M_PI) {
    while (ang < -M_PI) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

// Module functions
void flight_plan_guided_init(void)
{
  nominal_alt = NOMINAL_ALT;

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
uint8_t ResetAlt(void)
{
  ins_reset_altitude_ref();
  return false;
}


/********************************************//**
 *  Guided commands during flight
 ***********************************************/

/**
 * Climb: Climb with a certain climb rate
 * @param[in] climb_rate, climb_rate in [m/s] (positive z-axis is up)
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 *
 */
bool Climb(float climb_rate)
{
  // Waits untill autopilot mode is AP_MODE_GUIDED before taking off
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (climb_rate > 0) {
    guidance_v_set_guided_vz(-climb_rate);
    guidance_h_set_guided_body_vel(0, 0);
  } else { return true; }

  return false;
}

/**
 * Descent: Descent with a certain climb rate
 * @param[in] descent_rate, descent_rate in [m/s] (positive z-axis is down)
 * @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 *
 */
bool Descent(float descent_rate)
{
  // Waits untill autopilot mode is AP_MODE_GUIDED before taking off
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (descent_rate < 0) {
    guidance_v_set_guided_vz(descent_rate);
    guidance_h_set_guided_body_vel(0, 0);
  } else { return true; }

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



/********************************************//**
 *  Rotating functions
 ***********************************************/
float wanted_heading = 0;
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
 * @param[in] rotation_rate, speed to slowly rotate the heading(clockwise is positive)
 * @param[in] heading, here the MAV will stop spinning [rad] (clockwise is positive)
 *  @param[return] Boolean for flight plan to keep running (true) or to just run once (false)
 */
bool SpeedRotateToHeading(float rotation_rate, float heading)
{
  guidance_h_set_guided_heading_rate(rotation_rate);

  if (fabs(wrapToPi_guided(heading) - stateGetNedToBodyEulers_f()->psi) < 0.1) {
    guidance_h_set_guided_heading(heading);
    return false;
  }
  return true;
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
  if (fabs(wrapToPi_guided(heading) - stateGetNedToBodyEulers_f()->psi) < 0.1) {
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
