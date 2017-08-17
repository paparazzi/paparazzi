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
 * @file modules/computer_vision/flight_plan_guided.h
 *
 * Functions to use directly in a flight plan during an AP_GUIDED flight.
 *
 * @author MAVLab IMAV 2016
 * @author K. N. McGuire
 */

#ifndef FLIGHT_PLAN_GUIDED_PLUGIN_H
#define FLIGHT_PLAN_GUIDED_PLUGIN_H

#include <std.h>

extern float nominal_alt;

/*TODO: More functions to add in the future (feel free to add more)
 *
 * * Land function
 * * Diagonal movement function
 * * Wait for vertical mode
 */

// Module functions
void flight_plan_guided_init(void);

// Start functions for guided flight plan
extern uint8_t KillEngines(void);
extern uint8_t StartEngines(void);
extern uint8_t ResetAlt(void);

// Guided commands during flight
extern bool Climb(float climb_rate);
extern uint8_t Hover(float alt);
extern uint8_t MoveForward(float vx);
extern uint8_t MoveSideways(float vy);

// Rotating functions
extern bool RotateToHeading(float heading);
extern bool SpeedRotateToHeading(float rotation_speed, float heading);
extern bool RotateToHeading_ATT(float new_heading, float trim_phi, float trim_theta, bool in_flight);
extern bool ResetAngles_ATT(float current_heading, bool in_flight);

//Blocking conditions
extern bool WaitUntilAltitude(float altitude);
extern bool WaitUntilSpeedOrAltitude(float speed, float fail_altitude);
extern bool WaitforHeading(float heading);

//Timer functions conditions
extern bool ResetSpecialTimer(void);
extern bool WaitUntilTimer(float sec);
extern bool ResetCounter(void);
extern bool WaitUntilCounter(int32_t end_counter);
extern bool WaitUntilTimerOrAltitude(float sec, float fail_altitude);

#endif
