/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include "nps_radio_control.h"

#include "nps_radio_control_spektrum.h"
#include "nps_radio_control_joystick.h"

#include <stdio.h>

#define RADIO_CONTROL_DT (1./40.)

struct NpsRadioControl nps_radio_control;



void nps_radio_control_init(enum NpsRadioControlType type, int num_script, char *js_dev)
{

  nps_radio_control.next_update = 0.;
  nps_radio_control.type = type;
  nps_radio_control.num_script = num_script;

  switch (type) {
    case JOYSTICK:
      nps_radio_control_joystick_init(js_dev);
      break;
    case SPEKTRUM:
      nps_radio_control_spektrum_init(js_dev);
      break;
    case SCRIPT:
      break;
  }

}


typedef void (*rc_script)(double);

static void radio_control_script_takeoff(double time);
static void radio_control_script_hover(double time);
static void radio_control_script_step_roll(double time);
static void radio_control_script_step_pitch(double time);
static void radio_control_script_step_yaw(double time);
static void radio_control_script_ff(double time);

static rc_script scripts[] = {
  radio_control_script_hover,
  radio_control_script_step_roll,
  radio_control_script_step_pitch,
  radio_control_script_step_yaw,
  radio_control_script_ff
};


#define RADIO_CONTROL_TAKEOFF_TIME 8


bool_t nps_radio_control_available(double time)
{
  if (time >=  nps_radio_control.next_update) {
    nps_radio_control.next_update += RADIO_CONTROL_DT;

    if (nps_radio_control.type == JOYSTICK) {
      nps_radio_control_joystick_update();
      nps_radio_control.throttle = nps_joystick.throttle;
      nps_radio_control.roll = nps_joystick.roll;
      nps_radio_control.pitch = nps_joystick.pitch;
      nps_radio_control.yaw = nps_joystick.yaw;
      nps_radio_control.mode = nps_joystick.mode;
      //printf("throttle: %f, roll: %f, pitch: %f, yaw: %f\n", nps_joystick.throttle, nps_joystick.roll, nps_joystick.pitch, nps_joystick.yaw);
    } else if (nps_radio_control.type == SCRIPT) {
      if (time < RADIO_CONTROL_TAKEOFF_TIME) {
        radio_control_script_takeoff(time);
      } else {
        scripts[nps_radio_control.num_script](time);
      }
    }
    return TRUE;
  }
  return FALSE;
}




/*
 * Scripts
 *
 *
 */

void radio_control_script_takeoff(double time)
{
  nps_radio_control.roll = 0.;
  nps_radio_control.pitch = 0.;
  nps_radio_control.yaw = 0.;
  nps_radio_control.throttle = 0.;
  nps_radio_control.mode = MODE_SWITCH_MANUAL;
  /* starts motors */
  if (time < 1.) {
    nps_radio_control.yaw = 1.;
  } else {
    nps_radio_control.yaw = 0.;
  }

}

void radio_control_script_hover(double time __attribute__((unused)))
{
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  nps_radio_control.roll = 0.;
  nps_radio_control.yaw = 0.;
}


void radio_control_script_step_roll(double time)
{
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;

  if (((int32_t)rint((time * 0.5))) % 2) {
    nps_radio_control.roll = 0.2;
    nps_radio_control.yaw = 0.5;
  } else {
    nps_radio_control.roll = -0.2;
    nps_radio_control.yaw = 0.;
  }
}

void radio_control_script_step_pitch(double time)
{
  nps_radio_control.roll = 0.;
  nps_radio_control.yaw = 0.;
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  if (((int32_t)rint((time * 0.5))) % 2) {
    nps_radio_control.pitch = 0.2;
  } else {
    nps_radio_control.pitch = -0.2;
  }
}

void radio_control_script_step_yaw(double time)
{
  nps_radio_control.roll = 0.;
  nps_radio_control.pitch = 0.;
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;

  if (((int32_t)rint((time * 0.5))) % 2) {
    nps_radio_control.yaw = 0.5;
  } else {
    nps_radio_control.yaw = -0.5;
  }
}

void radio_control_script_ff(double time __attribute__((unused)))
{
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  if (time < RADIO_CONTROL_TAKEOFF_TIME + 3) {
    nps_radio_control.pitch = -1.;
  } else if (time < RADIO_CONTROL_TAKEOFF_TIME + 6) {
    //    nps_radio_control.roll = 0.5;
    nps_radio_control.pitch = -1.;
  } else {
    nps_radio_control.pitch = 0.;
    nps_radio_control.roll = 0.;
  }
}
