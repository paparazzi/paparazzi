/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 The Paparazzi Team
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
 *  @file nps_radio_control_joystick.c
 *  Direct RC control for NPS with a standard joystick using SDL.
 *
 *  Simple DirectMedia Layer library is used for cross-platform support.
 *  Joystick button and axes are mapped to RC commands directly with defines.
 *
 *  You must have a joystick with either:
 *    - 4 axes and 3 buttons, or
 *    - 5 axes
 *
 *  First you should run sw/ground_segment/joystick/test_stick to determine
 *  the indices of the physical axes and buttons on your joystick (and to test
 *  that it actually works). Then you can assign each axis and button to a
 *  command in your airframe file.
 *
 *  The default axes are Roll = 0, Pitch = 1, Yaw = 2 and Throttle = 3.
 *  The default buttons are Manual = 0, Auto1 = 1, Auto2 = 2.
 *
 *  Example for 4 axes and 3 buttons using a joystick with 7 axes and 5 buttons:
 *  @verbatim
 *    <section name="SIMULATOR" prefix="NPS_">
 *      ...
 *      <define name="JS_AXIS_ROLL" value="2"/> <!-- Use joystick axis 2 for roll -->
 *      <!-- Use joystick axis 1 for pitch (default) -->
 *      <define name="JS_AXIS_YAW" value="3"/> <!-- Use joystick axis 3 for yaw -->
 *      <define name="JS_AXIS_ROLL" value="6"/> <!-- Use joystick axis 6 for throttle -->
 *      <define name="JS_BUTTON_MODE_MANUAL" value="3"/> <!-- Use joystick button 3 for manual -->
 *      <!-- Use joystick button 1 for auto1 (default) -->
 *      <!-- Use joystick button 2 for auto2 (default) -->
 *    </section>
 *  @endverbatim
 *
 *  One can define NPS_JS_AXIS_MODE to use an axis instead of buttons to change
 *  You will need a 5-axis joystick.
 *
 *  If you need to reverse the direction of any axis, simply use:
 *  @verbatim
 *      <define name="JS_AXIS_PITCH_REVERSE" value="1"/>
 *      <!-- value="1" is required, setting to zero or omitting disables reversing -->
 *  @endverbatim
 *
 *  At this point, no other functionality or channels are supported for R/C control.
 *
 */

#include "nps_radio_control.h"
#include "nps_radio_control_joystick.h"

// for NPS_JS_AXIS_MODE
#include "generated/airframe.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <SDL/SDL.h>

// axes indice
#ifndef NPS_JS_AXIS_ROLL
#define NPS_JS_AXIS_ROLL     0
#endif
#ifndef NPS_JS_AXIS_PITCH
#define NPS_JS_AXIS_PITCH    1
#endif
#ifndef NPS_JS_AXIS_YAW
#define NPS_JS_AXIS_YAW      2
#endif
#ifndef NPS_JS_AXIS_THROTTLE
#define NPS_JS_AXIS_THROTTLE 3
#endif

#ifndef NPS_JS_AXIS_MODE
#define JS_NB_AXIS  4
#else
#define JS_NB_AXIS 5
#endif

// buttons to switch modes
#ifndef NPS_JS_BUTTON_MODE_MANUAL
#define NPS_JS_BUTTON_MODE_MANUAL 1
#endif
#ifndef NPS_JS_BUTTON_MODE_AUTO1
#define NPS_JS_BUTTON_MODE_AUTO1  2
#endif
#ifndef NPS_JS_BUTTON_MODE_AUTO2
#define NPS_JS_BUTTON_MODE_AUTO2  3
#endif

#ifndef NPS_JS_AXIS_MODE
#define JS_NB_BUTTONS  3
#else
#define JS_NB_BUTTONS 0
#endif

struct NpsJoystick nps_joystick;
SDL_Joystick *sdl_joystick;
SDL_Event sdl_event;

/**
 *  Initializes SDL and the joystick.
 *
 *  Function exits with -1 if fails
 *
 *  @param device_index  string integer of desired joystick device
 *
 *  @returns  0 on success
 */
int nps_radio_control_joystick_init(const char *device)
{

  nps_joystick.throttle = 0.5;
  nps_joystick.roll = 0.;
  nps_joystick.pitch = 0.;
  nps_joystick.yaw = 0.;
  nps_joystick.mode = MODE_SWITCH_AUTO2;

  //Convert device index string to integer
  int device_index = atoi(device);

  //Initialize SDL with joystick support and event support (through video)
  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_VIDEO) < 0) {
    printf("Could not initialize SDL: %s.\n", SDL_GetError());
    exit(-1);
  }

  //Quit SDL at exit
  atexit(SDL_Quit);

  //Start the event handler, disable all but joystick button events and quit handler
  SDL_EventState(SDL_ACTIVEEVENT, SDL_IGNORE);
  SDL_EventState(SDL_KEYDOWN, SDL_IGNORE);
  SDL_EventState(SDL_KEYUP, SDL_IGNORE);
  SDL_EventState(SDL_MOUSEMOTION, SDL_IGNORE);
  SDL_EventState(SDL_MOUSEBUTTONDOWN, SDL_IGNORE);
  SDL_EventState(SDL_MOUSEBUTTONUP, SDL_IGNORE);
  SDL_EventState(SDL_JOYAXISMOTION, SDL_IGNORE);
  SDL_EventState(SDL_JOYBALLMOTION, SDL_IGNORE);
  SDL_EventState(SDL_JOYHATMOTION, SDL_IGNORE);
  //SDL_EventState(SDL_JOYBUTTONDOWN,SDL_IGNORE);
  SDL_EventState(SDL_JOYBUTTONUP, SDL_IGNORE);
  SDL_EventState(SDL_VIDEORESIZE, SDL_IGNORE);
  SDL_EventState(SDL_VIDEOEXPOSE, SDL_IGNORE);
  //SDL_EventState(SDL_QUIT,SDL_IGNORE);
  SDL_EventState(SDL_USEREVENT, SDL_IGNORE);
  SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);

  //Check there are actually joysticks attached
  if (!SDL_NumJoysticks()) {
    printf("No joysticks attached! Quitting.\n");
    exit(-1);
  }

  //Open the desired joystick and make sure it will work
  sdl_joystick = SDL_JoystickOpen(device_index);
  if (!sdl_joystick) {
    printf("Joystick corresponding to SDL Index %d failed to open! Exiting.\n", device_index);
    exit(-1);
  } else if (SDL_JoystickNumAxes(sdl_joystick) < JS_NB_AXIS) {
    printf("Selected joystick does not support enough axes!\n");
    printf("Number of axes required: %i\n", JS_NB_AXIS);
    printf("Number of axes available: %i\n", SDL_JoystickNumAxes(sdl_joystick));
    SDL_JoystickClose(sdl_joystick);
    exit(-1);
  } else if (SDL_JoystickNumButtons(sdl_joystick) < JS_NB_BUTTONS) {
    printf("Selected joystick does not support enough buttons!\n");
    printf("Buttons supported: %d  needed: %d\n", SDL_JoystickNumButtons(sdl_joystick), JS_NB_BUTTONS);
    SDL_JoystickClose(sdl_joystick);
    exit(-1);
  } else {
    printf("Using joystick named: %s\n", SDL_JoystickName(device_index));
  }

  return 0;
}

/**
 *  Updates joystick buttons from events, directly reads current axis positions..
 */
void nps_radio_control_joystick_update(void)
{

#if NPS_JS_AXIS_THROTTLE_REVERSED
  nps_joystick.throttle = ((float)(-1 * SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_THROTTLE)) - 32767.) / -65534.;
#else
  nps_joystick.throttle = ((float)(SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_THROTTLE)) - 32767.) / -65534.;
#endif
#if NPS_JS_AXIS_ROLL_REVERSED
  nps_joystick.roll = (float)(-1 * SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_ROLL)) / 32767.;
#else
  nps_joystick.roll = (float)(SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_ROLL)) / 32767.;
#endif
#if NPS_JS_AXIS_PITCH_REVERSED
  nps_joystick.pitch = (float)(-1 * SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_PITCH)) / 32767.;
#else
  nps_joystick.pitch = (float)(SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_PITCH)) / 32767.;
#endif
#if NPS_JS_AXIS_YAW_REVERSED
  nps_joystick.yaw = (float)(-1 * SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_YAW)) / 32767.;
#else
  nps_joystick.yaw = (float)(SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_YAW)) / 32767.;
#endif
  // if an axis is asigned to the mode, use it instead of the buttons
#ifdef NPS_JS_AXIS_MODE
#if NPS_JS_AXIS_MODE_REVERSED
  nps_joystick.mode = (float)(-1 * SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_MODE)) / 32767.;
#else
  nps_joystick.mode = (float)(SDL_JoystickGetAxis(sdl_joystick, NPS_JS_AXIS_MODE)) / 32767.;
#endif
#endif

  while (SDL_PollEvent(&sdl_event)) {
    switch (sdl_event.type) {
      case SDL_JOYBUTTONDOWN: {
        switch (sdl_event.jbutton.button) {
#ifndef NPS_JS_AXIS_MODE
          case NPS_JS_BUTTON_MODE_MANUAL:
            nps_joystick.mode = MODE_SWITCH_MANUAL;
            break;

          case NPS_JS_BUTTON_MODE_AUTO1:
            nps_joystick.mode = MODE_SWITCH_AUTO1;
            break;

          case NPS_JS_BUTTON_MODE_AUTO2:
            nps_joystick.mode = MODE_SWITCH_AUTO2;
            break;
#endif
          default:
            //ignore
            break;
        }
      }
      break;

      case SDL_QUIT:
        printf("Quitting...\n");
        exit(-1);
        break;

      default:
        //do nothing
        printf("unknown SDL event!!!\n");
        break;
    }
  }
}
