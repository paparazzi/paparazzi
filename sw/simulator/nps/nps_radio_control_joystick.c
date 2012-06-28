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
 *  Currently it doesn't support different RC configurations.
 */

#include "nps_radio_control.h"
#include "nps_radio_control_joystick.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <SDL/SDL.h>

// axes indice
#define JS_ROLL     0
#define JS_PITCH    1
#define JS_YAW      2
#define JS_THROTTLE 3
#define JS_NB_AXIS  4

// buttons to switch modes
#define JS_MODE_MANUAL 4
#define JS_MODE_AUTO1  5
#define JS_MODE_AUTO2  6
#define JS_NB_BUTTONS  3

NpsJoystick nps_joystick;
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
int nps_radio_control_joystick_init(const char* device) {

  nps_joystick.throttle = 0.5;
  nps_joystick.roll = 0.;
  nps_joystick.pitch = 0.;
  nps_joystick.yaw = 0.;
  nps_joystick.mode = MODE_SWITCH_AUTO2;

  //Convert device index string to integer
  int device_index = atoi(device);

  //Initialize SDL with joystick support and event support (through video)
  if (SDL_Init(SDL_INIT_JOYSTICK|SDL_INIT_VIDEO) < 0)
  {
    printf("Could not initialize SDL: %s.\n", SDL_GetError());
    exit(-1);
  }

  //Quit SDL at exit
  atexit(SDL_Quit);

  //Start the event handler, disable all but joystick button events and quit handler
  SDL_EventState(SDL_ACTIVEEVENT,SDL_IGNORE);
  SDL_EventState(SDL_KEYDOWN,SDL_IGNORE);
  SDL_EventState(SDL_KEYUP,SDL_IGNORE);
  SDL_EventState(SDL_MOUSEMOTION,SDL_IGNORE);
  SDL_EventState(SDL_MOUSEBUTTONDOWN,SDL_IGNORE);
  SDL_EventState(SDL_MOUSEBUTTONUP,SDL_IGNORE);
  SDL_EventState(SDL_JOYAXISMOTION,SDL_IGNORE);
  SDL_EventState(SDL_JOYBALLMOTION,SDL_IGNORE);
  SDL_EventState(SDL_JOYHATMOTION,SDL_IGNORE);
  //SDL_EventState(SDL_JOYBUTTONDOWN,SDL_IGNORE);
  SDL_EventState(SDL_JOYBUTTONUP,SDL_IGNORE);
  SDL_EventState(SDL_VIDEORESIZE,SDL_IGNORE);
  SDL_EventState(SDL_VIDEOEXPOSE,SDL_IGNORE);
  //SDL_EventState(SDL_QUIT,SDL_IGNORE);
  SDL_EventState(SDL_USEREVENT,SDL_IGNORE);
  SDL_EventState(SDL_SYSWMEVENT,SDL_IGNORE);

  //Check there are actually joysticks attached
  if (!SDL_NumJoysticks())
  {
    printf("No joysticks attached! Quitting.\n");
    exit(-1);
  }

  //Open the desired joystick and make sure it will work
  sdl_joystick = SDL_JoystickOpen(device_index);
  if (!sdl_joystick)
  {
    printf("Joystick corresponding to SDL Index %d failed to open! Exiting.\n", device_index);
    exit(-1);
  }
  else if (SDL_JoystickNumAxes(sdl_joystick) < JS_NB_AXIS)
  {
    printf("Selected joystick does not support enough axes!\n");
    SDL_JoystickClose(sdl_joystick);
    exit(-1);
  }
  else if (SDL_JoystickNumButtons(sdl_joystick) < JS_NB_BUTTONS)
  {
    printf("Selected joystick does not support enough buttons!\n");
    SDL_JoystickClose(sdl_joystick);
    exit(-1);
  }
  else
  {
    printf("Using joystick named: %s\n",SDL_JoystickName(device_index));
  }

  return 0;
}

/**
 *  Updates joystick buttons from events, directly reads current axis positions..
 */
void nps_radio_control_joystick_update(void) {

  nps_joystick.throttle = ((float)(SDL_JoystickGetAxis(sdl_joystick,JS_THROTTLE)) - 32767.)/-65534.;
  nps_joystick.roll = (float)(SDL_JoystickGetAxis(sdl_joystick,JS_ROLL))/32767.;
  nps_joystick.pitch = (float)(SDL_JoystickGetAxis(sdl_joystick,JS_PITCH))/32767.;
  nps_joystick.yaw = (float)(SDL_JoystickGetAxis(sdl_joystick,JS_YAW))/32767.;

  while(SDL_PollEvent(&sdl_event))
  {
    switch(sdl_event.type)
    {
      case SDL_JOYBUTTONDOWN:
      {
	switch(sdl_event.jbutton.button)
	{
	  case JS_MODE_MANUAL:
          nps_joystick.mode = MODE_SWITCH_MANUAL;
	  break;

	  case JS_MODE_AUTO1:
	  nps_joystick.mode = MODE_SWITCH_AUTO1;
	  break;

	  case JS_MODE_AUTO2:
	  nps_joystick.mode = MODE_SWITCH_AUTO2;
	  break;

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
