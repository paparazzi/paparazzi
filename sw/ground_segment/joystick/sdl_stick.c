/*
 * Basic SDL joystick lib
 *
 * based on usb_stick.c
 * Copyright (C) 2012 The Paparazzi Team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "sdl_stick.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>

//needed for joystick interface
#include <SDL/SDL.h>

#ifdef STICK_DBG
#define dbgprintf fprintf
#else
#define dbgprintf(x ...)
#endif

#define MIN_ABS_CODE      ABS_X
#define MAX_ABS_CODE      ABS_MAX+1

#define ABS_MAX_VALUE     254
#define ABS_MID_VALUE     127

#define BUTTON_COUNT      STICK_BUTTON_COUNT
#define AXIS_COUNT        STICK_AXIS_COUNT


SDL_Joystick *sdl_joystick;
SDL_Event sdl_event;

int8_t stick_axis_values[AXIS_COUNT] = {0, 0, 0, 0, 0, 0};
uint8_t stick_hat_value = 0;
int32_t stick_button_values = 0;

int8_t stick_axis_moved[AXIS_COUNT] = {0};

int stick_axis_count = 0;
int stick_button_count = 0;


int32_t axis_min[AXIS_COUNT], axis_max[AXIS_COUNT];

int min(int x, int y);


int init_sdl_device(int device_index)
{
  int cnt;
  const char * name;

  stick_button_count = 0;
  stick_axis_count = 0;

  /* Open the device with SDL */
  sdl_joystick = SDL_JoystickOpen(device_index);
  if (sdl_joystick == NULL)
  {
    dbgprintf(stderr,"Joystick corresponding to SDL Index %d failed to open! Exiting.\n", device_index);
    return(1);
  }

  /* How many buttons available */
  stick_button_count = min(SDL_JoystickNumButtons(sdl_joystick),BUTTON_COUNT);
  dbgprintf(stderr,"Available button: %d (0x%x)\n",stick_button_count,stick_button_count);
  if (stick_button_count == 0) {
    dbgprintf(stderr,"ERROR: no suitable buttons found [%s:%d]\n",__FILE__,__LINE__);
  }

  /* How many POV hats available */
  int stick_hat_count = SDL_JoystickNumHats(sdl_joystick);
  dbgprintf(stderr,"Available hats: %d (0x%x)\n",stick_hat_count,stick_hat_count);
  if (stick_hat_count > 1) {
    dbgprintf(stderr,"ERROR: only one POV hat supported [%s:%d]\n",__FILE__,__LINE__);
  }

  /* How many axes available */
  stick_axis_count = min(SDL_JoystickNumAxes(sdl_joystick),AXIS_COUNT);
  dbgprintf(stderr,"Available axes: %d (0x%x)\n",stick_axis_count,stick_axis_count);
  if (stick_button_count < 2) {
    dbgprintf(stderr,"ERROR: not enough suitable axes found [%s:%d]\n",__FILE__,__LINE__);
  }

  /* Axis param */
  for (cnt = 0; cnt < stick_axis_count; cnt++)
  {
    // with joystick interface, all axes are signed 16 bit with full range
    axis_min[cnt]=-32768;
    axis_max[cnt]=32768;

    dbgprintf(stderr,"Axis %d : parameters = [%d,%d]\n",
        cnt,axis_min[cnt],axis_max[cnt]);
  }

  /* Get the device name */
  name = SDL_JoystickName(device_index);
  if (name == NULL)
  {
    dbgprintf(stderr,"Error getting name of device with SDL index %i.\n",device_index);
  } else {
    printf("Input device name: \"%s\" on SDL device \"%i\"\n", name, device_index);
  }

  return(0);
}

int stick_read( void ) {

  int cnt;

  while(SDL_PollEvent(&sdl_event))
  {
    switch(sdl_event.type)
    {
      case SDL_JOYBUTTONDOWN:
        //falls through to JOYBUTTONUP
      case SDL_JOYBUTTONUP:
        for (cnt = 0; cnt < stick_button_count; cnt++) {
          if (sdl_event.jbutton.button == cnt) {
            if (sdl_event.jbutton.state == SDL_PRESSED) {
              stick_button_values |= (1 << cnt); // Set bit
            } else stick_button_values &= ~(1 << cnt); // Clear bit
            break;
          }
        }
        break;

      case SDL_JOYHATMOTION:
        // only one hat (with index 0) supported
        if (sdl_event.jhat.hat == 0) {
          stick_hat_value = sdl_event.jhat.value;
          break;
        }
        break;

      case SDL_JOYAXISMOTION:
        for (cnt = 0; cnt < stick_axis_count; cnt++) {
          if (sdl_event.jaxis.axis == cnt) {
            stick_axis_values[cnt] = (( (sdl_event.jaxis.value - axis_min[cnt]) * ABS_MAX_VALUE ) / (axis_max[cnt] - axis_min[cnt])) - ABS_MID_VALUE;
            break;
          }
        }
        break;

      case SDL_QUIT:
        printf("Quitting...\n");
        exit(1);
        break;

      default:
        //do nothing
        //printf("unknown SDL event!!!\n");
        break;
    }
  }

  dbgprintf(stderr, "buttons ");
  for (cnt = 0; cnt < stick_button_count; cnt++) {
    dbgprintf(stderr, "%d ", (stick_button_values >> cnt) & 1 );
  }

  dbgprintf(stderr, "| hat ");
  dbgprintf(stderr, "%d ", stick_hat_value);

  dbgprintf(stderr, "| axes ");
  for (cnt = 0; cnt < stick_axis_count; cnt++) {
    dbgprintf(stderr, "%d ", stick_axis_values[cnt]);
  }

  dbgprintf(stderr, "\n");

  return 0;
}

int stick_check_axis(void)
{
  int cnt;
  while (SDL_PollEvent(&sdl_event)) {
    switch (sdl_event.type) {
      case SDL_JOYAXISMOTION:
        for (cnt = 0; cnt < stick_axis_count; cnt++) {
          if (sdl_event.jaxis.axis == cnt) {
            stick_axis_values[cnt] = (((sdl_event.jaxis.value - axis_min[cnt]) * ABS_MAX_VALUE) / (axis_max[cnt] - axis_min[cnt])) - ABS_MID_VALUE;
            stick_axis_moved[cnt] = 1;
            break;
          }
        }
        break;

      case SDL_QUIT:
        printf("Quitting...\n");
        exit(1);
        break;

      default:
        //do nothing
        break;
    }
  }
  
  for (cnt = 0; cnt < stick_axis_count; cnt++) {
    if (stick_axis_moved[cnt] == 0) {
      return 0;
    }
  }
  return 1;
  
}

int stick_init( int device_index ) {

  int cnt = 0;

  /* Initialize SDL with joystick support and event support (through video) */
  if (SDL_Init(SDL_INIT_JOYSTICK|SDL_INIT_VIDEO) < 0)
  {
    printf("Could not initialize SDL: %s.\n", SDL_GetError());
    exit(-1);
  }

  //Quit SDL at exit
  atexit(SDL_Quit);

  //Start the event handler, disable all but joystick events and quit handler
  SDL_EventState(SDL_ACTIVEEVENT,SDL_IGNORE);
  SDL_EventState(SDL_KEYDOWN,SDL_IGNORE);
  SDL_EventState(SDL_KEYUP,SDL_IGNORE);
  SDL_EventState(SDL_MOUSEMOTION,SDL_IGNORE);
  SDL_EventState(SDL_MOUSEBUTTONDOWN,SDL_IGNORE);
  SDL_EventState(SDL_MOUSEBUTTONUP,SDL_IGNORE);
  //SDL_EventState(SDL_JOYAXISMOTION,SDL_IGNORE);
  //SDL_EventState(SDL_JOYBALLMOTION,SDL_IGNORE);
  //SDL_EventState(SDL_JOYHATMOTION,SDL_IGNORE);
  //SDL_EventState(SDL_JOYBUTTONDOWN,SDL_IGNORE);
  //SDL_EventState(SDL_JOYBUTTONUP,SDL_IGNORE);
  SDL_EventState(SDL_VIDEORESIZE,SDL_IGNORE);
  SDL_EventState(SDL_VIDEOEXPOSE,SDL_IGNORE);
  //SDL_EventState(SDL_QUIT,SDL_IGNORE);
  SDL_EventState(SDL_USEREVENT,SDL_IGNORE);
  SDL_EventState(SDL_SYSWMEVENT,SDL_IGNORE);

  //Check there are actually joysticks attached
  if (!SDL_NumJoysticks())
  {
    fprintf(stderr,"Error: No joysticks attached!\n");
    SDL_Quit();
    return(1);
  }

  /* test device_index, else look for a suitable device */
  if (init_sdl_device(device_index) != 0)
  {
    printf("Failed to open joystick at SDL device index %d, attempting to find a suitable joystick...\n",device_index);
    for (cnt = 0; cnt < STICK_INPUT_DEV_MAX; cnt++) {
      if (init_sdl_device(cnt) == 0) break;
    }
    printf("Found an alternative device!\n");
  }

  /* return 1 if no device found */
  if (cnt == STICK_INPUT_DEV_MAX) {
    fprintf(stderr,"ERROR: no suitable joystick found [%s:%d]\n",
        __FILE__,__LINE__);
    SDL_Quit();
    return(1);
  }

  return 0;
}

int min(int x, int y)
{
  return ( x > y ) ? y : x;
}
