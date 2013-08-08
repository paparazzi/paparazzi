/*
 * Basic SDL joystick lib
 *
 * based on usb_stick.h
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

/**
 *  @file sdl_stick.h
 *  Basic SDL joystick lib.
 */

#ifndef SDL_STICK_H
#define SDL_STICK_H

#include <SDL/SDL.h>

/* Max number of axis and buttons */
/*  Increased, many new controllers have pressure sensitive buttons that show up as axes */
#define STICK_BUTTON_COUNT    32
#define STICK_AXIS_COUNT      32

/* Default values for the options */
#define STICK_INPUT_DEV_MAX   15

/* Global variables about the initialized device */
extern int8_t stick_axis_values[STICK_AXIS_COUNT];
extern int32_t stick_button_values;
extern uint8_t stick_hat_value;
extern int stick_axis_count, stick_button_count;

/** Initialize a joystick with SDL.
 *
 *  @param device_index  which device index to open (SDL enumerates all available devices at init)
 *
 *  @returns  1 if no joystick found, 0 otherwise
 */
extern int stick_init( int device_index );

/** Update joystick values.
 *
 *  Updates stick_axis_values array and stick_button_values
 *
 *  @returns  0
 */
extern int stick_read( void );

#endif
