/*
 * joystick lib
 *
 * based on Force Feedback: Constant Force Stress Test
 * Copyright (C) 2001 Oliver Hamann
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

#ifndef USB_STICK_H
#define USB_STICK_H

#include <linux/input.h>

/* Max number of axis and buttons */
/*  Increased, many new controllers have pressure sensitive buttons that show up as axes */
#define STICK_BUTTON_COUNT    32
#define STICK_AXIS_COUNT      32

/* Default values for the options */
#define STICK_INPUT_DEV_MAX   15
#define STICK_DEVICE_NAME     "/dev/input/event"

/* Event mode switched */
#define STICK_MODE_UNKNOWN    0
#define STICK_MODE_EVENT      1
#define STICK_MODE_JOYSTICK   2

/* Max name length */
#define MAX_NAME_LENGTH       255

/* Global variables about the initialized device */
extern int stick_device_handle;
extern int8_t stick_axis_values[STICK_AXIS_COUNT];
extern int32_t stick_button_values;
extern int stick_axis_count, stick_button_count;

/* Structure for custom configuration
 * if button_count > 0, button_code is used (valid range:[0x120, 0x13e])
 * if axis_count > 0, axis_code is used (valid range: [0x00, 0x3f])
 * codes are defined in "linux/input.h"
 */
struct stick_code_param_ {
  int button_count;
  int button_code[STICK_BUTTON_COUNT];
  int axis_count;
  int axis_code[STICK_AXIS_COUNT];
};
extern struct stick_code_param_ stick_init_param;

/* Init: device_name = NULL for automatic search
 * return 1 if no joystick found
 */
extern int stick_init( char * device_name );

/* Update values */
extern int stick_read( void );

#endif

