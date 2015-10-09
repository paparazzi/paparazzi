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

#ifndef NPS_RADIO_CONTROL_JOYSTICK_H
#define NPS_RADIO_CONTROL_JOYSTICK_H

extern int nps_radio_control_joystick_init(const char *device);
extern void nps_radio_control_joystick_update(void);

struct NpsJoystick {
  double throttle;
  double roll;
  double pitch;
  double yaw;
  double mode;
};

extern struct NpsJoystick nps_joystick;

#endif /* NPS_RADIO_CONTROL_JOYSTICK_H */
