/* 
 * $Id: $
 *
 * Copyright (C) 2009 Gautier Hattenberger <gautier.hattenberger@laas.fr>,
 *                    Antoine Drouin <poinix@gmail.com>
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

#ifndef BOOZ_CAM_H
#define BOOZ_CAM_H

#include "airframe.h"
#include "std.h"
#include "led.h"

#define BOOZ_CAM_MODE_NONE     0
#define BOOZ_CAM_MODE_MANUAL   1
#define BOOZ_CAM_MODE_HEADING  2
#define BOOZ_CAM_MODE_WP       3

extern uint8_t booz_cam_mode;

#ifdef BOOZ_CAM_TILT_NEUTRAL
extern int16_t booz_cam_tilt_pwm;
extern int16_t booz_cam_tilt;
#endif
#ifdef BOOZ_CAM_PAN_NEUTRAL
extern int16_t booz_cam_pan;
#endif

extern void booz_cam_init(void);
extern void booz_cam_periodic(void);

#define booz_cam_SetCamMode(_v) { \
  booz_cam_mode = _v; \
  if (booz_cam_mode == BOOZ_CAM_MODE_NONE) { LED_ON(CAM_SWITCH_LED); } \
  else { LED_OFF(CAM_SWITCH_LED); } \
}

#endif /* BOOZ2_CAM_H */

