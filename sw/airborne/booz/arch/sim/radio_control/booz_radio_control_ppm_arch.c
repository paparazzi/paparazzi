/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "booz_radio_control.h"

#include "nps_radio_control.h"

void booz_radio_control_ppm_arch_init ( void ) {

}

#define PPM_OF_NPS(_nps, _neutral, _min, _max) 			\
  ((_nps) >= 0 ? (_neutral) + (_nps) * ((_max)-(_neutral)) : (_neutral) + (_nps) * ((_neutral)- (_min)))

void booz_radio_control_feed(void) {
  booz_radio_control_ppm_pulses[RADIO_CONTROL_ROLL]     = PPM_OF_NPS(nps_radio_control.roll,     1500, 950,  2050);
  booz_radio_control_ppm_pulses[RADIO_CONTROL_PITCH]    = PPM_OF_NPS(nps_radio_control.pitch,    1500, 2050,  950);
  booz_radio_control_ppm_pulses[RADIO_CONTROL_YAW]      = PPM_OF_NPS(nps_radio_control.yaw,      1500, 2050,  950);
  booz_radio_control_ppm_pulses[RADIO_CONTROL_THROTTLE] = PPM_OF_NPS(nps_radio_control.throttle, 1223, 1223, 2050);
  booz_radio_control_ppm_pulses[RADIO_CONTROL_MODE]     = PPM_OF_NPS(nps_radio_control.mode,     1500, 2050,  940);
  booz_radio_control_ppm_frame_available = TRUE;
}

