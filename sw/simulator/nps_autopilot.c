/*
 * $Id$
 *  
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

#include "nps_autopilot.h"

#include "booz2_main.h"
#include "nps_sensors.h"

struct NpsAutopilot autopilot;


void nps_autopilot_init(void) {

  /* Just for testing fdm */
  double hover = 0.2495;

  autopilot.commands[SERVO_FRONT] = hover;
  autopilot.commands[SERVO_BACK]  = hover;
  autopilot.commands[SERVO_RIGHT] = hover;
  autopilot.commands[SERVO_LEFT]  = hover;
  
  booz2_main_init();

}


void nps_autopilot_run_step(double time __attribute__ ((unused))) {
  double hover = 0.2493;
  autopilot.commands[SERVO_FRONT] = hover;
  autopilot.commands[SERVO_BACK]  = hover;
  autopilot.commands[SERVO_RIGHT] = hover;
  autopilot.commands[SERVO_LEFT]  = hover;
  
  if (nps_sensors_gyro_available()) {
    //    booz2_imu_feed_data();
    //    booz2_main_event();
  }


  booz2_main_periodic();


}
