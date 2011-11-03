/*
 * $Id$
 *
 * Copyright (C) 2011 Stephen Dwyer, based on trigger_ext_hw by Martin Mueller
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
 *
 */


#include "core/pwm_measure_hw.h"
#include "pwm_input.h" //get the pwm_input drivers
#include "std.h"
#include "sys_time_hw.h"
#include "LPC21xx.h"
#include BOARD_CONFIG


void pwm_measure_init ( void ) {

  //Assign the arch indep array as alias of arch dep array, links arch indep to arch dep parts of code
  pwm_meas_duty_tics = pwm_input_duration;
  pwm_meas_duty_valid = pwm_input_valid;
  
  //Use the pwm_input.* hardware dependent for lpc2148, this will change for each arch
  pwm_input_init();

}