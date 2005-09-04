/*
 * $Id$
 * Flight-time calibration facility
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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


#include <inttypes.h>
#include "radio.h"
#include "autopilot.h"
#include "if_calib.h"
#include "infrared.h"
#include "pid.h"
#include "nav.h"


#define ParamValInt16(param_init_val, param_travel, cur_pulse, init_pulse) \
(param_init_val + (int16_t)(((float)(cur_pulse - init_pulse)) * param_travel / (float)MAX_PPRZ))

#define ParamValFloat(param_init_val, param_travel, cur_pulse, init_pulse) \
(param_init_val + ((float)(cur_pulse - init_pulse)) * param_travel / (float)MAX_PPRZ)



uint8_t  inflight_calib_mode = IF_CALIB_MODE_NONE;

static int16_t slider1_init, slider2_init;

#include "estimator.h"
#include "inflight_calib.h"


/***
inline uint8_t inflight_calib(void) {
  static int16_t slider1_init, slider2_init;
  //static float ir_gain_init;
  //static float roll_pgain_init;
  static float course_pgain_init;
  static int16_t  roll_neutral_init;
  static float    pitch_pgain_init;
  static int16_t  pitch_neutral_init;

  int8_t mode_changed = inflight_calib_mode_update();

  if (inflight_calib_mode == IF_CALIB_MODE_NEUTRAL) {
    if (mode_changed) {
      pitch_neutral_init =  ir_pitch_neutral;
      roll_neutral_init  =  ir_roll_neutral;
      slider1_init = from_fbw.channels[RADIO_GAIN1];
      slider2_init = from_fbw.channels[RADIO_GAIN2];
    }
    ir_pitch_neutral = PARAM_VAL_INT16( pitch_neutral_init, -60., from_fbw.channels[RADIO_GAIN1], slider1_init);
    ir_roll_neutral  = PARAM_VAL_INT16( roll_neutral_init, 60., from_fbw.channels[RADIO_GAIN2], slider2_init);
  }
  else if (inflight_calib_mode == IF_CALIB_MODE_GAIN) {
    if (mode_changed) {
      //      ir_gain_init = ir_gain;
      course_pgain_init = course_pgain;
      // roll_pgain_init = roll_pgain;
      pitch_pgain_init = pitch_pgain;
      slider1_init = from_fbw.channels[RADIO_GAIN1];
      slider2_init = from_fbw.channels[RADIO_GAIN2];
    }
    course_pgain = PARAM_VAL_FLOAT( course_pgain_init, -0.1, from_fbw.channels[RADIO_GAIN1], slider1_init);
    //    ir_gain = PARAM_VAL_FLOAT( ir_gain_init, 0.0015, from_fbw.channels[RADIO_GAIN2], slider2_init);
    // roll_pgain = PARAM_VAL_FLOAT( roll_pgain_init, -5000., from_fbw.channels[RADIO_GAIN2], slider1_init);
    pitch_pgain = PARAM_VAL_FLOAT( pitch_pgain_init, -5000., from_fbw.channels[RADIO_GAIN1], slider1_init);
  }
  return (mode_changed);
}
***/





