/*
 * Copyright (C) Kevin van Hecke
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/glide_wing_lock/glide_wing_lock.c"
 * @author Kevin van Hecke
 * Locks the wing of an ornicopter so it can glide.
 */

#include "modules/glide_wing_lock/glide_wing_lock.h"

#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"

#include "mcu_periph/adc.h"
#include "modules/core/commands.h"
#include "modules/radio_control/radio_control.h"
#include "autopilot.h"

struct adc_buf adcbuf;

int lock_wings;

#ifndef WING_POS_DOWN_THRESH
#define WING_POS_DOWN_THRESH 100
#endif
#ifndef WING_POS_LOCK_MIN_THRESH
#define WING_POS_LOCK_MIN_THRESH 2000
#endif
#ifndef WING_POS_LOCK_MAX_THRESH
#define WING_POS_LOCK_MAX_THRESH 2100
#endif
#ifndef WING_POS_NOMINAL_THRUST
#define WING_POS_NOMINAL_THRUST 5000
#endif
#ifndef WING_POS_LOCK_SWITCH
#define WING_POS_LOCK_SWITCH RADIO_AUX2
#endif


void glide_wing_lock_init(void)
{
  adc_buf_channel(ADC_CHANNEL_MOTORSENSOR, &adcbuf, 1);
}

void glide_wing_lock_event()
{
  static int lockstate = 0;
  if (radio_control.values[WING_POS_LOCK_SWITCH] > (MIN_PPRZ / 2)) { // check glide switch
    float wpos = adcbuf.sum / adcbuf.av_nb_sample;
    switch (lockstate) {
      case 0:
        if (wpos < WING_POS_DOWN_THRESH) { //set wings to fixed speed for one rotation starting from when wings are at lowest position
          lock_wings = 1;
          lockstate++;
        }
        break;
      case 1:
        if (wpos > WING_POS_LOCK_MIN_THRESH) { //start wait for a rotation
          lockstate++;
        }
        break;
      case 2:
        if (wpos < WING_POS_DOWN_THRESH) { //rotation finished
          lockstate++;
        }
        break;
      case 3:
        if (wpos > WING_POS_LOCK_MIN_THRESH && wpos < WING_POS_LOCK_MAX_THRESH) { // wait for exact wing position
          //esc brakes when throttle = 0, which should lock the wing in this position;
          lock_wings = 2;
          lockstate++;
        }
        break;
      default:
        break;
    }
  } else {
    lock_wings = 0;
    lockstate = 0;
  }
}

void glide_wing_lock_periodic()
{
  uint16_t wpos = adcbuf.sum / adcbuf.av_nb_sample;
  DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, DefaultDevice, &wpos, &wpos);
}

void set_rotorcraft_commands(pprz_t *cmd_out, int32_t *cmd_in, bool in_flight, bool motors_on)
{
  if (!(in_flight)) { cmd_in[COMMAND_YAW] = 0; }
  if (!(motors_on)) { cmd_in[COMMAND_THRUST] = 0; }
  cmd_out[COMMAND_ROLL] = cmd_in[COMMAND_ROLL];
  cmd_out[COMMAND_PITCH] = cmd_in[COMMAND_PITCH];
  cmd_out[COMMAND_YAW] = cmd_in[COMMAND_YAW];
  cmd_out[COMMAND_THRUST] = cmd_in[COMMAND_THRUST];

  if (lock_wings == 1 && motors_on && cmd_out[COMMAND_THRUST] > 0) {
    cmd_out[COMMAND_THRUST] = WING_POS_NOMINAL_THRUST;
  } else if (lock_wings == 2) {
    cmd_out[COMMAND_THRUST] = 0;
  }
}



