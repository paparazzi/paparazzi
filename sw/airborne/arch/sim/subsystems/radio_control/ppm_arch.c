/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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

#include "sys_time.h"
#include "subsystems/radio_control.h"
#include "subsystems/radio_control/ppm.h"

#include <inttypes.h>
#include <caml/mlvalues.h>

#ifdef USE_NPS
#include "nps_radio_control.h"
#endif

uint8_t  ppm_cur_pulse;
uint32_t ppm_last_pulse_time;
bool_t   ppm_data_valid;

void ppm_arch_init ( void ) {
  ppm_last_pulse_time = 0;
  ppm_cur_pulse = RADIO_CONTROL_NB_CHANNEL;
  ppm_data_valid = FALSE;
  ppm_frame_available = FALSE;
}

#ifdef RADIO_CONTROL

value update_rc_channel(value c, value v) {
  ppm_pulses[Int_val(c)] = Double_val(v);
  return Val_unit;
}

value send_ppm(value unit) {
  ppm_frame_available = TRUE;
  return unit;
}

#ifdef USE_NPS
#define PPM_OF_NPS(_nps, _neutral, _min, _max)                          \
  ((_nps) >= 0 ? (_neutral) + (_nps) * ((_max)-(_neutral)) : (_neutral) + (_nps) * ((_neutral)- (_min)))

void radio_control_feed(void) {
  ppm_pulses[RADIO_ROLL]     = PPM_OF_NPS(nps_radio_control.roll,       \
                                          RADIO_ROLL_NEUTRAL,          \
                                          RADIO_ROLL_MIN,              \
                                          RADIO_ROLL_MAX);
  ppm_pulses[RADIO_PITCH]    = PPM_OF_NPS(nps_radio_control.pitch,      \
                                          RADIO_PITCH_NEUTRAL,         \
                                          RADIO_PITCH_MIN,             \
                                          RADIO_PITCH_MAX);
  ppm_pulses[RADIO_YAW]      = PPM_OF_NPS(nps_radio_control.yaw,        \
                                          RADIO_YAW_NEUTRAL,           \
                                          RADIO_YAW_MIN,               \
                                          RADIO_YAW_MAX);
  ppm_pulses[RADIO_THROTTLE] = PPM_OF_NPS(nps_radio_control.throttle,   \
                                          RADIO_THROTTLE_NEUTRAL,      \
                                          RADIO_THROTTLE_MIN,          \
                                          RADIO_THROTTLE_MAX);
  ppm_pulses[RADIO_MODE]     = PPM_OF_NPS(nps_radio_control.mode,       \
                                          RADIO_MODE_NEUTRAL,          \
                                          RADIO_MODE_MIN,              \
                                          RADIO_MODE_MAX);
  ppm_frame_available = TRUE;
}
#endif

#else // RADIO_CONTROL

value update_rc_channel(value c __attribute__ ((unused)), value v __attribute__ ((unused))) {
  return Val_unit;
}

value send_ppm(value unit) {
  return unit;
}

#ifdef USE_NPS
void radio_control_feed(void) {}
#endif

#endif // RADIO_CONTROL
