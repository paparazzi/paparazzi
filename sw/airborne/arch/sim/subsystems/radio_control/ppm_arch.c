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
#include <subsystems/radio_control.h>
#include <subsystems/radio_control/ppm.h>

#include <inttypes.h>
#include <caml/mlvalues.h>

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

#else // RADIO_CONTROL

value update_rc_channel(value c __attribute__ ((unused)), value v __attribute__ ((unused))) {
  return Val_unit;
}

value send_ppm(value unit) {
  return unit;
}

#endif // RADIO_CONTROL
