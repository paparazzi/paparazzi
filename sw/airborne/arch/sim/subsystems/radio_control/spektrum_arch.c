/*
 * Copyright (C) 2010-2012 The Paparazzi Team
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

/**
 * @file arch/sim/subsystems/radio_control/spektrum_arch.c
 *
 * Simulator implementation for spektrum radio control.
 *
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/spektrum_arch.h"
#include "std.h"
#include <inttypes.h>

#if USE_NPS
#include "nps_radio_control.h"
#else
#include <caml/mlvalues.h>
#endif

static bool spektrum_available;

void radio_control_impl_init(void)
{
  spektrum_available = false;
}
void spektrum_event(void (*frame_handler)(void))
{
  if (spektrum_available) {
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    radio_control.status = RC_OK;
    (*frame_handler)();
  }
  spektrum_available = false;
}

void spektrum_try_bind(void) {}

#if USE_NPS
#ifdef RADIO_CONTROL
void radio_control_feed(void)
{
  radio_control.values[RADIO_ROLL]     = nps_radio_control.roll * MAX_PPRZ;
  radio_control.values[RADIO_PITCH]    = nps_radio_control.pitch * MAX_PPRZ;
  radio_control.values[RADIO_YAW]      = nps_radio_control.yaw * MAX_PPRZ;
  radio_control.values[RADIO_THROTTLE] = nps_radio_control.throttle * MAX_PPRZ;
  radio_control.values[RADIO_MODE]     = nps_radio_control.mode * MAX_PPRZ;
  spektrum_available = true;
}
#else //RADIO_CONTROL
void radio_control_feed(void) {}
#endif //RADIO_CONTROL

#else // not NPS -> simple ocaml sim
#ifdef RADIO_CONTROL
value update_rc_channel(value c, value v)
{
  // OCaml sim sends ppm values read from radio xml
  //assume "ppm" value range from 1000 to 2000 for now.. like in fake spektrum.xml
  if (Int_val(c) == 0) {
    // throttle channel has neutral at 1000
    radio_control.values[Int_val(c)] = (Double_val(v) - 1000.0) / 1000 * MAX_PPRZ;
  } else {
    // all other channels at 1500
    radio_control.values[Int_val(c)] = (Double_val(v) - 1500.0) / 500 * MAX_PPRZ;
  }
  return Val_unit;
}

value send_ppm(value unit)
{
  spektrum_available = true;
  return unit;
}
#else // RADIO_CONTROL
value update_rc_channel(value c __attribute__((unused)), value v __attribute__((unused)))
{
  return Val_unit;
}
value send_ppm(value unit) {return unit;}
#endif // RADIO_CONTROL
#endif // USE_NPS
