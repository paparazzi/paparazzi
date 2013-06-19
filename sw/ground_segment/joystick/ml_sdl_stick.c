/*
 Copyright (C) 2009 ENAC, Pascal Brisset, Gautier Hattenberger
 Copyright (C) 2012 The Paparazzi Team

 Ocaml bindings for SDL joystick support

 This file is part of paparazzi.

 paparazzi is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2, or (at your option)
 any later version.

 paparazzi is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with paparazzi; see the file COPYING.  If not, write to
 the Free Software Foundation, 59 Temple Place - Suite 330,
 Boston, MA 02111-1307, USA.
*/

/**
 * @file ml_sdl_stick.c
 * Ocaml bindings for basic SDL joystick support.
 */

#include <stdio.h>
#include <string.h>
#include <caml/mlvalues.h>
#include <caml/alloc.h>
#include <caml/memory.h>
#include <caml/callback.h>
#include <caml/fail.h>
#include "sdl_stick.h"


/** Open an input device. Try to discover alternate if given index fails
 *
 *  @param device_index_val  an ocaml integer with the target device index
 */
value
ml_stick_init(value device_index_val) {
  int device_index = Int_val(device_index_val);
  int opened = stick_init(device_index);
  return Val_int(opened);
}

/** Return a tuple with
 *  - the number of buttons
 *  - one integer with the buttons values
 *  - one integer with the hat value
 *  - the array of axis values
 *
 * @param _unit  placeholder for ocaml equivalent of void
 */
CAMLprim value
ml_stick_read(value _unit) {
  CAMLparam0();
  CAMLlocal4 (result, buttons, hat, axis);

  stick_read();

  buttons = Val_int(stick_button_values);
  hat = Val_int(stick_hat_value);

  axis = caml_alloc_tuple(stick_axis_count);
  unsigned int i;
  for(i = 0; i < stick_axis_count; i++)
    Store_field(axis, i, Val_int(stick_axis_values[i]));

  result = caml_alloc_tuple(4);
  Store_field(result, 0, Val_int(stick_button_count));
  Store_field(result, 1, buttons);
  Store_field(result, 2, hat);
  Store_field(result, 3, axis);
  CAMLreturn(result);
}
