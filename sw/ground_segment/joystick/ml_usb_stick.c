/*
   $Id$

 Copyright (C) 2009 ENAC, Pascal Brisset, Gautier Hattenberger

 Ocaml bindings for USB stick

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

#include <stdio.h>
#include <string.h>
#include <caml/mlvalues.h>
#include <caml/alloc.h>
#include <caml/memory.h>
#include <caml/callback.h>
#include <caml/fail.h>
#include "usb_stick.h"


/** Open an input device. Try to discover it if name is empty */
value
ml_stick_init(value device_name_val) {
  char * device_name = String_val(device_name_val);
  if (strlen(device_name) == 0)
    device_name = NULL;
  int opened = stick_init(device_name);
  return Val_int(opened);
}

/** Return a triple of
    - the number of buttons
    - one integer with the buttons values
    - the array of axis values */
CAMLprim value
ml_stick_read(value _unit) {
  CAMLparam0();
  CAMLlocal3 (result, buttons, axis);

  stick_read();

  buttons = Val_int(stick_button_values);

  axis = caml_alloc_tuple(stick_axis_count);
  unsigned int i;
  for(i = 0; i < stick_axis_count; i++)
    Store_field(axis, i, Val_int(stick_axis_values[i]));

  result = caml_alloc_tuple(3);
  Store_field(result, 0, Val_int(stick_button_count));
  Store_field(result, 1, buttons);
  Store_field(result, 2, axis);
  CAMLreturn(result);
}
