/*
   $Id$

   Copyright (C) 2004 Pascal Brisset, Antoine Drouin

 Ocaml low level conversions

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

#include <sys/types.h>
#include <fcntl.h>
#include <sys/termios.h>
#include <stdio.h>
#include "caml/mlvalues.h"
#include "caml/alloc.h"

value
c_float_of_indexed_bytes(value s, value index)
{
  float *x = (float*)(String_val(s) + Int_val(index));

  return copy_double((double)(*x));
}

value
c_double_of_indexed_bytes(value s, value index)
{
  double *x = (double*)(String_val(s) + Int_val(index));

  return copy_double(*x);
}

value
c_sprint_float(value s, value index, value f) {
  float *p = (float*) (String_val(s) + Int_val(index));
  double ff = Double_val(f);
  *p = (float)ff;
  return Val_unit;
}

value
c_sprint_double(value s, value index, value f) {
  double *p = (double*) (String_val(s) + Int_val(index));
  *p = Double_val(f);
  return Val_unit;
}

value
c_sprint_int16(value s, value index, value f) {
  int16_t *p = (int16_t*) (String_val(s) + Int_val(index));
  *p = (int16_t)Int_val(f);
  return Val_unit;
}

value
c_sprint_int8(value s, value index, value f) {
  int8_t *p = (int8_t*) (String_val(s) + Int_val(index));
  *p = (int8_t)Int_val(f);
  return Val_unit;
}

value
c_sprint_int32(value s, value index, value x) {
  int32_t *p = (int32_t*) (String_val(s) + Int_val(index));
  *p = (int32_t)Int32_val(x);
  return Val_unit;
}

value
c_sprint_int64(value s, value index, value x) {
  int64_t *p = (int64_t*) (String_val(s) + Int_val(index));
  *p = (int64_t)Int64_val(x);
  return Val_unit;
}

value
c_int16_of_indexed_bytes(value s, value index)
{
  int16_t *x = (int16_t*)(String_val(s) + Int_val(index));

  return Val_int(*x);
}

value
c_int8_of_indexed_bytes(value s, value index)
{
  int8_t *x = (int8_t*)(String_val(s) + Int_val(index));

  return Val_int(*x);
}

value
c_int32_of_indexed_bytes(value s, value index)
{
  int32_t *x = (int32_t*)(String_val(s) + Int_val(index));

  return copy_int32(*x);
}

value
c_int64_of_indexed_bytes(value s, value index)
{
  int64_t *x = (int64_t*)(String_val(s) + Int_val(index));

  return copy_int64(*x);
}
