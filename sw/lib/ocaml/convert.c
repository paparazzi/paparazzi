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
#include <termio.h>
#include <stdio.h>
#include <sys/termios.h>
#include "caml/mlvalues.h"
#include "caml/alloc.h"

value 
c_float_of_indexed_bytes(value s, value index)
{
  float *x = (float*)(String_val(s) + Int_val(index));

  return copy_double((double)(*x));
}

value
c_sprint_float(value s, value index, value f) {
  char *p = String_val(s) + Int_val(index);
  *p = (float)Double_val(f);
  return Val_unit;
}

value
c_sprint_int32(value s, value index, value x) {
  char *p = String_val(s) + Int_val(index);
  *p = Int32_val(x);
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
