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
#include <caml/mlvalues.h>
#include <caml/alloc.h>
#include <caml/memory.h>
#include <inttypes.h>

#ifdef ARCH_ALIGN_DOUBLE
value
c_float_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  char *x = (char *)(String_val(s) + Int_val(index));

  //Assert(sizeof(float) == 4);
  union { char b[4]; float f; } buffer;
  buffer.b[0] = x[0];
  buffer.b[1] = x[1];
  buffer.b[2] = x[2];
  buffer.b[3] = x[3];

  CAMLreturn (copy_double((double)buffer.f));
}

value
c_double_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  char *x = (char *)(String_val(s) + Int_val(index));

  //Assert(sizeof(double) == 8);
  union { char b[sizeof(double)]; double d; } buffer;
  int i;
  for (i=0; i < sizeof(double); i++) {
    buffer.b[i] = x[i];
  }
  CAMLreturn (copy_double(buffer.d));
}

#else /* no ARCH_ALIGN_DOUBLE */
value
c_float_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  float *x = (float*)(String_val(s) + Int_val(index));
  CAMLreturn (copy_double((double)(*x)));
}

value
c_double_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  double *x = (double*)(String_val(s) + Int_val(index));
  CAMLreturn (copy_double(*x));
}
#endif /* ARCH_ALIGN_DOUBLE */

value
c_sprint_float(value s, value index, value f) {
  CAMLparam3 (s, index, f);
  float *p = (float*) (String_val(s) + Int_val(index));
  double ff = Double_val(f);
  *p = (float)ff;
  CAMLreturn (Val_unit);
}

value
c_sprint_double(value s, value index, value f) {
  CAMLparam3 (s, index, f);
  double *p = (double*) (String_val(s) + Int_val(index));
  *p = Double_val(f);
  CAMLreturn (Val_unit);
}

value
c_sprint_int16(value s, value index, value f) {
  CAMLparam3 (s, index, f);
  int16_t *p = (int16_t*) (String_val(s) + Int_val(index));
  *p = (int16_t)Int_val(f);
  CAMLreturn (Val_unit);
}

value
c_sprint_int8(value s, value index, value f) {
  CAMLparam3 (s, index, f);
  int8_t *p = (int8_t*) (String_val(s) + Int_val(index));
  *p = (int8_t)Int_val(f);
  CAMLreturn (Val_unit);
}

value
c_sprint_int32(value s, value index, value x) {
  CAMLparam3 (s, index, x);
  int32_t *p = (int32_t*) (String_val(s) + Int_val(index));
  *p = (int32_t)Int32_val(x);
  CAMLreturn (Val_unit);
}

value
c_sprint_int64(value s, value index, value x) {
  CAMLparam3 (s, index, x);
  int64_t *p = (int64_t*) (String_val(s) + Int_val(index));
  *p = (int64_t)Int64_val(x);
  CAMLreturn (Val_unit);
}

value
c_int16_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  int16_t *x = (int16_t*)(String_val(s) + Int_val(index));
  CAMLreturn (Val_int(*x));
}

value
c_int8_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  int8_t *x = (int8_t*)(String_val(s) + Int_val(index));
  CAMLreturn (Val_int(*x));
}

value
c_int32_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  int32_t *x = (int32_t*)(String_val(s) + Int_val(index));
  CAMLreturn (copy_int32(*x));
}

value
c_uint32_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  uint32_t *x = (uint32_t*)(String_val(s) + Int_val(index));

  /* since OCaml doesn't have unsigned integers,
   * we represent it as 64bit signed int to make sure it doesn't overflow
   */
  int64_t y = *x;
  CAMLreturn (copy_int64(y));
}

#ifdef ARCH_ALIGN_INT64
value
c_int64_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  char *x = (char *)(String_val(s) + Int_val(index));

  union { char b[sizeof(int64_t)]; int64_t i; } buffer;
  int i;
  for (i=0; i < sizeof(int64_t); i++) {
    buffer.b[i] = x[i];
  }
  CAMLreturn (copy_int64(buffer.i));
}
#else
value
c_int64_of_indexed_bytes(value s, value index)
{
  CAMLparam2 (s, index);
  int64_t *x = (int64_t*)(String_val(s) + Int_val(index));
  CAMLreturn (copy_int64(*x));
}
#endif
