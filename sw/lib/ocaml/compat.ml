(*
 * Abstract some String/Byte functions for compatibility between OCaml 3.x and 4.x
 *
 * Copyright (C) 2016 Felix Ruess <felix.ruess@gmail.com>
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
 *)

 IFDEF HAS_BYTES_MODULE THEN
 module BYTES = Bytes
 ELSE
 module BYTES = String
 END

let bytes_create = fun len ->
  BYTES.create len

let bytes_contains = fun c s ->
  BYTES.contains c s

let bytes_length = fun len ->
  BYTES.length len

let bytes_make = fun n c->
  BYTES.make n c

let bytes_copy = fun s->
  BYTES.copy s

let bytes_blit = fun src srcoff dst dstoff len->
  BYTES.blit src srcoff dst dstoff len

let bytes_sub = fun s start len->
  BYTES.sub s start len

let bytes_index = fun c s ->
  BYTES.index c s

let bytes_concat = fun sep sl->
  BYTES.concat sep sl

let bytes_index_from = fun s i c ->
  BYTES.index_from s i c

let bytes_get = fun s n->
  BYTES.get s n

let bytes_compare = fun s1 s2->
  BYTES.compare s1 s2

let bytes_set = fun s n c->
  BYTES.set s n c

let bytes_iter = fun f s->
  BYTES.iter f s

IFDEF OCAML_V404 THEN
let lowercase_ascii = BYTES.lowercase_ascii

let uppercase_ascii = BYTES.uppercase_ascii

let capitalize_ascii = BYTES.capitalize_ascii

ELSE
let lowercase_ascii = BYTES.lowercase

let uppercase_ascii = BYTES.uppercase

let capitalize_ascii = BYTES.capitalize

END
