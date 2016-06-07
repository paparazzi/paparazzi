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

val bytes_create : int -> string
val bytes_contains : string -> char -> bool
val bytes_length : string -> int
val bytes_make : int -> char -> string
val bytes_copy : string -> string
val bytes_lowercase : string -> string
val bytes_uppercase : string -> string
val bytes_blit : string -> int -> string -> int -> int -> unit
val bytes_sub : string -> int -> int -> string
val bytes_index : string -> char -> int
val bytes_concat : string -> string list -> string
val bytes_index_from : string -> int -> char -> int
val bytes_get : string -> int -> char
val bytes_compare : string -> string -> int
val bytes_set : string -> int -> char -> unit
val bytes_iter : (char -> unit) -> string -> unit
