(*
 * Utilities
 *
 * Copyright (C) 2004-2009 CENA/ENAC, Yann Le Fablec, Pascal Brisset
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 *)

val open_compress : string -> in_channel
(** [open_compress file] Opens [file] compressed or non-compressed.
    [.gz], [.Z], [.bz2], [.zip] and [.ZIP] recognized *)

val compress : string -> unit
(** [compress file] Compress [file] with gzip *)

val find_file : string list -> string -> string
(** [find_file path file] Search for [file] or a compressed extension of it in
    [path]. Returns the first occurence found. Checked extensions are
    gz, Z, bz2, zip and ZIP.
    Raises [Not_found] if none is found *)

val affine_transform : string -> float * float
(* [affine_transform format] Parses [format] as a+b and returns (a,b). Returns
a if +b is not present. Returns 1. if the parsing fails *)

val normal : float -> float -> float
(* [normal mean stdev] Normal distribution *)

val make_1st_order_noise_generator :
  ?init:float -> float -> float -> (unit -> float)
(* [make_1st_order_noise_generator ?init k sigma] Returns a generator
   initialized to [init], damped by [k] with a variation of stdev [sigma].
   0 < [k] < 1 . x_n+1 <- k x_n + normal 0 sigma . *)

val shifter : int -> 'a -> ('a -> 'a)
(* [shifter size init] Returns a shift register containing [size] values
   initialized to [init] *)

val assoc_opt : 'a -> ('a * 'b) list -> 'b option
(* [assoc_opt a l] Returns the result of [List.assoc a l]
   or None if [a] not in [l] *)

val assoc_opt_map : 'a -> ('a * 'b) list -> ('b -> 'c) -> 'c option
(* [assoc_opt_map a l f] Returns the result of [f (List.assoc a l)]
   or None if [a] not in [l] *)

val assoc_opt_int : 'a -> ('a * string) list -> int option
(* [assoc_opt_map a l f] Returns the result of [int_of_string (List.assoc a l)]
   or None if [a] not in [l] *)

val assoc_default : 'a -> ('a * 'b) list -> 'b -> 'b
(* [assoc_default a l d] Returns the result of [List.assoc a l]
   or [d] if [a] not in [l] *)

