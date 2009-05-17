(*
 * $Id$
 *
 * Utilities
 *
 * Copyright (C) 2004-2009 CENA/ENAC, Yann Le Fablec, Pascal Brisset
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

val open_compress : string -> in_channel
(** [open_compress file] Opens [file] compressed or non-compressed.
    [.gz], [.Z], [.bz2], [.zip] and [.ZIP] recognized *)

val find_file : string list -> string -> string
(** [find_file path file] Search for [file] or a compressed extension of it in
    [path]. Returns the first occurence found. Checked extensions are 
    gz, Z, bz2, zip and ZIP.
    Raises [Not_found] if none is found *)

val affine_transform : string -> float * float
(* [affine_transform format] Parses [format] as a+b and returns (a,b). Returns
a if +b is not present. Returns 1. if the parsing fails *)
