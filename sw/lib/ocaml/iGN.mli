(*
 * Handling IGN tiles
 *
 * Copyright (C) 2004-2006 ENAC, Pascal Brisset, Antoine Drouin
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

val tile_size : int * int (** Pixels *)

type tile_t = {
    key : int * int; (* LambertIIe meters / size *)
    sw_corner : Latlong.geographic;
    ne_corner : Latlong.geographic
  };;

val cache_path : string ref
(** Where to cache *)

val data_path : string ref
(** Where to find the original tiles *)

val tile_of_geo : Latlong.geographic -> tile_t

val get_tile : tile_t -> string
(** Returns the filename of the given tile *)
