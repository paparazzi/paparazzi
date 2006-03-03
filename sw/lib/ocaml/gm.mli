(*
 * $Id$
 *
 * Google Maps utilities
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset
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

val tile_size : int * int

type tile_t = {
    key : string; (* [qrst] string *)
    sw_corner : Latlong.geographic;
    width : float; (* Longitude difference *)
    height : float (* Latitude difference *)
  }
      
val tile_of_geo : Latlong.geographic -> int -> tile_t
(** [tile_string geo zoom] Returns the tile description containing a
   given point *)

val tile_of_key : string -> tile_t
(** [tile_of_key google_maps_tile_key] Returns tile description of a
   named tile. *)

val cache_path : string ref

val get_tile : ?no_http:bool -> Latlong.geographic -> int -> tile_t*string
(** May raise [Not_available] *)

exception Not_available

