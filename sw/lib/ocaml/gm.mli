(*
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
val zoom_max : int
val zoom_min : int
val tile_coverage : float -> int -> float * float
(** [tile_coverage wgs84_lat zoom] Returns (width,height) *)

type tile_t = {
  key : string; (* [qrst] string *)
  sw_corner : Latlong.geographic;
  width : float; (* Longitude difference *)
  height : float (* Latitude difference *)
}

type maps_source = Google | OSM | MS | MQ | MQ_Aerial
val string_of_maps_source : maps_source -> string
val maps_sources : maps_source list
val set_maps_source : maps_source -> unit
val get_maps_source : unit -> maps_source
(** Initialized to Google *)

val tile_of_geo : ?level:int -> Latlong.geographic -> int -> tile_t
(** [tile_string geo zoom] Returns the tile description containing a
    given point with a the smallest available zoom greater or equal to [zoom]. *)

val tile_of_key : string -> tile_t
(** [tile_of_key google_maps_tile_key] Returns tile description of a
    named tile. *)

val cache_path : string ref

type policy =  CacheOrHttp | NoHttp | NoCache
val string_of_policy : policy -> string
val policies : policy list
val set_policy : policy -> unit
val get_policy : unit -> policy
(** Initialized to CacheOrHttp using cache and http access *)

exception Not_available

type hashtbl_cache = (string, string)Hashtbl.t

val get_hashtbl_of_cache : unit -> hashtbl_cache

val get_image : ?tbl:hashtbl_cache -> string -> tile_t * string
(** [get_image key] Returns the tile description and the image file name.
    May raise [Not_available] *)

val get_tile : Latlong.geographic -> int -> tile_t*string
(** [get_tile geo zoom] May raise [Not_available] *)
