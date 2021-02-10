(*
 * Acces functions to SRTM data (http://edcftp.cr.usgs.gov/pub/data/srtm)
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec, Pascal Brisset, Antoine Drouin
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

val srtm_url : string

val add_path : string -> unit
(** [add_path directory] Adds [directory] to the current path where, possibly
compressed, SRTM data files are searched. *)

type error = string
exception Tile_not_found of error

val error : error -> string

val available : Latlong.geographic -> bool
(** [available wgs84_pos] Returns true if srtm tile is available *)

val of_utm : Latlong.utm -> int
(** [of_utm utm_pos] Returns the altitude of the given UTM position *)

val of_wgs84 : Latlong.geographic -> int
(** [of_wgs84 wgs84_pos] Returns the altitude of the given geographic position *)

val horizon_slope : Latlong.geographic -> int -> float -> float -> float -> float
(** [horizon_slope geo alt route half_aperture horizon] *)
