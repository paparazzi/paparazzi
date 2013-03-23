(*
 * Displaying Google Maps on a MapCanvas object
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

val display_tile : MapCanvas.widget -> Latlong.geographic -> int -> unit
(** Displaying the Google Maps tile around the given point (zoom=1) up to max level *)

val fill_window : MapCanvas.widget -> int -> unit
(** Filling the canvas window with Google Maps tiles at given zoomlevel*)

val pixbuf : Latlong.geographic -> Latlong.geographic -> int -> GdkPixbuf.pixbuf
(** [pixbuf south_west north_east zoomlevel] Returns a map background of the given area *)
