(*
 * Displaying IGN Maps on a MapCanvas object
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

module LL = Latlong

let displayed_tiles = Hashtbl.create 41
let mem_tile = fun t -> Hashtbl.mem displayed_tiles t.IGN.key
let add_tile = fun t -> Hashtbl.add displayed_tiles t.IGN.key ()


let opacity = 100 (* FIXME *)

(** Displaying the tile around the given point *)
let display_tile = fun (geomap:MapCanvas.widget) wgs84 ->
  let tile = IGN.tile_of_geo wgs84 in

  if not (mem_tile tile) then
    let jpg_file = IGN.get_tile tile in

    let (sx,sy) = IGN.tile_size in
    let pixbuf = GdkPixbuf.from_file jpg_file in

    let map = geomap#display_pixbuf ~opacity ((0,sx), tile.IGN.sw_corner) ((sy,0),tile.IGN.ne_corner) pixbuf in
    map#raise 1;
    add_tile tile
