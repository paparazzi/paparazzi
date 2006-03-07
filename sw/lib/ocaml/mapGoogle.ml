(*
 * $Id$
 *
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

module LL = Latlong

(** Quadtreee of displayed tiles *)
type tiles_tree = 
    Empty
  | Tile
  | Node of tiles_tree array
let gm_tiles = Node (Array.create 4 Empty)

(** Google Maps paths in the quadtree are coded with q,r,s and t*)
let index_of = function
    'q' -> 0 | 'r' -> 1 | 's' -> 2 | 't' -> 3
  | _ -> invalid_arg "index_of"

(** Checking that a tile is already displayed *)
let mem_tile = fun tile_key ->
  let rec loop = fun i tree ->
    tree = Tile ||
    i < String.length tile_key &&
    match tree with
      Empty -> false
    | Tile -> true
    | Node sons -> loop (i+1) sons.(index_of tile_key.[i]) in
  loop 0 gm_tiles

(** Adding a tile to the store *)
let add_tile = fun tile_key ->
  let rec loop = fun i tree j ->
    if i < String.length tile_key then
      match tree.(j) with
	Empty -> 
	  let sons = Array.create 4 Empty in
	  tree.(j) <- Node sons;
	  loop (i+1) sons (index_of tile_key.[i])
      | Tile -> () (* Already there *)
      | Node sons -> 
	  loop (i+1) sons (index_of tile_key.[i])
    else
      tree.(j) <- Tile in
  loop 0 [|gm_tiles|] 0


(** Displaying the tile around the given point *)
let display_tile = fun (geomap:MapCanvas.widget) wgs84 ->
  let desired_tile = Gm.tile_of_geo wgs84 1 in

  let key = desired_tile.Gm.key in
  if not (mem_tile key) then
    let (tile, jpg_file) = Gm.get_tile wgs84 1 in
    let south_lat = tile.Gm.sw_corner.LL.posn_lat
    and west_long = tile.Gm.sw_corner.LL.posn_long in
    let north_lat = south_lat +. tile.Gm.height
    and east_long = west_long +. tile.Gm.width in
    let ne = { LL.posn_lat = north_lat; posn_long = east_long } in
    
    
    let map = geomap#display_pixbuf ((0,256), tile.Gm.sw_corner) ((256,0),ne) (GdkPixbuf.from_file jpg_file) in
    map#raise 1;
    add_tile key
  

(** Filling the window with tiles *)
let fill_window = fun (geomap:MapCanvas.widget) ->
  let width_c, height_c = Gdk.Drawable.get_size geomap#canvas#misc#window
  and (xc0, yc0) = geomap#canvas#get_scroll_offsets in
  let (xw0, yw0) = geomap#window_to_world (float xc0) (float yc0)
  and (xw1, yw1) = geomap#window_to_world (float (xc0+width_c)) (float (yc0+height_c)) in
  let nw = geomap#of_world (xw0, yw0)
  and se = geomap#of_world (xw1, yw1) in

  (* Hypothesis: no strong variation of the height of the tiles on the whole area *)
  let (width_tile, height_tile) = Gm.tile_coverage se.LL.posn_lat 1 in
  for ilong = 0 to truncate ((se.LL.posn_long -. nw.LL.posn_long) /. width_tile) do
    let long = nw.LL.posn_long +. float ilong *. width_tile in
    for ilat = 0 to truncate ((nw.LL.posn_lat -. se.LL.posn_lat) /. height_tile) do
      let lat = nw.LL.posn_lat -. float ilat *. height_tile in
      let wgs84 = { LL.posn_lat = lat; posn_long = long } in
      try
	display_tile geomap wgs84
      with
	Gm.Not_available -> ()
    done
  done
  
