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

let array_forall = fun f a ->
  Array.fold_right (fun x r -> f x && r) a true

open Printf

module LL = Latlong

(** Quadtreee of displayed tiles *)
type tiles_tree =
    Empty
  | Tile
  | Node of tiles_tree array
let gm_tiles = Node (Array.make 4 Empty)

(** Google Maps paths in the quadtree are coded with q,r,s and t*)
let index_of = function
'q' -> 0 | 'r' -> 1 | 's' -> 2 | 't' -> 3
  | _ -> invalid_arg "index_of"
let char_of = function
0 -> 'q' | 1 -> 'r' | 2 -> 's' | 3 -> 't'
  | _ -> invalid_arg "char_of"

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
            let sons = Array.make 4 Empty in
            tree.(j) <- Node sons;
            loop (i+1) sons (index_of tile_key.[i])
        | Tile -> () (* Already there *)
        | Node sons ->
          loop (i+1) sons (index_of tile_key.[i])
    else
      tree.(j) <- Tile in
  loop 0 [|gm_tiles|] 0


let display_the_tile = fun (geomap:MapCanvas.widget) tile jpg_file level ->
  let south_lat = tile.Gm.sw_corner.LL.posn_lat
  and west_long = tile.Gm.sw_corner.LL.posn_long in
  let north_lat = south_lat +. tile.Gm.height
  and east_long = west_long +. tile.Gm.width in
  let ne = LL.make_geo north_lat east_long in

  let (tx, ty) = Gm.tile_size in
  try
    let pixbuf = GdkPixbuf.from_file jpg_file in
    ignore (GMain.Idle.add (fun () ->
      let map = geomap#display_pixbuf ((0,tx), tile.Gm.sw_corner) ((ty,0),ne) pixbuf ~level in
      map#raise 1;
      false));
    add_tile tile.Gm.key
  with
      GdkPixbuf.GdkPixbufError(_, msg) ->
        match GToolbox.question_box ~title:"Corrupted file" ~buttons:["Erase"; "Cancel"] (sprintf "%s. Erase ?" msg) with
            1 ->
              Sys.remove jpg_file
          | _ -> ()



(** Displaying the tile around the given point *)
let display_tile = fun (geomap:MapCanvas.widget) wgs84 level ->
  let desired_tile = Gm.tile_of_geo ~level wgs84 1 in

  let key = desired_tile.Gm.key in
  if not (mem_tile key) then
    let (tile, jpg_file) = Gm.get_image key in
    display_the_tile geomap tile jpg_file (String.length tile.Gm.key)


exception New_displayed of int
(** [New_displayed zoom] Raised when a new is loadded *)

let fill_window = fun (geomap:MapCanvas.widget) zoomlevel ->
  (** First estimate the coverage of the window *)
  let width_c, height_c = Gdk.Drawable.get_size geomap#canvas#misc#window
  and (xc0, yc0) = geomap#canvas#get_scroll_offsets in
  let (xw0, yw0) = geomap#window_to_world (float xc0) (float (yc0+height_c))
  and (xw1, yw1) = geomap#window_to_world (float (xc0+width_c)) (float yc0) in
  let sw = geomap#of_world (xw0, yw0)
  and ne = geomap#of_world (xw1, yw1) in
  let west = sw.LL.posn_long /. LL.pi
  and east = ne.LL.posn_long /. LL.pi
  and north = LL.mercator_lat ne.LL.posn_lat /. LL.pi
  and south = LL.mercator_lat sw.LL.posn_lat /. LL.pi in

  let east = if east < west then east +. 2. else east in

  (** Get Hashtbl from cache *)
  let tbl = Gm.get_hashtbl_of_cache () in

  (** Go through the quadtree and look for the holes *)
  let rec loop = fun twest tsouth tsize trees i zoom key ->
    (* Check for intersection *)
    if not (twest > east || (twest+.tsize < west && (east < 1. (* Standard case *) || twest+.2.>east (* Over 180° *))) || tsouth > north || tsouth+.tsize < south) then
      let tsize2 = tsize /. 2. in
      try
        match trees.(i) with
            Tile -> ()
          | Empty ->
            if zoom = 1 then
              let tile, image = Gm.get_image ~tbl key in
              let level = String.length tile.Gm.key in
              display_the_tile geomap tile image level;
              raise (New_displayed (zoomlevel+1-String.length tile.Gm.key))
            else begin
              trees.(i) <- Node (Array.make 4 Empty);
              loop twest tsouth tsize trees i zoom key
            end
          | Node sons ->
            let continue = fun j tw ts ->
              loop tw ts tsize2 sons j (zoom-1) (key^String.make 1 (char_of j)) in

            continue 0 twest (tsouth+.tsize2);
            continue 1 (twest+.tsize2) (tsouth+.tsize2);
            continue 2 (twest+.tsize2) tsouth;
            continue 3 twest tsouth;

            (* If the current node is complete, replace it by a Tile *)
            if array_forall (fun x -> x = Tile) sons then begin
              trees.(i) <- Tile
            end
      with
          New_displayed z when z = zoom ->
            trees.(i) <- Tile
        | Gm.Not_available -> () in
  loop (-1.) (-1.)  2. [|gm_tiles|] 0 zoomlevel "t"


exception To_copy of int * string

let gdk_pixbuf_safe_copy_area ~dest ~dest_x ~dest_y ~width ~height ~src_x ~src_y pixbuf =
  let dest_x, width, src_x =
    if dest_x < 0 then 0, width+dest_x, src_x-dest_x else dest_x, width, src_x in
  let dest_y, height, src_y =
    if dest_y < 0 then 0, height+dest_y, src_y-dest_y else dest_y, height, src_y in
  let width = min width (GdkPixbuf.get_width dest - dest_x)
  and height = min height (GdkPixbuf.get_height dest -dest_y) in
  GdkPixbuf.copy_area ~dest ~dest_x ~dest_y ~width ~height ~src_x ~src_y pixbuf

let pixbuf = fun sw ne zoomlevel->
  assert (sw.LL.posn_lat < ne.LL.posn_lat);
  assert (sw.LL.posn_long < ne.LL.posn_long);
  let west = sw.LL.posn_long /. LL.pi
  and east = ne.LL.posn_long /. LL.pi
  and north = LL.mercator_lat ne.LL.posn_lat /. LL.pi
  and south = LL.mercator_lat sw.LL.posn_lat /. LL.pi in

  let pixel_size = 1. /. (2. ** 16.) /. 256. in
  let width = truncate ((east -. west) /. pixel_size)
  and height = truncate ((north -. south) /. pixel_size) in
  let dest = GdkPixbuf.create ~width ~height () in
  let rec loop = fun twest tsouth tsize zoom key ->
    if not (twest > east || twest+.tsize < west || tsouth > north || tsouth+.tsize < south) then
      let tsize2 = tsize /. 2. in
      try
        if zoom = 1
        then
          let tile, image = Gm.get_image key in
          raise (To_copy (zoomlevel+1-String.length tile.Gm.key, image))
        else begin
          let continue = fun j tw ts ->
            loop tw ts tsize2 (zoom-1) (key^String.make 1 (char_of j)) in
          continue 0 twest (tsouth+.tsize2);
          continue 1 (twest+.tsize2) (tsouth+.tsize2);
          continue 2 (twest+.tsize2) tsouth;
          continue 3 twest tsouth;
        end
      with
          To_copy (z, image) when z = zoom ->
            let dest_x = truncate ((twest -. west) /. pixel_size)
            and dest_y = truncate ((north -. (tsouth+.tsize)) /. pixel_size) in
            let width = truncate (tsize /. pixel_size) in
            let src_x = 0
            and src_y = 0 in
            let pixbuf = GdkPixbuf.from_file image in
            gdk_pixbuf_safe_copy_area ~dest ~dest_x ~dest_y ~width ~height:width ~src_x ~src_y pixbuf
        | Gm.Not_available -> () in
  loop (-1.) (-1.)  2. zoomlevel "t";
  dest
