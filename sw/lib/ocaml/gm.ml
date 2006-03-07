(*
 * $Id$
 *
 * Google Maps utilities
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

let (//) = Filename.concat

open Latlong
open Printf

let tile_size = 256, 256

let cache_path = ref "/var/tmp"

type tile_t = {
    key : string;
    sw_corner : Latlong.geographic;
    width : float; (* Longitude difference *)
    height : float (* Longitude difference *)
  }

let (/.=) r x = r := !r /. x
let (+.=) r x = r := !r +. x
let (-.=) r x = r := !r -. x

let inv_norm_lat = fun l -> Latlong.inv_mercator_lat (l *. pi)
let norm_lat = fun l -> Latlong.mercator_lat l /. pi

let tile_coverage = fun lat zoom ->
  let normed_size = 2. /. (2. ** (float (18-zoom))) in
  let normed_lat = norm_lat lat in
  let normed_lat' = normed_lat +. normed_size in
  let lat' = inv_norm_lat normed_lat' in
  (normed_size, lat' -. lat)


let gm_pos_and_scale = fun keyholeString tLat latHeight tLon lonWidth ->
  let bot_lat = inv_norm_lat tLat in
  let top_lat = inv_norm_lat (tLat +. latHeight) in
  let bottom_left = {posn_lat = bot_lat ; posn_long = tLon *. pi} in
  { key = keyholeString;
    sw_corner = bottom_left;
    width = lonWidth *. pi;
    height = top_lat -. bot_lat }


(** Returns a keyhole string for a longitude (x), latitude (y), and zoom 
   for Google Maps (http://www.ponies.me.uk/maps/GoogleTileUtils.java) *)
let tile_of_geo = fun wgs84 zoom ->
  let zoom = 18 - zoom in
  
  (* first convert the lat lon to transverse mercator coordintes.*)
  let lon = (Rad>>Deg)wgs84.posn_long in
  let lon = if lon > 180. then lon -. 180. else lon in
  let lon = lon /. 180. in

  (*  convert latitude to a range -1..+1 *)
  let lat = norm_lat wgs84.posn_lat in
(*** log (tan (pi/.4. +. 0.5*. wgs84.posn_lat)) /. pi in ***)

  let tLat = ref (-1.)
  and tLon      = ref (-1.)
  and lonWidth  = ref 2.
  and latHeight = ref 2. (** Always identical to lonWidth !!! *)
  and keyholeString = Buffer.create 3 in
  Buffer.add_char keyholeString 't';

  for i = 0 to zoom - 1 do
    lonWidth /.= 2.;
    latHeight /.= 2.;

    if !tLat +. !latHeight > lat then
      if ((!tLon +. !lonWidth) > lon) then begin
        Buffer.add_char keyholeString 't';
      end else begin
        tLon +.= !lonWidth;
        Buffer.add_char keyholeString 's';
      end
    else begin
      tLat +.= !latHeight;

      if ((!tLon +. !lonWidth) > lon) then begin
        Buffer.add_char keyholeString 'q';
      end
      else begin  
        tLon +.= !lonWidth;
        Buffer.add_char keyholeString 'r';
      end
    end
  done;
  gm_pos_and_scale (Buffer.contents keyholeString) !tLat !latHeight !tLon !lonWidth

let tile_of_key = fun keyholeStr ->
  assert(keyholeStr.[0] = 't');
  
  let lon  = ref (-180.)
  and lonWidth = ref 360. 
  and lat       = ref (-1.) 
  and latHeight = ref 2. in
  
  for i = 1 to String.length keyholeStr - 1  do
    lonWidth /.= 2.;
    latHeight /.= 2.;

    match keyholeStr.[i] with
      's' -> lon +.= !lonWidth
    | 'r' -> 
        lat +.= !latHeight;
        lon +.= !lonWidth
    | 'q' -> lat +.= !latHeight
    | 't' -> ()
    | _ -> invalid_arg ("gm_get_lat_long " ^ keyholeStr)
  done;

  gm_pos_and_scale keyholeStr !lat !latHeight (!lon/.180.) (!lonWidth/.180.)


let is_prefix = fun a b ->
  String.length b >= String.length a &&
  a = String.sub b 0 (String.length a)


let get_from_cache = fun f ->
  let files = Sys.readdir !cache_path in
  let rec loop = fun i ->
    if i < Array.length files then
      let fi = files.(i) in
      let fi_key = Filename.chop_extension fi in
      if try is_prefix fi_key f with _ -> false then begin
	(tile_of_key fi_key, !cache_path // fi)
      end else
	loop (i+1)
    else
      raise Not_found in
  loop 0


let google_maps_url = fun s -> 
  sprintf "http://kh1.google.com/kh?n=404&v=3&t=%s" s

exception Not_available

let no_http = ref false

let get_image = fun tile ->
  try get_from_cache tile.key with
    Not_found ->
      if !no_http then raise Not_available;
      let url = google_maps_url tile.key in
      let jpg_file = !cache_path // (tile.key ^ ".jpg") in
      try
	ignore (Http.file_of_url ~dest:jpg_file url);
	tile, jpg_file
      with
	Http.Failure _ -> raise Not_available


let rec get_tile = fun wgs84 zoom ->
  if zoom < 10 then
    let tile = tile_of_geo wgs84 zoom in
    try get_image tile with
      (** Error, let's try a lower zoom *)
      Not_available when not !no_http -> get_tile wgs84 (zoom+1)
  else
    raise Not_available


