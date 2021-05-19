(*
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
let zoom_max = 22
let zoom_min = 18

let cache_path = ref "/var/tmp"

type tile_t = {
  key : string;
  sw_corner : Latlong.geographic;
  width : float; (* Longitude difference *)
  height : float (* Latitude difference *)
}

type maps_source = Google | OSM | MS | MQ | MQ_Aerial
let maps_sources = [Google; OSM; MS; MQ; MQ_Aerial]
let string_of_maps_source = function
Google -> "Google" | OSM -> "OpenStreetMap" | MS -> "Bing" | MQ -> "MapQuest OSM" | MQ_Aerial -> "MapQuest Open Aerial"

let maps_source = ref Google
let set_maps_source = fun s -> maps_source := s
let get_maps_source = fun () -> !maps_source

let mkdir = fun d ->
  if not (Sys.file_exists d) then
    Unix.mkdir d 0o755

let (/.=) r x = r := !r /. x
let (+.=) r x = r := !r +. x

let inv_norm_lat = fun l -> Latlong.inv_mercator_lat (l *. pi)
let norm_lat = fun l -> Latlong.mercator_lat l /. pi

let tile_coverage = fun lat zoom ->
  let normed_size = 2. /. (2. ** (float (zoom_max-zoom))) in
  let normed_lat = norm_lat lat in
  let normed_lat' = normed_lat +. normed_size in
  let lat' = inv_norm_lat normed_lat' in
  (normed_size, lat' -. lat)


let gm_pos_and_scale = fun keyholeString tLat latHeight tLon lonWidth ->
  let bot_lat = inv_norm_lat tLat in
  let top_lat = inv_norm_lat (tLat +. latHeight) in
  let bottom_left = make_geo bot_lat (tLon *. pi) in
  { key = keyholeString;
    sw_corner = bottom_left;
    width = lonWidth *. pi;
    height = top_lat -. bot_lat }


(** Returns a keyhole string for a longitude (x), latitude (y), and zoom
    for Google Maps (http://www.ponies.me.uk/maps/GoogleTileUtils.java) *)
let tile_of_geo = fun ?level wgs84 zoom ->
  let max = match level with
    | None -> zoom_max
    | Some l -> if l < zoom_min then zoom_min else if l > zoom_max then zoom_max else l
  in
  let zoom = max - zoom in

  (* first convert the lat lon to transverse mercator coordinates *)
  let lon = (Rad>>Deg)wgs84.posn_long in
  let lon = if lon > 180. then lon -. 180. else lon in
  let lon = lon /. 180. in

  (* convert latitude to a range -1..+1 *)
  let lat = norm_lat wgs84.posn_lat in

  let tLat = ref (-1.)
  and tLon      = ref (-1.)
  and latLonSize  = ref 2.
  and keyholeString = Buffer.create 3 in
  Buffer.add_char keyholeString 't';

  for i = 0 to zoom - 1 do
    latLonSize /.= 2.;

    if !tLat +. !latLonSize > lat then
      if ((!tLon +. !latLonSize) > lon) then begin
        Buffer.add_char keyholeString 't';
      end else begin
        tLon +.= !latLonSize;
        Buffer.add_char keyholeString 's';
      end
    else begin
      tLat +.= !latLonSize;

      if ((!tLon +. !latLonSize) > lon) then begin
        Buffer.add_char keyholeString 'q';
      end
      else begin
        tLon +.= !latLonSize;
        Buffer.add_char keyholeString 'r';
      end
    end
  done;
  gm_pos_and_scale (Buffer.contents keyholeString) !tLat !latLonSize !tLon !latLonSize

let tile_of_key = fun keyholeStr ->
  assert(keyholeStr.[0] = 't');

  let lon  = ref (-1.)
  and lat       = ref (-1.)
  and latLonSize = ref 2. in

  for i = 1 to String.length keyholeStr - 1  do
    latLonSize /.= 2.;

    match keyholeStr.[i] with
        's' -> lon +.= !latLonSize
      | 'r' ->
        lat +.= !latLonSize;
        lon +.= !latLonSize
      | 'q' -> lat +.= !latLonSize
      | 't' -> ()
      | _ -> invalid_arg ("gm_get_lat_long " ^ keyholeStr)
  done;

  gm_pos_and_scale keyholeStr !lat !latLonSize !lon !latLonSize


let is_prefix = fun a b ->
  String.length b >= String.length a &&
    a = String.sub b 0 (String.length a)

(** Get the tile or one which contains it from the cache *)
let get_from_cache = fun dir f ->
  let files = Sys.readdir dir in
  (* sort files to have the longest names first *)
  Array.sort (fun a b -> String.length b - String.length a) files;
  let rec loop = fun i ->
    if i < Array.length files then
      let fi = files.(i) in
      let fi_key = try Filename.chop_extension fi with _ -> fi in
      (* is it a valid substring ? *)
      if fi_key <> "" && is_prefix fi_key f then
        (tile_of_key fi_key, dir // fi)
      else
        loop (i+1)
    else
      raise Not_found
  in
  loop 0

(** Get the tile or one which contains it from the a hash table *)
let get_from_hashtbl = fun tbl key ->
  let l = String.length key in
  let rec loop = fun i ->
    if i = 0 then raise Not_found;
    try
      let subkey = String.sub key 0 i in
      let file = Hashtbl.find tbl subkey in
      (tile_of_key subkey, file)
    with _ -> loop (i-1)
  in
  loop l

(** Translate the old quadtree naming policy into new (x,y) coordinates
    if z is the zoom level, 0 <= x, y < 2^z are the coordinates of the tile *)
let xyz_of_qsrt = fun s ->
  let x = ref 0
  and y = ref 0
  and n = String.length s in
  for i = 1 to n - 1 do (* Skip the first t *)
    x := !x * 2;
    y := !y * 2;
    match s.[i] with
        'q' -> ()
      | 'r' -> incr x
      | 's' -> incr x; incr y
      | 't' -> incr y
      | _ -> failwith "xyz_of_qsrt"
  done;
  (!x, !y, n-1)

let ms_key = fun key ->
  let n = String.length key in
  if n = 1 then invalid_arg "Gm.ms_key";
  let ms_key = Bytes.create (n-1) in
  for i = 1 to n - 1 do
    Bytes.set ms_key (i-1)
      (match key.[i] with
          'q' -> '0'
        | 'r' -> '1'
        | 's' -> '3'
        | 't' -> '2'
        | _ -> invalid_arg "Gm.ms_key")
  done;
  let s = Bytes.to_string ms_key in
  (s, s.[n-2])

let url_of_tile_key = fun maps_source s ->
  let (x, y, z) = xyz_of_qsrt s in
  match maps_source with
      Google -> sprintf "http://mt1.google.com/vt/lyrs=s&x=%d&s=&y=%d&z=%d" x y z
    | OSM ->    sprintf "http://tile.openstreetmap.org/%d/%d/%d.png" z x y
    | MQ -> sprintf "http://otile1.mqcdn.com/tiles/1.0.0/osm/%d/%d/%d.png" z x y
    | MQ_Aerial -> sprintf "http://otile1.mqcdn.com/tiles/1.0.0/sat/%d/%d/%d.png" z x y
    | MS ->
      let (key, last_char) = ms_key s in
      (* That's the old naming scheme, that still works as of 1st August 2010
         sprintf "http://a0.ortho.tiles.virtualearth.net/tiles/a%s.jpeg?g=%d" key (z+32)
      *)
      (* That's the new code, which conforms to MS naming scheme as of 1st August 2010 *)
      sprintf "http://ecn.t%c.tiles.virtualearth.net/tiles/a%s.jpeg?g=516" last_char key
(**)


let get_cache_dir = function
    Google -> !cache_path // "Google"
  | OSM -> !cache_path // "OSM"
  | MQ -> !cache_path // "MapQuest"
  | MQ_Aerial -> !cache_path // "MapQuestAerial"
  | MS -> !cache_path // "MS"


exception Not_available

type policy = CacheOrHttp | NoHttp | NoCache
let string_of_policy = function
    CacheOrHttp -> "CacheOrHttp"
  | NoHttp -> "NoHttp"
  | NoCache -> "NoCache"
let policies = [CacheOrHttp; NoHttp; NoCache]
let policy = ref CacheOrHttp
let set_policy = fun p ->
  policy := p
let get_policy = fun () ->
  !policy

let remove_last_char = fun s -> String.sub s 0 (String.length s - 1)


type hashtbl_cache = (string, string) Hashtbl.t

let get_hashtbl_of_cache = fun () ->
  let cache_dir = get_cache_dir !maps_source in
  mkdir cache_dir;
  let files = Sys.readdir cache_dir in
  let tbl = Hashtbl.create (Array.length files) in
  Array.iter (fun e ->
    let key = try Filename.chop_extension e with _ -> e in
    if key <> "" then Hashtbl.add tbl key (cache_dir // e);
  ) files;
  tbl

let get_image = fun ?tbl key ->
  let cache_dir = get_cache_dir !maps_source in
  mkdir cache_dir;
  let rec get_from_http = fun k ->
    if String.length k >= 1 then
      let url = url_of_tile_key !maps_source k in
      let jpg_file = cache_dir // (k ^ ".jpg") in
      try
        ignore (Http.file_of_url ~dest:jpg_file url);
        tile_of_key k, jpg_file
      with
          Http.Not_Found _ -> get_from_http (remove_last_char k)
        | Http.Blocked _ ->
          begin
            prerr_endline (Printf.sprintf "Seem to be temporarily blocked, '%s'" url);
            raise Not_available
          end
        | _ -> raise Not_available
    else
      raise Not_available
  in
  try
    if !policy = NoCache then raise Not_found;
    let (t, f) = match tbl with
      | None -> get_from_cache cache_dir key
      | Some ht -> get_from_hashtbl ht key
    in
    (* if not exact match from cache, try http if CacheOrHttp policy *)
    if !policy = CacheOrHttp && (String.length t.key < String.length key) then
      try get_from_http key with _ -> (t, f)
    else (t, f)
  with
    | Not_found ->
      if !policy = NoHttp then raise Not_available;
      get_from_http key


let rec get_tile = fun wgs84 zoom ->
  let tile = tile_of_geo wgs84 zoom in
  get_image tile.key
