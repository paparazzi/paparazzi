(*
 * $Id$
 *
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

open Latlong

type error = string
exception Tile_not_found of string

let srtm_url = "ftp://e0dps01u.ecs.nasa.gov/srtm"

let error = fun string ->
  Printf.sprintf "wget %s/Eurasia/%s.hgt.zip" srtm_url string

let tile_size = 1201

(* Previously opened tiles *)
let htiles = Hashtbl.create 13

(* Path to data files *)
let path = ref ["."]

let add_path = fun p -> path := p :: !path

let open_compressed = fun f ->
  Ocaml_tools.open_compress (Ocaml_tools.find_file !path f)
 
let find tile =
  if not (Hashtbl.mem htiles tile) then begin
    try
      let f = open_compressed (tile^".hgt") in
      let n = tile_size*tile_size*2 in
      let buf = String.create n in
      really_input f buf 0 n;
      Hashtbl.add htiles tile buf
    with Not_found ->
      raise (Tile_not_found tile)
  end;
  Hashtbl.find htiles tile 
  
let get = fun tile y x ->
    let tile = find tile in
    let pos0 = (2*((tile_size-y)*tile_size+x)) in
    let rec skip_bad = fun pos ->
      let a = (Char.code tile.[pos] lsl 8) lor Char.code tile.[pos+1] in
      if a > 8848 || a < 0 then skip_bad (pos+2) else a in
    skip_bad pos0

let of_wgs84 = fun geo ->
  let lat = (Rad>>Deg)geo.posn_lat
  and long = (Rad>>Deg)geo.posn_long in
  let bottom = floor lat and left = floor long in
  let tile =
    Printf.sprintf "%c%.0f%c%03.0f" (if lat > 0. then 'N' else 'S') bottom (if long > 0. then 'E' else 'W') (abs_float left) in

  get tile (truncate ((lat-.bottom)*.1200.+.0.5)) (truncate ((long-.left)*.1200.+.0.5))

let of_utm = fun utm ->
  of_wgs84 (Latlong.of_utm WGS84 utm)

