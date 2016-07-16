(*
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 *)

 module G = MapCanvas
 open Latlong
 module LL = Latlong
 open Printf


type shdata = {
  shid : string;
  shlinecolor : string;
  shfillcolor : string;
  shtype : float;
  shstatus : float;
  shlatarr : float array;
  shlonarr : float array;
  shradius : float}

let circleshapes = Hashtbl.create 1
let polygonshapes = Hashtbl.create 1
let lineshapes = Hashtbl.create 1

let circle_exist = fun id ->
  Hashtbl.mem circleshapes id

let polygon_exist = fun id ->
  Hashtbl.mem polygonshapes id

let line_exist = fun id ->
  Hashtbl.mem lineshapes id

let update_circle = fun id wgs84 color ?fill_color radius (geomap:MapCanvas.widget) ->
  try
    let gencircle = geomap#circle ~width:2 ?fill_color ~color:color wgs84.(0) radius in
    if (circle_exist id) then
    let shape = Hashtbl.find circleshapes id in
    shape#destroy ();
    Hashtbl.add circleshapes id gencircle;
    else
    Hashtbl.add circleshapes id gencircle
  with _ -> ()

let update_polygon = fun id positionarr color ?fill_color (geomap:MapCanvas.widget) ->
  try
    let genpolygon = geomap#polygon ~width:2 ?fill_color ~color:color  positionarr in
    if (polygon_exist id) then
    let shape = Hashtbl.find polygonshapes id in
    shape#destroy ();
    Hashtbl.add polygonshapes id genpolygon;
    else
    Hashtbl.add polygonshapes id genpolygon
  with _ -> ()

let update_line = fun id positionarr color (geomap:MapCanvas.widget) ->
  try
    let genline = geomap#segment ~width:2 ~fill_color:color  positionarr.(0) positionarr.(1) in
    if (line_exist id) then
    let shape = Hashtbl.find lineshapes id in
    shape#destroy ();
    Hashtbl.add lineshapes id genline;
    else
    Hashtbl.add lineshapes id genline
  with _ -> ()

let convert_to_positions = fun raw ->
  let position = fun lat lon -> { posn_lat=(Deg>>Rad)lat; posn_long=(Deg>>Rad)lon } in
  let arrlen = Array.length raw.shlatarr in
  let positionarr = Array.make arrlen (position raw.shlatarr.(1) raw.shlonarr.(1))  in
  for i = 0 to arrlen - 1 do positionarr.(i) <- position raw.shlatarr.(i) raw.shlonarr.(i) done;
  positionarr

let update_shape = fun raw positions ?fill_color geomap ->
  try
    if raw.shtype = 0. then
    update_circle raw.shid positions ?fill_color raw.shlinecolor raw.shradius geomap;
    if raw.shtype = 1. then
    update_polygon raw.shid positions ?fill_color raw.shlinecolor geomap;
    if raw.shtype = 2. then
    update_line raw.shid positions raw.shlinecolor geomap
  with _ -> ()

let del_shape = fun raw ->
  try
    if raw.shtype = 0. then
    let shape = Hashtbl.find circleshapes raw.shid in
    Hashtbl.remove circleshapes raw.shid;
    shape#destroy ();
    else if raw.shtype = 1. then
    let shape = Hashtbl.find polygonshapes raw.shid in
    Hashtbl.remove polygonshapes raw.shid;
    shape#destroy ();
    else if raw.shtype = 2. then
    let shape = Hashtbl.find lineshapes raw.shid in
    Hashtbl.remove lineshapes raw.shid;
    shape#destroy ()
  with _ -> ()

let new_shmsg = fun raw (geomap:MapCanvas.widget) ->
  let without_fill = (Compat.bytes_compare "None" raw.shfillcolor) = 0 in
  if raw.shstatus = 0. && without_fill then update_shape raw (convert_to_positions raw) geomap;
  if raw.shstatus = 0. && (not without_fill) then update_shape raw (convert_to_positions raw) ~fill_color:raw.shfillcolor geomap;
  if raw.shstatus = 1. then del_shape raw
