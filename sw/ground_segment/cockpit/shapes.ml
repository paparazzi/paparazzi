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

open Latlong

type shstatus = Update
              | Delete

type shtype = Circle
            | Polygon
            | Segment
            | Text

type shdata = {
  shid : int;
  shlinecolor : string;
  shfillcolor : string;
  shtype : shtype;
  shstatus : shstatus;
  shlatarr : float array;
  shlonarr : float array;
  shradius : float;
  shtext : string;
  shopacity : int}

let int2shtype = fun i ->
  match i with
    | 0 -> Circle
    | 1 -> Polygon
    | 2 -> Segment
    | 3 -> Text
    | _ -> Text

let int2shstatus = fun i ->
  match i with
    | 0 -> Update
    | 1 -> Delete
    | _ -> Delete


let circleshapes = Hashtbl.create 1
let polygonshapes = Hashtbl.create 1
let lineshapes = Hashtbl.create 1
let textshapes = Hashtbl.create 1

let circle_exist = fun id ->
  Hashtbl.mem circleshapes id

let polygon_exist = fun id ->
  Hashtbl.mem polygonshapes id

let line_exist = fun id ->
  Hashtbl.mem lineshapes id

let text_exist = fun id ->
  Hashtbl.mem textshapes id

let update_circle = fun id wgs84 opacity fill_color color radius (geomap:MapCanvas.widget) ->
  try
    let gencircle = geomap#circle ~group:geomap#background ~width:2 ~fill_color ~opacity ~color wgs84.(0) radius in
    if (circle_exist id) then
      let shape = Hashtbl.find circleshapes id in
      shape#destroy ();
      Hashtbl.add circleshapes id gencircle;
    else
      Hashtbl.add circleshapes id gencircle;
  with _ -> ()

let update_polygon = fun id positionarr opacity fill_color color (geomap:MapCanvas.widget) ->
  try
    let genpolygon = geomap#polygon ~group:geomap#background ~width:2 ~fill_color ~opacity ~color  positionarr in
    if (polygon_exist id) then
      let shape = Hashtbl.find polygonshapes id in
      shape#destroy ();
      Hashtbl.add polygonshapes id genpolygon;
    else
      Hashtbl.add polygonshapes id genpolygon
  with _ -> ()

let update_line = fun id positionarr color (geomap:MapCanvas.widget) ->
  try
    let genline = geomap#segment ~group:geomap#background ~width:2 ~fill_color:color positionarr.(0) positionarr.(1) in
    if (line_exist id) then
      let shape = Hashtbl.find lineshapes id in
      shape#destroy ();
      Hashtbl.add lineshapes id genline;
    else
      Hashtbl.add lineshapes id genline
  with _ -> ()

let update_text = fun id positionarr color text (geomap:MapCanvas.widget)->
  try
    let gentext = geomap#text ~group:geomap#background ~fill_color:color positionarr.(0) text in
    if (text_exist id) then
      let shape = Hashtbl.find textshapes id in
      shape#destroy ();
      Hashtbl.add textshapes id gentext;
    else
      Hashtbl.add textshapes id gentext
  with _ -> ()

let convert_to_positions = fun raw ->
  let position = fun lat lon -> { posn_lat=(Deg>>Rad)lat; posn_long=(Deg>>Rad)lon } in
  let arrlen = Array.length raw.shlatarr in
  let positionarr = Array.make arrlen (position raw.shlatarr.(1) raw.shlonarr.(1))  in
  for i = 0 to arrlen - 1 do positionarr.(i) <- position raw.shlatarr.(i) raw.shlonarr.(i) done;
  positionarr

let del_text = fun raw ->
  try
    let shape = Hashtbl.find textshapes (raw.shid, raw.shtype) in
    Hashtbl.remove textshapes (raw.shid, raw.shtype);
    shape#destroy ()
  with _ -> ()

let update_shape = fun raw positions geomap ->
  try
    if raw.shtext = "NULL" then del_text raw else update_text (raw.shid, raw.shtype) positions raw.shlinecolor raw.shtext geomap;
    match raw.shtype with
      | Circle -> update_circle raw.shid positions raw.shopacity raw.shfillcolor raw.shlinecolor raw.shradius geomap;
      | Polygon -> update_polygon raw.shid positions raw.shopacity raw.shfillcolor raw.shlinecolor geomap;
      | Segment -> update_line raw.shid positions raw.shlinecolor geomap;
      | Text -> update_text (raw.shid, raw.shtype) positions raw.shlinecolor raw.shtext geomap;
  with _ -> ()


let del_shape = fun raw ->
  try
    del_text raw;
    match raw.shtype with
      | Circle ->
        let shape = Hashtbl.find circleshapes raw.shid in
        Hashtbl.remove circleshapes raw.shid;
        shape#destroy ()
      | Polygon ->
        let shape = Hashtbl.find polygonshapes raw.shid in
        Hashtbl.remove polygonshapes raw.shid;
        shape#destroy ()
      | Segment ->
        let shape = Hashtbl.find lineshapes raw.shid in
        Hashtbl.remove lineshapes raw.shid;
        shape#destroy ()
      | Text -> ()
  with _ -> ()

let new_shmsg = fun raw (geomap:MapCanvas.widget) ->
  match raw.shstatus with
    | Update -> update_shape raw (convert_to_positions raw) geomap
    | Delete -> del_shape raw
