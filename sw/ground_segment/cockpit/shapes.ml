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
  shcolor : string;
  shtype : float;
  shstatus : float;
  shlatarr : float array;
  shlonarr : float array;
  shradius : float}

type circleshape = { mutable circsh : GnoCanvas.ellipse}
type polygonshape = { mutable polysh : GnoCanvas.polygon}

let circleshapes = Hashtbl.create 1
let polygonshapes = Hashtbl.create 1

let circle_exist = fun id ->
  Hashtbl.mem circleshapes id

let polygon_exist = fun id ->
  Hashtbl.mem polygonshapes id

let update_circle = fun id wgs84 fillcolor radius (geomap:MapCanvas.widget) ->
  try
    let gencircle = geomap#circle ~width:2 ~color:fillcolor wgs84 radius in
    if not (circle_exist id) then
    let circleshape = {circsh = gencircle } in
    Hashtbl.add circleshapes id circleshape;
    else
    let shape = Hashtbl.find circleshapes id in
    shape.circsh#destroy ();
    shape.circsh <- gencircle
  with _ -> ()

let update_polygon = fun id positionarr fillcolor (geomap:MapCanvas.widget) ->
  try
    let genpolygon = geomap#polygon ~width:2 ~color:fillcolor positionarr in
    if not (polygon_exist id) then
    let polygonshape = {polysh = genpolygon } in
    Hashtbl.add polygonshapes id polygonshape;
    else
    let shape = Hashtbl.find polygonshapes id in
    shape.polysh#destroy ();
    shape.polysh <- genpolygon
  with _ -> ()

let update_shape = fun raw geomap->
  let position = fun lat lon -> { posn_lat=(Deg>>Rad)lat; posn_long=(Deg>>Rad)lon } in
  let arrlen = Array.length raw.shlatarr in
  let positionarr = Array.make arrlen (position raw.shlatarr.(1) raw.shlonarr.(1))  in
  for i = 0 to arrlen - 1 do positionarr.(i) <- position raw.shlatarr.(i) raw.shlonarr.(i) done;
  try
    if raw.shtype = 0. then
    update_circle raw.shid positionarr.(0) raw.shcolor raw.shradius geomap;
    if raw.shtype = 1. then
    update_polygon raw.shid positionarr raw.shcolor geomap
  with _ -> ()

let del_shape = fun raw ->
  try
    if raw.shtype = 0. then
    let shape = Hashtbl.find circleshapes raw.shid in
    Hashtbl.remove circleshapes raw.shid;
    shape.circsh#destroy ();
    else if raw.shtype = 1. then
    let shape = Hashtbl.find polygonshapes raw.shid in
    Hashtbl.remove polygonshapes raw.shid;
    shape.polysh#destroy ()
  with _ -> ()

let new_shmsg = fun raw (geomap:MapCanvas.widget) ->
  if raw.shstatus = 0. then update_shape raw geomap;
  if raw.shstatus = 1. then del_shape raw
