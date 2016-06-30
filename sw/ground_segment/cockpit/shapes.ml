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
  shlat1 : float;
  shlon1 : float;
  shlat2 : float;
  shlon2 : float;
  shlat3 : float;
  shlon3 : float;
  shlat4 : float;
  shlon4 : float;
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

let update_polygon = fun id wgs84_1 wgs84_2 wgs84_3 wgs84_4 fillcolor (geomap:MapCanvas.widget) ->
  try
    let genpolygon = geomap#polygon ~width:2 ~color:fillcolor wgs84_1 wgs84_2 wgs84_3 wgs84_4 in
    if not (polygon_exist id) then
    let polygonshape = {polysh = genpolygon } in
    Hashtbl.add polygonshapes id polygonshape;
    else
    let shape = Hashtbl.find polygonshapes id in
    shape.polysh#destroy ();
    shape.polysh <- genpolygon
  with _ -> ()

let update_shape = fun raw geomap->
  try
    let pos1 = { posn_lat=(Deg>>Rad)raw.shlat1; posn_long=(Deg>>Rad)raw.shlon1 } in
    if raw.shtype = 0. then
    update_circle raw.shid pos1 raw.shcolor raw.shradius geomap;
    if raw.shtype = 1. then
    let pos2 = { posn_lat=(Deg>>Rad)raw.shlat2; posn_long=(Deg>>Rad)raw.shlon2 } in
    let pos3 = { posn_lat=(Deg>>Rad)raw.shlat3; posn_long=(Deg>>Rad)raw.shlon3 } in
    let pos4 = { posn_lat=(Deg>>Rad)raw.shlat4; posn_long=(Deg>>Rad)raw.shlon4 } in
    update_polygon raw.shid pos1 pos2 pos3 pos4 raw.shcolor geomap
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
