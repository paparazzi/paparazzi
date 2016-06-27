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

 open Printf

 module G = MapCanvas

 type shapes = {
   shpid : string;
   mutable circleshape : GnoCanvas.ellipse;
   mutable last_update : float
 }

let shapes = Hashtbl.create 1

let new_shape = fun id wgs84 fillcolor radius time (geomap:MapCanvas.widget) ->
  let gencircle = geomap#circle ~fill_color:fillcolor  ~color:fillcolor wgs84 radius in
  let shape = {shpid = id; circleshape = gencircle; last_update = time } in
  Hashtbl.add shapes id shape

let remove_shape = fun id ->
  try
   let shape = Hashtbl.find shapes id in
   shape.circleshape#destroy ();
   Hashtbl.remove shapes id
  with _ -> ()

let update_shape = fun id wgs84 fillcolor radius time (geomap:MapCanvas.widget) ->
  try
    let shape = Hashtbl.find shapes id in
    let gencircle = geomap#circle ~fill_color:fillcolor  ~color:fillcolor wgs84 radius in
    shape.circleshape#destroy ();
    shape.circleshape <- gencircle;
    shape.last_update <- time
  with _ -> ()

let shape_exist = fun id ->
  Hashtbl.mem shapes id
