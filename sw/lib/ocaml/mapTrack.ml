(*
 * $Id$
 *
 * Track objects
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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

open Printf

module G = MapCanvas

let affine_pos_and_angle z xw yw angle =
  let rad_angle = angle /. 180. *. acos (-1.) in
  let cos_a = cos rad_angle in
  let sin_a = sin rad_angle in
  [| cos_a /. z ; sin_a /. z ; ~-. sin_a /. z; cos_a /. z; xw ; yw |]

class track = fun ?(name="coucou") ?(size = 50) ?(color="red") (geomap:MapCanvas.widget) ->
  let group = GnoCanvas.group geomap#canvas#root in
  let empty = ({ G.east = 0.; north = 0. },  GnoCanvas.line group) in

  let aircraft = GnoCanvas.group group in
  let _ac_icon =
    ignore (GnoCanvas.line ~fill_color:color ~props:[`WIDTH_PIXELS 4;`CAP_STYLE `ROUND] ~points:[|0.;-6.;0.;14.|] aircraft);
    ignore (GnoCanvas.line ~fill_color:color ~props:[`WIDTH_PIXELS 4;`CAP_STYLE `ROUND] ~points:[|-9.;0.;9.;0.|] aircraft);
    ignore (GnoCanvas.line ~fill_color:color ~props:[`WIDTH_PIXELS 4;`CAP_STYLE `ROUND] ~points:[|-4.;10.;4.;10.|] aircraft) in
  let ac_label =
    GnoCanvas.text group ~props:[`TEXT name; `X 25.; `Y 25.; `ANCHOR `SW; `FILL_COLOR color] in
  
  object (self)
    val mutable segments = Array.create size empty
    val mutable top = 0
    val mutable last = None
    method clear_one = fun i ->
      if segments.(i) != empty then begin
	(snd segments.(i))#destroy ();
	segments.(i) <- empty
      end
    method incr =
      let s = Array.length segments in
      top <- (top + 1) mod s
    method clear =
      for i = 0 to Array.length segments - 1 do
	self#clear_one i
      done;
      top <- 0
    method add_point = fun en ->
      self#clear_one top;
      begin
	match last with
	  None -> 
	    segments.(top) <- (en, geomap#segment ~fill_color:color en en)
	| Some last ->
	    segments.(top) <- (en, geomap#segment ~width:2 ~fill_color:color last en)
      end;
      self#incr;
      last <- Some en
    method move_icon = fun en heading ->
      let (xw,yw) = geomap#world_of_en en in
      aircraft#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw heading);
      ac_label#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw 0.);
    method zoom = fun z ->
      let a = aircraft#i2w_affine in
      let z' = sqrt (a.(0)*.a.(0)+.a.(1)*.a.(1)) in
      for i = 0 to 3 do a.(i) <- a.(i) /. z' *. 1./.z done;
      aircraft#affine_absolute a	
    method resize =  fun new_size ->
      let a = Array.create new_size empty in
      let size =  Array.length segments in
      let m = min new_size size in
      let j = ref ((top - m + size) mod size) in
      for i = 0 to m - 1 do
	a.(i) <- segments.(!j);
	j := (!j + 1) mod size
      done;
      for i = 1 to size - new_size do (* Never done if new_size > size *)
	self#clear_one !j;
	j := (!j + 1) mod size
      done;
      top <- m mod new_size;
      segments <- a
    method size = Array.length segments
    initializer ignore(geomap#zoom_adj#connect#value_changed (fun () -> self#zoom geomap#zoom_adj#value))
end
