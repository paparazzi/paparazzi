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
open Geometry_2d

module G = MapCanvas

let affine_pos_and_angle z xw yw angle =
  let rad_angle = angle /. 180. *. acos(-1.) in
  let cos_a = cos rad_angle in
  let sin_a = sin rad_angle in
  [| cos_a /. z ; sin_a /. z ; ~-. sin_a /. z; cos_a /. z; xw ; yw |]

let rec norm_angle_360 = fun alpha ->
  if alpha > 360.0 then norm_angle_360 (alpha -. 360.0)
  else if alpha < 0.0 then norm_angle_360 (alpha +. 360.0)
  else alpha

(** for tests, coordinates of a fixed target: *)
 
let fixed_cam_targeted_xw = 500.0
let fixed_cam_targeted_yw = 500.0

(** variables used for handling cam moves: *)

let cam_half_aperture = m_pi /. 4.0
let cam_field_half_width = ref 0.0
let cam_field_half_height_1 = ref 0.0
let cam_field_half_height_2 = ref 0.0
let cam_heading = ref 0.0
let angle_of_view = ref 0.0
let oblic_distance = ref 0.0
let max_cam_half_height = 10000.0
let max_oblic_distance = 10000.0
let min_distance = 0.1
let min_height = 0.1

let half_pi = m_pi /. 2.0

class track = fun ?(name="coucou") ?(size = 50) ?(color="red") (geomap:MapCanvas.widget) ->
  let group = GnoCanvas.group geomap#canvas#root in
  let empty = ({ G.east = 0.; north = 0. },  GnoCanvas.line group) in

  let aircraft = GnoCanvas.group group
  and track = GnoCanvas.group group in
  let _ac_icon =
    ignore (GnoCanvas.line ~fill_color:color ~props:[`WIDTH_PIXELS 4;`CAP_STYLE `ROUND] ~points:[|0.;-6.;0.;14.|] aircraft);
    ignore (GnoCanvas.line ~fill_color:color ~props:[`WIDTH_PIXELS 4;`CAP_STYLE `ROUND] ~points:[|-9.;0.;9.;0.|] aircraft);
    ignore (GnoCanvas.line ~fill_color:color ~props:[`WIDTH_PIXELS 4;`CAP_STYLE `ROUND] ~points:[|-4.;10.;4.;10.|] aircraft) in
  let ac_label =
    GnoCanvas.text group ~props:[`TEXT name; `X 25.; `Y 25.; `ANCHOR `SW; `FILL_COLOR color] in
  let carrot = GnoCanvas.group group in
  let _ac_carrot =
    ignore (GnoCanvas.polygon ~points:[|0.;0.;-5.;-10.;5.;-10.|] ~props:[`WIDTH_UNITS 1.;`FILL_COLOR "orange"; `OUTLINE_COLOR "orange"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] carrot) in

  let cam = GnoCanvas.group group in
  let ac_cam_cover = ref ( GnoCanvas.rect ~x1:(-. !cam_field_half_width) ~y1: (-. !cam_field_half_height_2) ~x2:(!cam_field_half_width) ~y2:(!cam_field_half_height_1) ~fill_color:"none" ~props:[`WIDTH_UNITS 1.; `OUTLINE_COLOR "brown"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] cam) in

  let cam_targeted = GnoCanvas.group group in

(** rectangle representing the field covered by the cam *)
  let _ac_cam_targeted =
    ignore ( GnoCanvas.ellipse ~x1: (-. 2.5) ~y1: (-. 2.5) ~x2: 2.5 ~y2: 2.5 ~fill_color:"blue" ~props:[`WIDTH_UNITS 1.; `OUTLINE_COLOR "blue"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] cam_targeted ) in

  let mission_target = GnoCanvas.group group in

(** red circle : target of the mission *)
  let ac_mission_target =
    ignore ( GnoCanvas.ellipse ~x1: (-5.) ~y1: (-5.) ~x2: 5. ~y2: 5. ~fill_color:"red" ~props:[`WIDTH_UNITS 1.; `OUTLINE_COLOR "red"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] mission_target ) in 
  
  object (self)
    val mutable segments = Array.create size empty
    val mutable top = 0
    val mutable last = None
    val mutable last_heading = 0.0
    val mutable last_altitude = 0.0
    val mutable last_height = 0.0
    val mutable last_xw = 0.0
    val mutable last_yw = 0.0
    method track = track
    method aircraft = aircraft
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
	    segments.(top) <- (en, geomap#segment ~group:track ~fill_color:color en en)
	| Some last ->
	    segments.(top) <- (en, geomap#segment ~group:track ~width:2 ~fill_color:color last en)
      end;
      self#incr;
      last <- Some en
    method move_icon = fun en heading altitude relief_height ->
      let (xw,yw) = geomap#world_of_en en in
      aircraft#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw heading);
      last_heading <- heading;
      last_altitude <- altitude;
      last_xw <- xw;
      last_yw <- yw;
      last_height <- altitude -. relief_height;
      ac_label#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw 0.);
    method move_carrot = fun en ->
      let (xw,yw) = geomap#world_of_en en in
      carrot#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw 0.)


(** moves the rectangle representing the field covered by the camera *)
    method move_cam = fun en mission_target_en ->
      let (xw,yw) = geomap#world_of_en en in 
      let (mission_target_xw, mission_target_yw) = geomap#world_of_en mission_target_en in
      begin
	(!ac_cam_cover)#destroy ();
	let pt1 = { x2D = last_xw; y2D = last_yw} in
	let pt2 = { x2D = xw ; y2D = yw } in

	(* y axis is downwards so North vector is as follows *)

	let vect_north = (vect_make  { x2D = 0.0 ; y2D = 0.0 } { x2D = 0.0 ; y2D = -. 1.0 } ) in
	let d = distance pt1 pt2 in
	begin
	  if d > min_distance then
	    let cam_vect_normalized = (vect_normalize (vect_make pt1 pt2)) in
	      if (dot_product vect_north cam_vect_normalized) > 0.0 then
	    cam_heading := norm_angle_360 ( rad2deg (asin (cross_product vect_north cam_vect_normalized)))
	      else cam_heading := norm_angle_360 ( rad2deg (m_pi -. asin (cross_product vect_north cam_vect_normalized)))
	  else cam_heading := last_heading;
	  if last_height < min_height then 
	    begin
	      angle_of_view := half_pi;
	      oblic_distance := max_oblic_distance
	    end
	  else 
	    begin
	      angle_of_view := (atan ( d /. last_height) );
	      oblic_distance := last_height /. (cos !angle_of_view)
	    end;
	  let alpha_1 = !angle_of_view +. cam_half_aperture in
	  let alpha_2 = !angle_of_view -. cam_half_aperture in
	  begin 
	    if alpha_1 < half_pi then
	      cam_field_half_height_1 := (tan alpha_1) *. last_height -. d
	    else cam_field_half_height_1 := max_cam_half_height;
	    cam_field_half_height_2 := d -. (tan ( !angle_of_view -. cam_half_aperture)) *. last_height;
	    cam_field_half_width := ( tan (cam_half_aperture) ) *. !oblic_distance;
	      
(***	  Printf.printf "dist %.2f aoview %.2f oblic_distance %.2f cfh1 %.2f cfh2 %.2f cfhw %.2f last_xw %.2f last_yw %.2f cam_heading %.2f \n " d !angle_of_view !oblic_distance !cam_field_half_height_1 !cam_field_half_height_2 !cam_field_half_width last_xw last_yw !cam_heading; 
   flush stdout;   ***)
  
	    ac_cam_cover := GnoCanvas.rect ~x1:( !cam_field_half_width) ~y1:( !cam_field_half_height_2) ~x2:(-. !cam_field_half_width) ~y2:(-. !cam_field_half_height_1) ~fill_color:"grey" ~props:[`WIDTH_UNITS 0.5 ; `OUTLINE_COLOR "blue"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] cam;
	  end;
	  cam#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw !cam_heading);
	  cam_targeted#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw 0.0);
	  mission_target#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value mission_target_xw mission_target_yw 0.0)
	end;
      end

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
