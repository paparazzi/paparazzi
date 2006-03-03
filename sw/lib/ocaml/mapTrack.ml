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

let cam_half_aperture = m_pi /. 6.0
let half_pi = m_pi /. 2.0
let sqrt_2_div_2 = sqrt 2.0


class track = fun ?(name="coucou") ?(size = 500) ?(color="red") (geomap:MapCanvas.widget) (vertical_display:MapCanvas.basic_widget) ->
  let group = GnoCanvas.group geomap#canvas#root in
  let v_group = GnoCanvas.group vertical_display#canvas#root in
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
  
(** rectangle representing the field covered by the cam *)
  let _ac_cam_targeted =
    ignore ( GnoCanvas.ellipse ~x1: (-. 2.5) ~y1: (-. 2.5 ) ~x2: 2.5 ~y2: 2.5 ~fill_color:color ~props:[`WIDTH_UNITS 1.; `OUTLINE_COLOR color; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] cam) in
  
  let mission_target = GnoCanvas.group group in
  
(** red circle : target of the mission *)
  let ac_mission_target =
    ignore ( GnoCanvas.ellipse ~x1: (-5.) ~y1: (-5.) ~x2: 5. ~y2: 5. ~fill_color:"red" ~props:[`WIDTH_UNITS 1.; `OUTLINE_COLOR "red"; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] mission_target ) in
  
 (** data at map scale *)
  let max_cam_half_height_scaled = 10000.0 /. (geomap#get_world_unit ()) in
  let max_oblic_distance_scaled = 10000.0 /. (geomap#get_world_unit ()) in
  let min_distance_scaled = 10. /. (geomap#get_world_unit ())   in
  let min_height_scaled = 0.1 /. (geomap#get_world_unit ()) in


(** vertical display items *)

 let vertical_group = GnoCanvas.group vertical_display#canvas#root in
 let vertical_aircraft = GnoCanvas.group vertical_group in
 let vertical_plot =
    ignore ( GnoCanvas.ellipse ~x1: (-. 5.0) ~y1: (-. 5.0) ~x2: 5.0 ~y2: 5.0 ~fill_color:color ~props:[`WIDTH_UNITS 1.; `OUTLINE_COLOR color; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] vertical_aircraft);
  (*** ignore (GnoCanvas.line ~fill_color:color ~props:[`WIDTH_PIXELS 2;`CAP_STYLE `ROUND] ~points:[|0.;0.;0.; -10.|] vertical_aircraft); ***)
 in
let ac_v_label =
  GnoCanvas.text v_group ~props:[`TEXT name; `X 25.; `Y 25.; `ANCHOR `SW; `FILL_COLOR color]
 in

 let top = ref 0 
 and v_top = ref 0 in
  object (self)
    val mutable color = color
    val mutable segments = Array.create size empty
    val mutable v_segments = Array.create size empty
    val mutable last = None
    val mutable v_last = None 
    val mutable last_heading = 0.0
    val mutable last_altitude = 0.0
    val mutable last_speed = 0.0
    val mutable last_climb = 0.0
    val mutable last_height = 0.0
    val mutable last_xw = 0.0
    val mutable last_yw = 0.0
    val mutable last_flight_time = 0.0
    val mutable last_x_val = 0.0
    val mutable cam_on = false
    val mutable vertical_time_axis_on = false
    val mutable params_on = false
    val mutable v_params_on = false
    val mutable desired_track =  ((GnoCanvas.ellipse group) :> GnoCanvas.base_item)
    val mutable ac_cam_cover = GnoCanvas.rect cam
    method color = color
    method set_color c = color <- c
    method track = track
    method aircraft = aircraft
    method set_label = fun s -> ac_label#set [`TEXT s]
    method clear_one = fun i seg ->
      if seg.(i) != empty then begin
	(snd seg.(i))#destroy ();
	seg.(i) <- empty
      end
    method incr = fun seg ->
      let s = Array.length seg in
      top := (!top + 1) mod s
    method clear = fun seg top ->
      for i = 0 to Array.length seg - 1 do
	self#clear_one i seg
      done;
      top := 0
    method set_cam_state = fun b -> cam_on <- b

    (** switches time and longitude on the vertical display x axis.
        tracks are cleared *)
    method set_vertical_time_axis = fun b ->
      vertical_time_axis_on <- b;
      if vertical_time_axis_on then vertical_display#set_lbl_x_axis "x-axis: time"
	  else vertical_display#set_lbl_x_axis "x-axis: longitude";
      self#clear v_segments v_top;
      v_last <- None

    method update_ap_status = fun time -> 
      last_flight_time <- time
    method set_params_state = fun b -> params_on <- b 
    method set_v_params_state = fun b -> v_params_on <- b
    method set_last = fun x -> last <- x
    method set_v_last = fun x -> v_last <- x

    (** add track points on map2D or vertical display, according to the
       track parameter *)
    method add_point = fun en seg set_last_point last_point top track ->
      self#clear_one (!top) seg ;
      begin
	match last_point with
	  None -> 
	    seg.((!top)) <- (en, geomap#segment ~group:track ~fill_color:color en en)
	| Some pt ->
	    seg.((!top)) <- (en, geomap#segment ~group:track ~width:2 ~fill_color:color pt en);
      end;
      self#incr seg;
      set_last_point (Some en)

    method clear_map2D = self#clear segments top 

    method move_icon = fun en heading altitude relief_height speed climb ->
      let (xw,yw) = geomap#world_of_en en in
      aircraft#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw heading);
      last_heading <- heading;
      last_yw <- yw;
      last_height <- (altitude -. relief_height);
      last_speed <- speed ;
      last_climb <- climb;
      
      if params_on then
	  ac_label#set [`TEXT ( name^" \n"^(string_of_float last_height)^" m\n"^(string_of_float last_speed)^" m/s\n" ); `Y 70. ] else
	ac_label#set [`TEXT name; `Y 25.];
      ac_label#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw 0.);

      let y_val = ((vertical_display#get_vertical_max_level -. altitude) *. (vertical_display#get_vertical_factor) /. ( vertical_display#get_world_unit ()) ) in
      let x_val = if vertical_time_axis_on then
	last_flight_time /. ( vertical_display#get_world_unit () )
      else xw in
      vertical_aircraft#affine_absolute (affine_pos_and_angle vertical_display#zoom_adj#value x_val y_val 0.0 );

      let v_en = { MapCanvas.east = x_val *. ( vertical_display#get_world_unit ()  ); MapCanvas.north = y_val *. ( -. vertical_display#get_world_unit () ) } in


      (** on the vertical_display, 
	 params displayed are different if we have time or longitude on x-axis *)

      if vertical_time_axis_on then 
	begin
	  self#add_point v_en v_segments (self#set_v_last) v_last v_top v_group;
	  if v_params_on then ac_v_label#set [`TEXT ( name^" \n alt: "^(string_of_float altitude)^" m\n"^" height: "^(string_of_float last_height)^" m\n flight_time: "^( sprintf"%.2f sec\n" last_flight_time)); `Y 70. ]
	  else 	ac_v_label#set [`TEXT name; `Y 25.]
	end
      else
	begin
	  if v_params_on then ac_v_label#set [`TEXT ( name^" \n alt: "^(string_of_float altitude)^" m\n"^" height: "^(string_of_float last_height)^" m\n long(utm_world): "^(sprintf"%.2f m\n" en.MapCanvas.east)); `Y 70. ] 
	  else 	ac_v_label#set [`TEXT name; `Y 25.]
	end;
      ac_v_label#affine_absolute (affine_pos_and_angle vertical_display#zoom_adj#value x_val y_val 0.);
      self#add_point en segments (self#set_last) last top group;
      last_altitude <- altitude;
      last_xw <- xw;
      last_x_val <- x_val;
    method move_carrot = fun en ->
      let (xw,yw) = geomap#world_of_en en in
      carrot#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value xw yw 0.);

(** draws the circular path to be followed by the aircraft in circle mode *)
    method draw_circle = fun en radius ->
      desired_track#destroy ();
      desired_track <- ((geomap#circle ~color:"green" en radius) :> GnoCanvas.base_item);

(** draws the linear path to be followed by the aircraft between two waypoints *)
    method draw_segment = fun en en2 ->
      desired_track#destroy ();
      desired_track <-  ((geomap#segment ~fill_color:"green" en en2) :> GnoCanvas.base_item)
	  
(** moves the rectangle representing the field covered by the camera *)
    method move_cam = fun en mission_target_en ->
      if not cam_on then
	cam#hide ()
      else
	let (xw,yw) = geomap#world_of_en en in 
	let (mission_target_xw, mission_target_yw) = geomap#world_of_en mission_target_en in
	let last_height_scaled = last_height /. (geomap#get_world_unit ()) in
	
(** all data are at map scale *)
	
	begin
	let pt1 = { x2D = last_xw; y2D = last_yw} in
	let pt2 = { x2D = xw ; y2D = yw } in
	
(** y axis is downwards so North vector is as follows: *)
	let vect_north = (vect_make  { x2D = 0.0 ; y2D = 0.0 } { x2D = 0.0 ; y2D = -1.0 } ) in
	let d = distance pt1 pt2 in
	begin
	  let cam_heading = 
	    if d > min_distance_scaled then
	      let cam_vect_normalized = (vect_normalize (vect_make pt1 pt2)) in
	      if (dot_product vect_north cam_vect_normalized) > 0.0 then
		norm_angle_360 ( rad2deg (asin (cross_product vect_north cam_vect_normalized)))
	      else norm_angle_360 ( rad2deg (m_pi -. asin (cross_product vect_north cam_vect_normalized)))
	    else last_heading in
	  let (angle_of_view, oblic_distance) = 
	    if last_height < min_height_scaled then 
	      (half_pi, max_oblic_distance_scaled)
	    else
	      let oav = atan ( d /. last_height_scaled) in
	      (oav, last_height_scaled /. (cos oav))
	  in
	  let alpha_1 = angle_of_view +. cam_half_aperture in
	  let alpha_2 = angle_of_view -. cam_half_aperture in
	  begin
	    let cam_field_half_height_1 =
	      if alpha_1 < half_pi then
		(tan alpha_1) *. last_height_scaled -. d
	      else max_cam_half_height_scaled in
	    let cam_field_half_height_2 = d -. (tan ( angle_of_view -. cam_half_aperture)) *. last_height_scaled in
	    let cam_field_half_width = ( tan (cam_half_aperture) ) *. oblic_distance in
	    begin
(***	      Printf.printf "dist %.2f aoview %.2f oblic_distance %.2f cfh1 %.2f cfh2 %.2f cfhw %.2f last_xw %.2f last_yw %.2f cam_heading %.2f \n%!" (d  *. (geomap#get_world_unit ()) ) angle_of_view (oblic_distance  *. (geomap#get_world_unit ()) ) (cam_field_half_height_1  *. (geomap#get_world_unit ()) ) (cam_field_half_height_2  *. (geomap#get_world_unit ()) ) (cam_field_half_width  *. (geomap#get_world_unit ()) ) last_xw last_yw cam_heading; ***)
	      
	      ac_cam_cover#destroy ();
	      ac_cam_cover <- GnoCanvas.rect ~x1:(-. cam_field_half_width) ~y1:(-. cam_field_half_height_1) ~x2:(cam_field_half_width) ~y2:(cam_field_half_height_2) ~fill_color:"grey" ~props:[`WIDTH_PIXELS 1 ; `OUTLINE_COLOR color; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")] cam
	    end
	  end;
	  cam#affine_absolute (affine_pos_and_angle 1.0 xw yw cam_heading);
	  mission_target#affine_absolute (affine_pos_and_angle geomap#zoom_adj#value mission_target_xw mission_target_yw 0.0);
	  cam#show ()
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
	let j = ref ((!top - m + size) mod size) in
	for i = 0 to m - 1 do
	  a.(i) <- segments.(!j);
	  j := (!j + 1) mod size
	done;
	for i = 1 to size - new_size do (* Never done if new_size > size *)
	  self#clear_one !j segments;
	  j := (!j + 1) mod size
	done;
	top := m mod new_size;
	segments <- a
    method size = Array.length segments
    initializer ignore(geomap#zoom_adj#connect#value_changed (fun () -> self#zoom geomap#zoom_adj#value))
end
