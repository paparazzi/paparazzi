(*
 *  $Id$
 *
 * Hardware in the loop basic simulator (handling GPS, infrared and servos)
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

open Stdlib
open Geometry_2d

let float_attrib xml a = float_of_string (ExtXml.attrib xml a)

let wind = (0., 0.) (* m/s in local ref *)

(* Frequencies for perdiodic tasks are expressed in periods of 100Hz *)
let timebase = 10 (* ms *)
let ir_period = 5
let fm_period = 4


module type AIRCRAFT = 
  sig
    val init : int -> GPack.box -> unit
    val boot : unit -> unit
    val servos : us array -> unit
	(** Called once at init *)
	
    val infrared : float -> float -> unit
	(** [infrared ir_left ir_front] Called on timer *)
	
    val gps : Gps.state -> unit
	(** [gps state] Called on timer *)
  end


module type AIRCRAFT_ITL = functor (A : Data.MISSION) -> AIRCRAFT


let ac_name = ref ""


let common_options = []

module Make(AircraftItl : AIRCRAFT_ITL) = struct

  module A = struct
    let ac = Data.aircraft !ac_name
  end

  module Aircraft = AircraftItl(A)

  module FM = FlightModel.Make(A)

  let flight_plan = A.ac.Data.flight_plan

  let lat0 = rad_of_deg (float_attrib flight_plan "lat0")
  let lon0 = rad_of_deg (float_attrib flight_plan "lon0")
  let qfu = (float_attrib flight_plan "qfu")
  let alt0 = (float_attrib flight_plan "ground_alt")
(*
   let gust_dir = 0 and
   let gust_speed = 0

  let wind = fun ->  
   let wind_dir = wind_dir_adj#value in
   let wind_speed = wind_speed_adj#value in
   let gust_max = gust_max_adj#value in
   let gust_dir_fact = gust_dir_fact_adj#value in
   let gust_speed_fact = gust_dir_fact_adj#value in
   gust_speed = trim 0, gust_max (gust_speed + Random.float gust_dir_fact)
   gust_dir = gust_dirspeed + Random.float gu)

*)
  let main () =
    let window = GWindow.dialog ~title:("Aircraft "^ !ac_name) () in
    let quit = fun () -> GMain.Main.quit (); exit 0 in
    ignore (window#connect#destroy ~callback:quit);
    Srtm.add_path (Env.paparazzi_home ^ "/data/srtm");
    
    Aircraft.init A.ac.Data.id window#vbox;

    let gps_period = 25 in
    
    let compute_gps_state = Gps.state lat0 lon0 (alt0) in

    let utm0 = Latlong.utm_of Latlong.WGS84 {Latlong.posn_long = lon0; Latlong.posn_lat = lat0}  in

    let initial_state = FlightModel.init (pi/.2. -. qfu/.180.*.pi) in

    let state = ref initial_state in

    let reset = fun () -> state := initial_state in 

    let servos = Array.create FM.nb_servos 0 in

    Aircraft.servos servos;

    let north_label = GMisc.label ~text:"000" ()
    and east_label = GMisc.label ~text:"000" ()
    and alt_label = GMisc.label ~text:"000" () in
    let wind_dir_adj = GData.adjustment ~value:0. ~lower:(0.) ~upper:370. ~step_incr:1.0 () in
    let wind_speed_adj = GData.adjustment ~value:0. ~lower:(0.) ~upper:20. ~step_incr:0.1 () in
    let gust_norm_max_adj = GData.adjustment ~value:0. ~lower:(0.) ~upper:20. ~step_incr:0.1 () in
    let gust_norm_ch_fact_adj = GData.adjustment ~value:0. ~lower:(0.) ~upper:20. ~step_incr:0.1 () in
    let gust_dir_ch_fact_adj = GData.adjustment ~value:0. ~lower:(0.) ~upper:20. ~step_incr:0.1 () in
    let infrared_contrast_adj = GData.adjustment ~value:500. ~lower:(0.) ~upper:1010. ~step_incr:10. () in



    let half_aperture = (Latlong.pi /. 4.) in
    let last_gps_state = ref None in
    let run = ref false in
    let ir_srtm = ref false in

    let scheduler =
      let t = ref 0 in
      let f =
	fun () ->
	  incr t;
	  if !t mod fm_period = 0 then begin
	    FM.do_servos !state servos;
	    let wind_dir_rad = deg2rad wind_dir_adj#value in
	    let wind_angle_rad = heading_of_to_angle_rad wind_dir_rad in
	    let wind_speed_polar = {r2D = wind_speed_adj#value; theta2D = oposite_heading_rad wind_angle_rad} in
	    let wind_speed_cart = polar2cart wind_speed_polar in
	    FM.state_update !state ( wind_speed_cart.x2D, wind_speed_cart.y2D)
	  end;

	  if !t mod ir_period = 0 then begin
	    let phi = FlightModel.get_phi !state in
	    let horizon_distance = 1000. in
	    try
	      match !last_gps_state with
		None -> Printf.printf "gps state NONE \n%!";()
	      | Some gps_state ->
		  let delta_ir = 
		    if !ir_srtm then
		      let altitude = (int_of_float gps_state.Gps.alt) in
		      let horizon_right = Srtm.horizon_slope gps_state.Gps.wgs84 altitude (gps_state.Gps.course +. Latlong.pi /. 2.)  half_aperture horizon_distance in
		      let horizon_left = Srtm.horizon_slope gps_state.Gps.wgs84 altitude (gps_state.Gps.course -. Latlong.pi /. 2.) half_aperture horizon_distance in 
		  (***) Printf.printf "IR: %f-%f\n%!" horizon_right horizon_left;
		  (***) Printf.printf "alt: %d\n%!" altitude;
		      horizon_right -. horizon_left
		    else
		      0. in
		  let ir_left = ( (phi +. delta_ir ) *. infrared_contrast_adj#value )
		  and ir_front = 0. in
		  Aircraft.infrared ir_left ir_front
	    with
	      x -> Printf.printf "%s\n%!" (Printexc.to_string x)
		  
	  end;
	  
	  if !t mod gps_period = 0 then begin
	    let (x,y,z) = FlightModel.get_xyz !state in
	    east_label#set_text (Printf.sprintf "%.0f" x);
	    north_label#set_text (Printf.sprintf "%.0f" y);
	    alt_label#set_text (Printf.sprintf "%.0f" z);
	    let s = compute_gps_state (x,y,z) (FlightModel.get_time !state) in
	    last_gps_state := Some s;
	    Aircraft.gps s
	  end;
	  true in
      fun () -> ignore (GMain.Timeout.add 10 f) in


    let boot = fun () ->
      Aircraft.boot ();
      scheduler () in
    
    let take_off = fun () -> prerr_endline "takeoff"; FlightModel.set_air_speed !state FM.nominal_airspeed in 

    let hbox = GPack.hbox ~packing:window#vbox#pack () in
    let s = GButton.button ~label:"Boot" ~packing:(hbox#pack ~padding:5) () in
    ignore (s#connect#clicked ~callback:boot);
    let t = GButton.button ~label:"Launch" ~packing:hbox#pack () in
    ignore (t#connect#clicked ~callback:take_off);
    let ir_srtm_button = GButton.toggle_button ~label:"IR/srtm" ~packing:hbox#pack () in
    ignore (ir_srtm_button#connect#toggled (fun () -> ir_srtm := not !ir_srtm));

    let hbox = GPack.hbox ~packing:window#vbox#pack () in
    let l = fun s -> ignore(GMisc.label ~text:s ~packing:hbox#pack ()) in
    l "East:"; hbox#pack east_label#coerce;
    l " North:"; hbox#pack north_label#coerce;
    l " Height:"; hbox#pack alt_label#coerce;

    let hbox = GPack.hbox ~packing:window#vbox#pack () in
    ignore (GMisc.label ~text:"wind dir:" ~packing:hbox#pack ());
    ignore (GRange.scale `HORIZONTAL ~adjustment:wind_dir_adj ~packing:hbox#add ());

    let hbox = GPack.hbox ~packing:window#vbox#pack () in
    ignore (GMisc.label ~text:"wind speed:" ~packing:hbox#pack ());
    ignore (GRange.scale `HORIZONTAL ~adjustment:wind_speed_adj ~packing:hbox#add ());

    let hbox = GPack.hbox ~packing:window#vbox#pack () in
    ignore (GMisc.label ~text:"gust max speed:" ~packing:hbox#pack ());
    ignore (GRange.scale `HORIZONTAL ~adjustment:gust_norm_max_adj ~packing:hbox#add ());


    let hbox = GPack.hbox ~packing:window#vbox#pack () in
    ignore (GMisc.label ~text:"infrared:" ~packing:hbox#pack ());
    ignore (GRange.scale `HORIZONTAL ~adjustment:infrared_contrast_adj ~packing:hbox#add ());


    window#show ();
    Unix.handle_unix_error GMain.Main.main ()
end
