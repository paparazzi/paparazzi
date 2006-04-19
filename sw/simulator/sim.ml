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

open Printf
open Stdlib

module Ground_Pprz = Pprz.Messages(struct let name = "ground" end)

let float_attrib xml a = float_of_string (ExtXml.attrib xml a)

(* Frequencies for perdiodic tasks are expressed in s *)
let ir_period = 1./.20.
let fm_period = 1./.25.
let fg_period = 1./.25.


module type AIRCRAFT = 
  sig
    val init : int -> GPack.box -> unit
    val boot : Stdlib.value -> unit
    (** [boot time_acceleration] *)

    val servos : us array -> unit
	(** Called once at init *)
	
    val infrared : float -> float -> float -> unit
	(** [infrared ir_left ir_front ir_top] Called on timer *)
	
    val gps : Gps.state -> unit
	(** [gps state] Called on timer *)
  end


module type AIRCRAFT_ITL = functor (A : Data.MISSION) -> AIRCRAFT

external fg_sizeof : unit -> int = "fg_sizeof"
external fg_msg : string -> float -> float -> float -> float -> float -> float -> unit = "fg_msg_bytecode" "fg_msg_native"


let ac_name = ref ""

let ivy_bus = ref "127.255.255.255:2010"

let fg_client = ref ""

let common_options = [
  "-b", Arg.Set_string ivy_bus, "Bus\tDefault is 127.255.255.25:2010";
  "-fg", Arg.Set_string fg_client, "Flight gear client address"
]

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

  let main () =
    let window = GWindow.window ~title:("Aircraft "^ !ac_name) () in
    let quit = fun () -> GMain.Main.quit (); exit 0 in
    ignore (window#connect#destroy ~callback:quit);
    let vbox = GPack.vbox ~packing:window#add () in
    Srtm.add_path (Env.paparazzi_home ^ "/data/srtm");
    
    Aircraft.init A.ac.Data.id vbox;

    let gps_period = 0.25 in
    
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

    let half_aperture = (Latlong.pi /. 4.) in
    let last_gps_state = ref None in
    let run = ref false in
    let ir_srtm = ref false in

    let wind_x = ref 0.
    and wind_y = ref 0. in
    let infrared_contrast = ref 266.
    and time_scale = object val mutable v = 1. method value = v method set_value x = v <- x end
    and gps_availability = ref 1 in

    let world_update = fun _ vs ->
      gps_availability := Pprz.int_assoc "gps_availability" vs;
      wind_x := Pprz.float_assoc "wind_east" vs;      
      wind_y := Pprz.float_assoc "wind_north" vs;      
      infrared_contrast := Pprz.float_assoc "ir_contrast" vs;
      time_scale#set_value (Pprz.float_assoc "time_scale" vs)
    in

    ignore (Ground_Pprz.message_bind "WORLD_ENV" world_update);


    let fm_task = fun () ->
      FM.do_servos !state servos;
      FM.state_update !state (!wind_x, !wind_y) fm_period
       
    and ir_task = fun () ->
      let phi = FlightModel.get_phi !state in
      let horizon_distance = 1000. in
      try
	match !last_gps_state with
	  None -> ()
	| Some gps_state ->
	    let delta_ir = 
	      if !ir_srtm then
		let altitude = (int_of_float gps_state.Gps.alt) in
		let horizon_right = Srtm.horizon_slope gps_state.Gps.wgs84 altitude (gps_state.Gps.course +. Latlong.pi /. 2.)  half_aperture horizon_distance in
		let horizon_left = Srtm.horizon_slope gps_state.Gps.wgs84 altitude (gps_state.Gps.course -. Latlong.pi /. 2.) half_aperture horizon_distance in
		horizon_right -. horizon_left
	      else
		0. in
	    let phi = phi +. FM.roll_neutral_default in
	    let ir_left = (phi +. delta_ir ) *. !infrared_contrast
	    and ir_front = 0.
	    and ir_top = pi /. 2. *. !infrared_contrast in
	    Aircraft.infrared ir_left ir_front ir_top
      with
	x -> Printf.printf "%s\n%!" (Printexc.to_string x)
	    
    and gps_task = fun () ->
      let (x,y,z) = FlightModel.get_xyz !state in
      east_label#set_text (Printf.sprintf "%.0f" x);
      north_label#set_text (Printf.sprintf "%.0f" y);
      alt_label#set_text (Printf.sprintf "%.0f" z);
      let s = compute_gps_state (x,y,z) (FlightModel.get_time !state) in
      s.Gps.availability <- not (!gps_availability = 0);
      last_gps_state := Some s;
      Aircraft.gps s in

    (** Sending to Flight Gear *)
    let fg_task = fun socket buffer () ->
      match !last_gps_state with
	None -> ()
      | Some s ->
	  let lat = s.Gps.wgs84.Latlong.posn_lat
	  and lon = s.Gps.wgs84.Latlong.posn_long
	  and alt = s.Gps.alt
(*	  and theta_ = s.Gps.course *)
	  and (phi, theta, psi) = FlightModel.get_attitude !state in
	  fg_msg buffer lat lon alt phi theta psi;
(**       for i = 0 to String.length buffer - 1 do fprintf stderr "%x " (Char.code buffer.[i]) done; fprintf stderr "\n"; **)
      try
	ignore (Unix.send socket buffer 0 (String.length buffer) [])
      with
	Unix.Unix_error (e,f,a) -> Printf.fprintf stderr "Error fg: %s (%s(%s))\n" (Unix.error_message e) f a
    in

    let boot = fun () ->
      Aircraft.boot (time_scale:>value);
      Stdlib.timer ~scale:time_scale fm_period fm_task;
      Stdlib.timer ~scale:time_scale ir_period ir_task;
      Stdlib.timer ~scale:time_scale gps_period gps_task;

      (** Connection to Flight Gear client *)
      if !fg_client <> "" then
	try
	  let inet_addr = Unix.inet_addr_of_string !fg_client in
	  let socket = Unix.socket Unix.PF_INET Unix.SOCK_DGRAM 0 in
	  Unix.connect socket (Unix.ADDR_INET (inet_addr, 5501));
	  let buffer = String.create (fg_sizeof ()) in
	  Stdlib.timer ~scale:time_scale fg_period (fg_task socket buffer)
	with
	  e -> fprintf stderr "Error while connecting to fg: %s" (Printexc.to_string e)
    in
    
    let take_off = fun () -> FlightModel.set_air_speed !state FM.nominal_airspeed in 

    let hbox = GPack.hbox ~packing:vbox#pack () in
    let s = GButton.button ~label:"Boot" ~packing:(hbox#pack ~padding:5) () in
    ignore (s#connect#clicked ~callback:boot);
    let t = GButton.button ~label:"Launch" ~packing:hbox#pack () in
    ignore (t#connect#clicked ~callback:take_off);
    let ir_srtm_button = GButton.toggle_button ~label:"IR/srtm" ~packing:hbox#pack () in
    ignore (ir_srtm_button#connect#toggled (fun () -> ir_srtm := not !ir_srtm));

    let hbox = GPack.hbox ~packing:vbox#pack () in
    let l = fun s -> ignore(GMisc.label ~text:s ~packing:hbox#pack ()) in
    l "East:"; hbox#pack east_label#coerce;
    l " North:"; hbox#pack north_label#coerce;
    l " Height:"; hbox#pack alt_label#coerce;

    Ivy.init (sprintf "Paparazzi sim %d" A.ac.Data.id) "READY" (fun _ _ -> ());
    Ivy.start !ivy_bus;

    window#show ();
    Unix.handle_unix_error GMain.Main.main ()
end
