(*
 * Hardware in the loop basic simulator (handling GPS, infrared and commands)
 *
 * Copyright (C) 2004-2006 Pascal Brisset, Antoine Drouin
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
open Latlong

module Ground_Pprz = PprzLink.Messages(struct let name = "ground" end)

let float_attrib xml a = float_of_string (ExtXml.attrib xml a)

(* From mapFP.ml to avoid complex linking *)
let georef_of_xml = fun xml ->
  let lat0 = Latlong.deg_of_string (ExtXml.attrib xml "lat0")
  and lon0 = Latlong.deg_of_string (ExtXml.attrib xml "lon0") in
  { posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 }


(* Frequencies for perdiodic tasks are expressed in s *)
let ir_period = 1./.20.
let fm_period = 1./.25.
let fg_period = 1./.25.
let ahrs_period = 1./.20.

let gensym = let n = ref 0 in fun p -> incr n; p ^ string_of_int !n
let cb_register = fun closure ->
  let s = gensym "sim_callback_" in
  Callback.register s closure;
  s


module type AIRCRAFT =
sig
  val init : int -> GPack.box -> unit
    (** [init ac_id box] *)
  val boot : Stdlib.value -> unit
    (** [boot time_acceleration] *)

  val commands : pprz_t array -> unit
    (** Called once at init *)

  val infrared_and_airspeed : float -> float -> float -> float -> unit
    (** [infrared ir_left ir_front ir_top air_speed] Called on timer *)

  val attitude_and_rates : float -> float -> float -> float -> float -> float ->unit
    (** [ahrs phi theta psi p q r] Called on timer *)

  val gps : Gps.state -> unit
  (** [gps state] Called on timer *)
end


module type AIRCRAFT_ITL =
  functor (A : Data.MISSION) -> functor (FM: FlightModel.SIG) -> AIRCRAFT

external fg_sizeof : unit -> int = "fg_sizeof"
external fg_msg : bytes -> float -> float -> float -> float -> float -> float -> unit = "fg_msg_bytecode" "fg_msg_native"

let ac_name = ref "A/C not set"

let ivy_bus = ref Defivybus.default_ivy_bus

let fg_client = ref ""

let autoboot = ref false
let autolaunch = ref false
let noground = ref false

let common_options = [
  "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
  "-boot", Arg.Set autoboot, "Boot the A/C on start";
  "-launch", Arg.Set autolaunch, "Launch the A/C on start";
  "-noground", Arg.Set noground, "Disable ground detection";
  "-fg", Arg.Set_string fg_client, "Flight gear client address"
]

module Make(AircraftItl : AIRCRAFT_ITL) = struct

  module A = struct
    let ac = Data.aircraft !ac_name
  end

  module FM = FlightModel.Make(A)

  module Aircraft = AircraftItl(A)(FM)

  let flight_plan = A.ac.Data.flight_plan

  let pos0 = ref (georef_of_xml flight_plan)
  let qfu = try float_attrib flight_plan "qfu" with _ -> 0.

  (* Try to get the ground alt from the SRTM data, default to flight plan *)

  let alt0 =
    let ground_alt =
      Srtm.add_path (Env.paparazzi_home ^ "/data/srtm");
      try
        float (Srtm.of_wgs84 !pos0)
      with Srtm.Tile_not_found x ->
        float_attrib flight_plan "ground_alt" in
    ref ground_alt

  let main () =
    let icon = GdkPixbuf.from_file Env.icon_sim_file in
    let window = GWindow.window ~type_hint:`DIALOG ~icon ~title: !ac_name () in
    let quit = fun () -> GMain.Main.quit (); exit 0 in
    ignore (window#connect#destroy ~callback:quit);
    let vbox = GPack.vbox ~packing:window#add () in

    Aircraft.init A.ac.Data.id vbox;

    let gps_period = 0.25 in

    let compute_gps_state = Gps.state pos0 alt0 in

    let initial_state = FM.init (pi/.2. -. qfu/.180.*.pi) in

    let state = ref initial_state in

    let _reset = fun () -> state := initial_state in

    let commands = Array.make FM.nb_commands 0 in

    Aircraft.commands commands;

    let north_label = GMisc.label ~text:"000" ()
    and east_label = GMisc.label ~text:"000" ()
    and alt_label = GMisc.label ~text:"000" () in

    let last_gps_state = ref None in
    let _run = ref false in

    let wind_x = ref 0.
    and wind_y = ref 0.
    and wind_z = ref 0. in
    let infrared_contrast = ref 266.
    and time_scale = object val mutable v = 1. method value = v method set_value x = v <- x end
    and gps_availability = ref 1 in

    let world_update = fun _ vs ->
      gps_availability := PprzLink.int_assoc "gps_availability" vs;
      wind_x := PprzLink.float_assoc "wind_east" vs;
      wind_y := PprzLink.float_assoc "wind_north" vs;
      wind_z := PprzLink.float_assoc "wind_up" vs;
      infrared_contrast := PprzLink.float_assoc "ir_contrast" vs;
      time_scale#set_value (PprzLink.float_assoc "time_scale" vs)
    in

    let ask_for_world_env = fun () ->
      try
        let (x, y, z) = FlightModel.get_xyz !state in

        let gps_sol = compute_gps_state (x,y,z) (FlightModel.get_time !state) in

        let float = fun f -> PprzLink.Float f in
        let values = ["east", float x; "north", float y; "up", float z;
                      "lat", float ((Rad>>Deg)gps_sol.Gps.wgs84.posn_lat);
                      "long", float ((Rad>>Deg)gps_sol.Gps.wgs84.posn_long);
                      "alt", float gps_sol.Gps.alt ] in
        let (b, flag) = Ground_Pprz.message_req "sim" "WORLD_ENV" values world_update in
        (* unbind manually after 1s if no message received *)
        ignore (GMain.Timeout.add 1000 (fun () -> if !flag then Ivy.unbind b; false))
      with
          exc -> fprintf stderr "Error in sim: %s\n%!" (Printexc.to_string exc)
    in

    ignore (GMain.Timeout.add 1000 (fun () -> ask_for_world_env (); true));


    let fm_task = fun () ->
      FM.do_commands !state commands;
      let agl =
        if !noground then max_float
        else
          match !last_gps_state with
              Some s ->
                begin
                  try s.Gps.alt -. float (Srtm.of_wgs84 s.Gps.wgs84) with
                      _ -> s.Gps.alt -. !alt0
                end
            | None -> 0. in
      FM.state_update !state FM.nominal_airspeed (!wind_x, !wind_y, !wind_z) agl fm_period

    and ir_task = fun () ->
      let phi, theta, _ = FlightModel.get_attitude !state in

      let phi_sensor = phi +. FM.roll_neutral_default
      and theta_sensor = theta +. FM.pitch_neutral_default in

      let ir_left = sin phi_sensor *. !infrared_contrast
      and ir_front = sin theta_sensor *. !infrared_contrast
      and ir_top = cos phi_sensor *. cos theta_sensor *. !infrared_contrast in
      Aircraft.infrared_and_airspeed ir_left ir_front ir_top (FlightModel.get_air_speed !state)

    and gps_task = fun () ->
      let (x,y,z) = FlightModel.get_xyz !state in
      east_label#set_text (Printf.sprintf "%.0f" x);
      north_label#set_text (Printf.sprintf "%.0f" y);
      alt_label#set_text (Printf.sprintf "%.0f" z);
      let s = compute_gps_state (x,y,z) (FlightModel.get_time !state) in
      s.Gps.availability <- not (!gps_availability = 0);
      last_gps_state := Some s;
      Aircraft.gps s

    and ahrs_task = fun () ->
      let (phi, theta, psi) = FlightModel.get_attitude !state
      and p, q, r = FlightModel.get_pqr !state in
      Aircraft.attitude_and_rates phi theta psi p q r in

    (** Sending to Flight Gear *)
    let fg_task = fun socket buffer sockaddr () ->
      match !last_gps_state with
          None -> ()
        | Some s ->
          let lat = s.Gps.wgs84.Latlong.posn_lat
          and lon = s.Gps.wgs84.Latlong.posn_long
          and alt = s.Gps.alt
      (*    and theta_ = s.Gps.course *)
          and (phi, theta, psi) = FlightModel.get_attitude !state in
          fg_msg buffer lat lon alt phi theta psi;
      (**       for i = 0 to Bytes.length buffer - 1 do fprintf stderr "%x " (Char.code buffer.[i]) done; fprintf stderr "\n"; **)
          try
            ignore (Unix.sendto socket buffer 0 (Bytes.length buffer) [] sockaddr)
          with
              Unix.Unix_error (e,f,a) -> Printf.fprintf stderr "Error sending to FlightGear: %s (%s(%s))\n" (Unix.error_message e) f a; flush stderr
    in

    let set_pos = fun _ ->
      let current_pos = Latlong.string_of !pos0 in
      begin
        match GToolbox.input_string ~title:"Setting geographic position" ~text:current_pos "Geographic position"  with
            Some s -> pos0 := Latlong.of_string s
          | _ -> ()
      end;
      begin
        let text = string_of_float !alt0 in
        match GToolbox.input_string ~title:"Setting initial altitude" ~text "Geographic altitude"  with
            Some s -> alt0 := float_of_string s
          | _ -> ()
      end
    in

    let boot = fun () ->
      Aircraft.boot (time_scale:>value);
      Stdlib.timer ~scale:time_scale fm_period fm_task;
      Stdlib.timer ~scale:time_scale ir_period ir_task;
      Stdlib.timer ~scale:time_scale gps_period gps_task;
      Stdlib.timer ~scale:time_scale ahrs_period ahrs_task;

      (** Connection to Flight Gear client *)
      if !fg_client <> "" then
        try
          let inet_addr = Unix.inet_addr_of_string !fg_client in
          let socket = Unix.socket Unix.PF_INET Unix.SOCK_DGRAM 0 in
          (* Unix.connect socket (Unix.ADDR_INET (inet_addr, 5501)); *)
          let buffer = Bytes.create (fg_sizeof ()) in
          let sockaddr = (Unix.ADDR_INET (inet_addr, 5501)) in
          Stdlib.timer ~scale:time_scale fg_period (fg_task socket buffer sockaddr);
          fprintf stdout "Sending to FlightGear at %s\n" !fg_client; flush stdout
        with
            e -> fprintf stderr "Error setting up FlightGear viz: %s\n" (Printexc.to_string e); flush stderr
    in

    let take_off = fun () -> FlightModel.set_air_speed !state FM.nominal_airspeed in

    let hbox = GPack.hbox ~spacing:4 ~packing:vbox#pack () in
    let s = GButton.button ~label:"Set Pos" ~packing:hbox#pack () in
    ignore (s#connect#clicked ~callback:set_pos);

    let set_pos_and_boot = fun () ->
      s#misc#set_sensitive false;
      boot () in

    autoboot := !autoboot || !autolaunch;

    if not !autoboot then begin
      let s = GButton.button ~label:"Boot" ~packing:(hbox#pack) () in
      let callback = fun () ->
        set_pos_and_boot ();
        s#misc#set_sensitive false in
      ignore (s#connect#clicked ~callback)
    end else
      set_pos_and_boot ();

    if not !autolaunch then begin
      let t = GButton.button ~label:"Launch" ~packing:hbox#pack () in
      let callback = fun () ->
        take_off ();
        t#misc#set_sensitive false in
      ignore (t#connect#clicked ~callback);

      (* Monitor an AUTO2 launch to disable the button *)
      let monitor = fun () ->
        if FlightModel.get_air_speed !state > 0. then begin
          t#misc#set_sensitive false;
          false
        end else
          true in
      ignore (GMain.Timeout.add 1000 monitor)
    end else
      take_off ();

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
