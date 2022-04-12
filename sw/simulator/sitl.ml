(*
 * Software in the loop basic simulator (handling GPS, infrared and commands)
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

module Ground_Pprz = PprzLink.Messages(struct let name = "ground" end)
module Dl_Pprz = PprzLink.Messages(struct let name = "datalink" end)

let ground_id = 0 (* cf tmtc/link.ml *)

let ios = int_of_string
let fos = float_of_string

let raw_datalink_msg_separator = Str.regexp ";"

let norc = ref false
let adc1 = ref false

module Make (A:Data.MISSION) (FM: FlightModel.SIG) = struct

  let servos_period = 1./.40. (* s *)
  let periodic_period = 1./.60. (* s *)
  let rc_period = 1./.40. (* s *)
  let sys_time_period = 1./.120. (* s *)

  (* Commands handling (rcommands is the intermediate storage) *)
  let rc_channels = Array.of_list (Xml.children A.ac.Data.radio)
  let nb_channels = Array.length rc_channels
  let rc_channel_no = fun x ->
    List.assoc x (Array.to_list (Array.mapi (fun i c -> Xml.attrib c "function", i) rc_channels))

  let rcommands = ref [||]
  let adj_bat = GData.adjustment ~value:FM.max_bat_level ~lower:0. ~upper:(FM.max_bat_level+.2.) ~step_incr:0.1 ~page_size:0. ()

  external get_commands : Simlib.pprz_t array -> int = "get_commands"
(** Returns gaz servo value (us) *)

  let energy = ref 0.

 (** Get the commands from the autopilot, store them
    (to be used by the flight model)
     Computes an energy consumption from throttle level *)
  let update_servos =
    let accu = ref 0. in
    fun bat_button () ->
      let gaz = get_commands !rcommands in
      (* 100% = 1W *)
      if bat_button#active then
	let energy = float (gaz-1000) /. 1000. *. servos_period in
	accu := !accu +. energy *. 0.00259259259259259252; (* To be improved !!! *)
	if !accu >= 0.1 then begin
	  let b = adj_bat#value in
	  adj_bat#set_value (b -. !accu);
	  accu := 0.
	end

(* Radio command handling *)
  external update_channel : int -> float -> unit = "update_rc_channel"
  external send_ppm : unit -> unit = "send_ppm"

  let inverted = ["ROLL"; "PITCH"; "YAW"; "GAIN1"; "GAIN2"]

  let rc = fun () ->
    let name = Xml.attrib A.ac.Data.radio "name" ^ " " ^ A.ac.Data.name in
    let icon = GdkPixbuf.from_file Env.icon_file in
    let window = GWindow.window ~icon ~title:name ~border_width:0 ~width:200 ~height:400 () in
    let quit = fun () -> GMain.Main.quit (); exit 0 in
    ignore (window#connect#destroy ~callback:quit);
    let vbox = GPack.vbox ~height:10 ~spacing: 1 ~border_width: 1 ~packing:window#add () in
    let on_off = GButton.check_button ~label:"On" ~active:true ~packing:vbox#pack () in
    let sliders = GPack.vbox ~packing:vbox#add () in
    let float_attrib = fun a x -> float_of_string (ExtXml.attrib x a) in
    Array.iteri
      (fun i c ->
	let mi = float_attrib "min" c
	and ma = float_attrib "max" c
	and value = float_attrib "neutral" c in
	let lower = min mi ma
	and upper = max mi ma in
	let adj = GData.adjustment ~value ~lower ~upper ~step_incr:1.0 () in
	let hbox = GPack.hbox ~packing:sliders#add () in
	let f = (ExtXml.attrib c "function") in
	let _l = GMisc.label ~width:75 ~text:f ~packing:hbox#pack () in
	let inv = not ((List.mem f inverted) == (ma < mi)) in
	let _scale = GRange.scale `HORIZONTAL ~inverted:inv ~adjustment:adj ~packing:hbox#add () in
	let update = fun () -> update_channel i adj#value in

	ignore (adj#connect#value_changed update);
	update ())
      rc_channels;
    (* github issue #821: people seems to want sliders always sensitive, even if RC is OFF *)
    (* ignore (on_off#connect#toggled (fun () -> sliders#coerce#misc#set_sensitive on_off#active)); *)

    on_off#set_active false;

    let send_ppm = fun () ->
      if on_off#active then send_ppm () in

    Simlib.timer rc_period send_ppm; (** FIXME: should use time_scale *)
    window#show ()

  external sys_time_task : unit -> unit = "sim_sys_time_task"
  external periodic_task : unit -> unit = "sim_periodic_task"
  external sim_init : unit -> unit = "sim_init"
  external update_bat : int -> unit = "update_bat"
  external update_adc1 : int -> unit = "update_adc1"
  external update_dl_status : int -> unit = "update_dl_status"

  let bat_button = GButton.check_button ~label:"Auto" ~active:false ()
  let dl_button = GButton.check_button ~label:"Enable Datalink/Telemetry" ~active:true ()

  let my_id = ref (-1)
  let init = fun id vbox ->
    if not !norc then
      rc ();
    my_id := id;
    sim_init ();

    (* Bat level *)
    let hbox = GPack.hbox ~spacing:4 ~packing:vbox#add () in
    let _label = GMisc.label ~text:"Bat (V) " ~packing:hbox#pack () in
    let _scale = GRange.scale `HORIZONTAL ~adjustment:adj_bat ~packing:hbox#add () in
    let update = fun () -> update_bat (truncate (adj_bat#value *. 10.)) in
    hbox#pack bat_button#coerce;
    let tips = GData.tooltips () in
    tips#set_tip bat_button#coerce ~text:"Select for auto-decreasing voltage";
    ignore (adj_bat#connect#value_changed update);
    update ();

    (* Datalink status *)
    let hbox = GPack.hbox ~spacing:4 ~packing:vbox#add () in
    let update = fun () -> update_dl_status (if dl_button#active then 1 else 0) in
    hbox#pack dl_button#coerce;
    let tips = GData.tooltips () in
    tips#set_tip dl_button#coerce ~text:"Enable/disable communication with the aircraft";
    ignore (dl_button#connect#toggled update);
    update();

    (* ADC1 *)
    if !adc1 then
      let hbox = GPack.hbox ~spacing:4 ~packing:vbox#add () in
      let _label = GMisc.label ~text:"Generic ADC 1 " ~packing:hbox#pack () in
      let adj_adc1 = GData.adjustment ~value:512. ~lower:0. ~upper:1033. ~step_incr:1. () in
      let _scale = GRange.scale `HORIZONTAL ~adjustment:adj_adc1 ~packing:hbox#add () in
      let update = fun () -> update_adc1 (truncate adj_adc1#value) in
      ignore (adj_adc1#connect#value_changed update);
      update ()

  open Latlong

  external set_message : string -> unit = "set_datalink_message"
  let get_message = fun name link_mode _sender vs ->
    let set = fun rcv_id ->
      let msg_id, _ = Dl_Pprz.message_of_name name in
      let s = Dl_Pprz.payload_of_values msg_id ground_id rcv_id vs in
      set_message (Protocol.string_of_payload s) in
    let ac_id = try Some (PprzLink.int_assoc "ac_id" vs) with _ -> None in
    match link_mode, ac_id with
      PprzLink.Forwarded, Some x when x = !my_id -> if dl_button#active then set x
    | PprzLink.Broadcasted, _ -> if dl_button#active then set PprzLink.broadcast_id
    | _ -> ()

  let message_bind = fun name link_mode ->
    ignore (Dl_Pprz.message_bind name (get_message name link_mode))

  let boot = fun time_scale ->
    Simlib.timer ~scale:time_scale servos_period (update_servos bat_button);
    Simlib.timer ~scale:time_scale periodic_period periodic_task;
    Simlib.timer ~scale:time_scale sys_time_period sys_time_task;

    (* Forward or broacast messages according to "link" mode *)
    Hashtbl.iter
      (fun _m_id msg ->
	match msg.PprzLink.link with
	  Some x -> message_bind msg.PprzLink.name x
	| _ -> ())
      Dl_Pprz.messages;;

(* Functions called by the simulator *)
  let commands = fun s -> rcommands := s

  external set_airspeed : float -> unit = "set_airspeed"
  let airspeed = fun air_speed ->
    set_airspeed air_speed

  external provide_attitude : float -> float -> float -> unit = "provide_attitude"
  external provide_rates : float -> float -> float -> unit = "provide_rates"
  let attitude_and_rates = fun phi theta psi p q r ->
    provide_attitude phi theta psi;
    provide_rates p q r

  external use_gps_pos: int -> int -> int -> float -> float -> float -> float -> float -> bool -> float -> float -> unit = "sim_use_gps_pos_bytecode" "sim_use_gps_pos"
  let gps = fun gps ->
    let utm = utm_of WGS84 gps.Gps.wgs84 in
    let cm = fun f -> truncate (f *. 100.) in
    use_gps_pos (cm utm.utm_x) (cm utm.utm_y) utm.utm_zone gps.Gps.course gps.Gps.alt gps.Gps.gspeed gps.Gps.climb gps.Gps.time gps.Gps.availability gps.Gps.wgs84.Latlong.posn_lat gps.Gps.wgs84.Latlong.posn_long

end
let options = ["-norc", Arg.Set norc, "Hide the simulated RC";
	       "-adc1", Arg.Set adc1, "Enable the generic adc 1"]
