(*
 *  $Id$
 *
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

module Ground_Pprz = Pprz.Messages(struct let name = "ground" end)

let ios = int_of_string
let fos = float_of_string

module Make(A:Data.MISSION) = struct

  let servos_period = 1./.40. (* s *)
  let periodic_period = 1./.61. (* s *)
  let rc_period = 1./.40. (* s *)
      
  let msg = fun name ->
    ExtXml.child Data.messages_ap ~select:(fun x -> ExtXml.attrib x "name" = name) "message"


(* Commands handling (rcommands is the intermediate storage) *)
  let rc_channels = Array.of_list (Xml.children A.ac.Data.radio)
  let nb_channels = Array.length rc_channels
  let rc_channel_no = fun x -> 
    List.assoc x (Array.to_list (Array.mapi (fun i c -> Xml.attrib c "function", i) rc_channels))

  let rcommands = ref [||]
  let adj_bat = GData.adjustment ~value:12.5 ~lower:0. ~upper:23. ~step_incr:0.1 ()

  external get_commands : Stdlib.pprz_t array -> int = "get_commands"
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
    let window = GWindow.window ~title:name ~border_width:0 ~width:200 ~height:400 () in
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
	let l = GMisc.label ~width:75 ~text:f ~packing:hbox#pack () in
	let inv = not ((List.mem f inverted) == (ma < mi)) in
	let _scale = GRange.scale `HORIZONTAL ~inverted:inv ~adjustment:adj ~packing:hbox#add () in
	let update = fun () -> update_channel i adj#value in
	
	ignore (adj#connect#value_changed update);
	update ())
      rc_channels;
    ignore (on_off#connect#toggled (fun () -> sliders#coerce#misc#set_sensitive on_off#active));

    let send_ppm = fun () ->
      if on_off#active then send_ppm () in

    Stdlib.timer rc_period send_ppm; (** FIXME: should use time_scale *)
    window#show ()

  external periodic_task : unit -> unit = "sim_periodic_task"
  external sim_init : unit -> unit = "sim_init"
  external update_bat : int -> unit = "update_bat"

  let bat_button = GButton.toggle_button ~label:"Bat" ()

  let my_id = ref (-1)
  let init = fun id vbox ->
    rc ();
    my_id := id;
    sim_init ();

    let hbox = GPack.hbox ~packing:vbox#add () in
    hbox#pack bat_button#coerce;
    let tips = GData.tooltips () in
    tips#set_tip bat_button#coerce ~text:"Select for auto-decreasing voltage";
    let _scale = GRange.scale `HORIZONTAL ~adjustment:adj_bat ~packing:hbox#add () in
    let update = fun () -> update_bat (truncate (adj_bat#value *. 10.)) in
    ignore (adj_bat#connect#value_changed update);
    update ()

  open Latlong
  external set_ac_info : int -> float -> float -> float -> float -> float -> unit = "set_ac_info"
  let get_flight_param = fun _sender vs ->
    let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
    if ac_id <> !my_id then
      let f = fun a -> Pprz.float_assoc a vs in
      let lat = f "lat"
      and long = f "long"
      and course = (Deg>>Rad)(f "course")
      and alt = f "alt"
      and gspeed = f "speed" in
      let wgs84 = {posn_lat=(Deg>>Rad)lat;posn_long=(Deg>>Rad)long} in
      let utm = Latlong.utm_of WGS84 wgs84 in
      set_ac_info ac_id utm.utm_x utm.utm_y course alt gspeed

  external move_waypoint : int -> float -> float -> float -> unit = "move_waypoint"
  let get_move_waypoint = fun _sender vs ->
    let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
    if ac_id = !my_id then
      let f = fun a -> Pprz.float_assoc a vs in
      let wp_id = Pprz.int_assoc "wp_id" vs
      and lat = f "lat"
      and long = f "long"
      and alt = f "alt" in
      let wgs84 = {posn_lat=(Deg>>Rad)lat;posn_long=(Deg>>Rad)long} in
      let utm = Latlong.utm_of WGS84 wgs84 in
      move_waypoint wp_id utm.utm_x utm.utm_y alt

  external send_event : int -> unit = "send_event"
  let get_send_event = fun _sender vs ->
    let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
    if ac_id = !my_id then
      send_event (Pprz.int_assoc "event_id" vs)


  external goto_block : int -> unit = "goto_block"
  let get_jump_to_block = fun _sender vs ->
    let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
    if ac_id = !my_id then
      goto_block (Pprz.int_assoc "block_id" vs)


  external dl_setting : int -> float -> unit = "dl_setting"
  let get_dl_setting = fun _sender vs ->
    let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
    if ac_id = !my_id then
      dl_setting (Pprz.int_assoc "index" vs) (Pprz.float_assoc "value" vs)


  let boot = fun time_scale ->
    Stdlib.timer ~scale:time_scale servos_period (update_servos bat_button);
    Stdlib.timer ~scale:time_scale periodic_period periodic_task;
    ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" get_flight_param);
    ignore (Ground_Pprz.message_bind "MOVE_WAYPOINT" get_move_waypoint);
    ignore (Ground_Pprz.message_bind "SEND_EVENT" get_send_event);
    ignore (Ground_Pprz.message_bind "JUMP_TO_BLOCK" get_jump_to_block);
    ignore (Ground_Pprz.message_bind "DL_SETTING" get_dl_setting)

(* Functions called by the simulator *)
  let commands = fun s -> rcommands := s

  external set_ir : int -> int -> unit = "set_ir"
  let infrared = fun ir_left ir_front ir_top ->
    (** ADC neutral is not taken into account in the soft sim (c.f. sim_ir.c)*)
    set_ir (truncate ir_left) (truncate ir_top)

  external use_gps_pos: int -> int -> int -> float -> float -> float -> float -> float -> bool -> unit = "sim_use_gps_pos_bytecode" "sim_use_gps_pos"
  let gps = fun gps ->
    let utm = utm_of WGS84 gps.Gps.wgs84 in
    let cm = fun f -> truncate (f *. 100.) in
    use_gps_pos (cm utm.utm_x) (cm utm.utm_y) utm.utm_zone gps.Gps.course gps.Gps.alt gps.Gps.gspeed gps.Gps.climb gps.Gps.time gps.Gps.availability

end
let options = []
