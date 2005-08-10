(*
 *  $Id$
 *
 * Software in the loop basic simulator (handling GPS, infrared and servos)
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

let ios = int_of_string
let fos = float_of_string

let ivy_bus = ref "127.255.255.255:2010"

module Make(A:Data.MISSION) = struct

  let servos_period = 25 (* ms *)
  let periodic_period = 16 (* ms *)
  let rc_period = 25 (* ms *)

  let periodic = fun p f ->
    f ();
    ignore (GMain.Timeout.add p (fun () -> f (); true))


  let msg = fun name ->
    ExtXml.child Data.messages_ap ~select:(fun x -> ExtXml.attrib x "name" = name) "message"
  let gps_msg = msg "GPS"


(* Servos handling (rservos is the intermediate storage) *)
  let rc_channels = Array.of_list (Xml.children A.ac.Data.radio)
  let nb_channels = Array.length rc_channels
  let rc_channel_no = fun x -> 
    List.assoc x (Array.to_list (Array.mapi (fun i c -> Xml.attrib c "function", i) rc_channels))

  let rservos = ref [||]
  let adj_bat = GData.adjustment ~value:12.5 ~lower:0. ~upper:23. ~step_incr:0.1 ()

  external set_servos : Stdlib.us array -> int = "set_servos"
(** Returns gaz servo value (us) *)

  let energy = ref 0.
  let update_servos = fun () ->
    let gaz = set_servos !rservos in
    (* 100% = 1W *)
    energy := !energy +. float (gaz-1000) /. 1000. *. float servos_period /. 1000.

  let update_adj_bat = fun () ->
    let b = adj_bat#value in
    adj_bat#set_value (b -. !energy *. 0.00259259259259259252); (* To be improved !!! *)
    energy := 0.


(* Radio command handling *)
  external update_channel : int -> float -> unit = "update_rc_channel"

  let inverted = ["ROLL"; "PITCH"; "YAW"; "GAIN1"; "GAIN2"]

  let rc = fun () ->
    let name = Xml.attrib A.ac.Data.radio "name" ^ " " ^ A.ac.Data.name in
    let window = GWindow.window ~title:name ~border_width:0 ~width:400 ~height:400 () in
    let quit = fun () -> GMain.Main.quit (); exit 0 in
    ignore (window#connect#destroy ~callback:quit);
    let vbox = GPack.vbox ~packing:window#add () in
    Array.iteri
      (fun i c ->
	let adj = GData.adjustment ~value:0. ~lower:(-100.) ~upper:110. ~step_incr:1.0 () in
	let hbox = GPack.hbox ~packing:vbox#add () in
	let f = (ExtXml.attrib c "function") in
	let l = GMisc.label ~width:75 ~text:f ~packing:hbox#pack () in
	let inv = List.mem f inverted in
	let _scale = GRange.scale `HORIZONTAL ~inverted:inv ~adjustment:adj ~packing:hbox#add () in
	let update = fun () -> update_channel i (adj#value /. 100.) in
	
	ignore (adj#connect#value_changed update);
	update ())
      rc_channels;

    window#show ()

  external periodic_task : unit -> unit = "sim_periodic_task"
  external rc_task : unit -> unit = "sim_rc_task"
  external sim_init : int -> unit = "sim_init"
  external update_bat : int -> unit = "update_bat"

  let init = fun id vbox ->
    Ivy.init (sprintf "Paparazzi sim %d" id) "READY" (fun _ _ -> ());
    Ivy.start !ivy_bus;
    rc ();
    sim_init id;

    let hbox = GPack.hbox ~packing:vbox#add () in
    let l = GMisc.label ~text:"Bat:" ~packing:hbox#pack () in
    let _scale = GRange.scale `HORIZONTAL ~adjustment:adj_bat ~packing:hbox#add () in
    let update = fun () -> update_bat (truncate (adj_bat#value *. 10.)) in
    ignore (adj_bat#connect#value_changed update);
    update ()


  let boot = fun () ->
    periodic servos_period update_servos;
    periodic periodic_period periodic_task;
    periodic rc_period rc_task;
    periodic 10000 update_adj_bat

(* Functions called by the simulator *)
  let servos = fun s -> rservos := s

  external set_ir_roll : int -> unit = "set_ir_roll"
  let infrared = fun phi ctrst ->
    set_ir_roll (truncate (phi *. ctrst))

  external use_gps_pos: int -> int -> float -> float -> float -> float -> float -> unit = "sim_use_gps_pos_bytecode" "sim_use_gps_pos"
  open Latlong
  let gps = fun gps ->
    let utm = utm_of WGS84 gps.Gps.wgs84 in
    let cm = fun f -> truncate (f *. 100.) in
    use_gps_pos (cm utm.utm_x) (cm utm.utm_y) gps.Gps.course gps.Gps.alt gps.Gps.gspeed gps.Gps.climb gps.Gps.time 

end
let options =
  [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.25:2010"]

