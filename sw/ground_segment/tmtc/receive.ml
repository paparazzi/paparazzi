(*
 * $Id$
 *
 * Multi aircrafts receiver, logger and broadcaster
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
module U = Unix

module Tele_Class = struct let name = "telemetry_ap" end
module Tc_Class = struct let name = "non" end
module AcInfo = struct let name = "aircraft_info" end
module Tele_Pprz = Pprz.Protocol(Tele_Class)
module Tc_Pprz = Pprz.Protocol(Tc_Class)
module AcInfo_Pprz = Pprz.Protocol(AcInfo)

let (//) = Filename.concat
let logs_path = Env.paparazzi_home // "var" // "logs"
let conf_xml = Xml.parse_file (Env.paparazzi_home // "conf" // "conf.xml")

type port = Ivy of string | Modem of string
type aircraft = { 
    port : port;
    mutable roll    : float;
    mutable pitch   : float;
    mutable east    : float;
    mutable north    : float;
    mutable nav_ref_east    : float;
    mutable nav_ref_north    : float;
    mutable desired_east    : float;
    mutable desired_north    : float;
    mutable gspeed  : float;
    mutable course  : float;
    mutable alt     : float;
    mutable climb   : float;
    mutable cur_block : int;
    mutable cur_stage : int;
(* warning twin engines ?? *)
    mutable throttle : float;
    mutable rpm  : float;
    mutable temp : float;
    mutable bat  : float;
    mutable amp : float;
    mutable energy  : float;
    mutable ap_mode : int;
    mutable ap_altitude : int;
    mutable if_calib_mode : int;
    mutable mcu1_status : int;
    mutable lls_calib : int;
  }

(** The aircrafts store *)
let aircrafts = Hashtbl.create 3

(** Broadcast of the received aircrafts *)
let aircrafts_msg_period = 5000 (* ms *)
let aircraft_msg_period = 1000 (* ms *)
let traffic_info_period = 2000 (* ms *)
let send_aircrafts_msg = fun () ->
  let t = U.gettimeofday () in
  let names = String.concat "," (Hashtbl.fold (fun k v r -> k::r) aircrafts [])  in
  Ivy.send (sprintf "ground AIRCRAFTS %s" names)
(*  Ivy.send (sprintf "YOUOPIIIII") *)

(* Opens the log file *)
(* FIXME : shoud open also an associated config file *)
let logger = fun () ->
  let d = U.localtime (U.gettimeofday ()) in
  let name = sprintf "%02d_%02d_%02d__%02d_%02d_%02d.log" (d.U.tm_year mod 100) (d.U.tm_mon+1) (d.U.tm_mday) (d.U.tm_hour) (d.U.tm_min) (d.U.tm_sec) in
  if not (Sys.file_exists logs_path) then begin
    printf "Creating '%s'\n" logs_path; flush stdout;
    ignore (Sys.command (sprintf "mkdir -p %s" logs_path))
  end;
  open_out (logs_path // name)

let fvalue = fun x ->
  match x with
    Pprz.Float x -> x 
  | Pprz.Int32 x -> Int32.to_float x 
  | Pprz.Int x -> float_of_int x 
  | _ -> failwith (sprintf "Receive.log_and_parse: float expected, got '%s'" (Pprz.string_of_value x))
let ivalue = fun x ->
  match x with
    Pprz.Int x -> x 
  | _ -> failwith "Receive.log_and_parse: int expected"



let log_and_parse = fun log ac_name a msg values ->
  let t = U.gettimeofday () in
  let s = String.concat " " (List.map (fun (_, v) -> Pprz.string_of_value v) values) in
  fprintf log "%.2f %s %s %s\n" t ac_name msg.Pprz.name s; flush log;
  Ivy.send (sprintf "%s RAW %.2f %s %s" ac_name t msg.Pprz.name s);
  let value = fun x -> try List.assoc x values with Not_found -> failwith (sprintf "Error: field '%s' not found\n" x) in
  let fvalue = fun x -> fvalue (value x)
  and ivalue = fun x -> ivalue (value x) in
  match msg.Pprz.name with
    "GPS" ->
      a.east    <- fvalue "east" /. 100.;
      a.north    <- fvalue "north" /. 100.;
      a.gspeed  <- fvalue "speed";
      a.course  <- fvalue "course";
      a.alt     <- fvalue "alt";
      a.climb   <- fvalue "climb"
  | "DESIRED" ->
      a.desired_east <- fvalue "desired_x";
      a.desired_north <- fvalue "desired_y"
  | "NAVIGATION_REF" ->
      a.nav_ref_east <- fvalue "utm_east";
      a.nav_ref_north <- fvalue "utm_north"
  | "ATTITUDE" ->
      a.roll <- fvalue "phi";
      a.pitch <- fvalue "theta"
  | "NAVIGATION" -> 
      a.cur_block <- ivalue "cur_block";
      a.cur_stage <- ivalue "cur_stage"
  | "CLIMB_PID" ->
      a.throttle <- fvalue "gaz" /. 9600. *. 100.;
      a.rpm <- a.throttle *. 100.
  | "BAT" ->
      a.bat <- fvalue "voltage" /. 10.
  | "PPRZ_MODE" ->
      a.ap_mode <- ivalue "ap_mode";
      a.ap_altitude <- ivalue "ap_altitude";
      a.if_calib_mode <- ivalue "if_calib_mode";
      a.mcu1_status <- ivalue "mcu1_status";
      a.lls_calib <- ivalue "lls_calib"
  | _ -> ()


(** Callback for a message from a soft simulator *)
let sim_msg = fun log ac_name a m ->
  try
    let (msg_id, values) = Tele_Pprz.values_of_string m in
    let msg = Tele_Pprz.message_of_id msg_id in
    log_and_parse log ac_name a msg values
  with
    Pprz.Unknown_msg_name x ->
      fprintf stderr "Unknown message %s from %s: %s\n" x ac_name m

let soi = string_of_int

let send_aircraft_msg = fun ac ->
  try
    let sof = fun f -> sprintf "%.1f" f in
    let a = Hashtbl.find aircrafts ac in
    let f = fun x -> Pprz.Float x in
    let values = ["roll", f (Geometry_2d.rad2deg a.roll);
		  "pitch", f (Geometry_2d.rad2deg a.pitch);
		  "east", f a.east;
		  "north", f a.north;
		  "speed", f a.gspeed;
		  "heading", f (Geometry_2d.rad2deg a.course);
		  "alt", f a.alt;
		  "climb", f a.climb] in
    let _, fp_msg = AcInfo_Pprz.message_of_name "FLIGHT_PARAM" in
    Ivy.send (sprintf "%s %s" ac (AcInfo_Pprz.string_of_message fp_msg values));

    let values = ["cur_block", Pprz.Int a.cur_block;"cur_stage", Pprz.Int a.cur_stage; "target_east", f (a.nav_ref_east+.a.desired_east); "target_north", f (a.nav_ref_north+.a.desired_north)]
    and _, ns_msg =  AcInfo_Pprz.message_of_name "NAV_STATUS" in
    Ivy.send (sprintf "%s %s" ac (AcInfo_Pprz.string_of_message ns_msg values));

    let values = ["throttle", f a.throttle;"rpm", f a.rpm;"temp", f a.temp;"bat", f a.bat;"amp", f a.amp;"energy", f a.energy]
    and _, es_msg = AcInfo_Pprz.message_of_name "ENGINE_STATUS" in
    Ivy.send (sprintf "%s %s" ac (AcInfo_Pprz.string_of_message es_msg values));

    let values = ["mode", Pprz.Int a.ap_mode; "v_mode", Pprz.Int a.ap_altitude]
    and _, as_msg =  AcInfo_Pprz.message_of_name "AP_STATUS" in
    Ivy.send (sprintf "%s %s" ac (AcInfo_Pprz.string_of_message as_msg values))
  with
    Not_found -> prerr_endline ac

let send_traffic_info = fun ac ->
  (* TODO: should send up on the datalink *)
  (* Sending on the Ivy bus for the simulators *)
  let a = Hashtbl.find aircrafts ac in
  let f = fun x -> Pprz.Float x in
  let conf = ExtXml.child conf_xml "aircraft" ~select:(fun x -> ExtXml.attrib x "ac_id" = ac) in
  let values = ["ac_id", Pprz.Int (int_of_string (ExtXml.attrib conf "ac_id"));
                "east", f a.east;
		"north", f a.north;
		"speed", f a.gspeed;
		"heading", f (Geometry_2d.rad2deg a.course);
		"alt", f a.alt;
		"climb", f a.climb] in
  let _, fp_msg = Tc_Pprz.message_of_name "TRAFFIC_INFO" in
  Ivy.send (sprintf "%s %s" ac (Tc_Pprz.string_of_message fp_msg values))
      
let new_aircraft = fun id ->
    { port = id ; roll = 0.; pitch = 0.; east = 0.; north = 0.; nav_ref_east = 0.; nav_ref_north = 0.; desired_east = 0.; desired_north = 0.; gspeed=0.; course = 0.; alt=0.; climb=0.; cur_block=0; cur_stage=0; throttle = 0.; rpm = 0.; temp = 0.; bat = 0.; amp = 0.; energy = 0.; ap_mode=0; ap_altitude=0; if_calib_mode=0; mcu1_status=0; lls_calib=0 }
    
let register_aircraft = fun name a ->
  Hashtbl.add aircrafts name a;
  ignore (Glib.Timeout.add aircraft_msg_period (fun () -> send_aircraft_msg name; true));
  ignore (Glib.Timeout.add traffic_info_period (fun () -> send_traffic_info name; true))


(** Callback of an identifying message from a soft simulator *)
let ident_msg = fun log id name ->
  if not (Hashtbl.mem aircrafts name) then begin
    let ac = new_aircraft (Ivy id) in
    let b = Ivy.bind (fun _ args -> sim_msg log name ac args.(0)) (sprintf "^%s +(.*)" id) in
    register_aircraft name ac;
    send_aircrafts_msg ()
  end

(* Waits for new simulated aircrafts *)
let listen_sims = fun log ->
  ignore (Ivy.bind (fun _ args -> ident_msg log args.(0) args.(1)) "^(.*) IDENT +(.*)")

(* Server on the Ivy bus *)
let send_flight_plan = fun id ->
  try
    let conf = ExtXml.child conf_xml "aircraft" ~select:(fun x -> ExtXml.attrib x "ac_id" = id) in
    let f = ExtXml.attrib conf "flight_plan" in
    Ivy.send (sprintf "ground FLIGHT_PLAN %s file://%s/conf/%s" id Env.paparazzi_home f)
  with
    Not_found ->
      Ivy.send (sprintf "ground UNKNOWN %s" id)

let send_config = fun id_ac id_req ->
  try
    prerr_endline (sprintf "[%s] [%s]\n" id_ac id_req);
    let conf = ExtXml.child conf_xml "aircraft" ~select:(fun x -> ExtXml.attrib x "ac_id" = id_ac) in
    let fp = sprintf "%s/conf/%s" Env.paparazzi_home (ExtXml.attrib conf "flight_plan") and
	af = sprintf "%s/conf/%s" Env.paparazzi_home (ExtXml.attrib conf "airframe") and
	rc = sprintf "%s/conf/%s" Env.paparazzi_home (ExtXml.attrib conf "radio")in
    let resp = sprintf "%s CONFIG_RES %s %s %s %s" id_ac id_req fp af rc in 
    Ivy.send (resp);
    prerr_endline (resp)
  with
    Not_found ->
      Ivy.send (sprintf "ground UNKNOWN %s" id_req)     
    
let ivy_server = fun () ->
  ignore (Ivy.bind (fun _ args -> send_aircrafts_msg ()) "^ask AIRCRAFTS");
  ignore (Ivy.bind (fun _ args -> send_flight_plan args.(0)) "^ask FLIGHT_PLAN +(.*)");
  ignore (Ivy.bind (fun _ args -> send_config args.(0) args.(1)) "^(.*) CONFIG_REQ +(.*)")

(***
let handle_pprz_message = fun log a ->
  let name = ref None (*** register_aircraft "log_twinstar" a; Some "log_twinstar" ***) in
  fun ...
    let msg = Tele_Pprz.message_of_id msg_id in
    match !name with
      None ->
	if msg.Pprz.name = "IDENT" then
	  let n = Pprz.string_of_value (List.assoc "id" values) in
	  name := Some n;
	  register_aircraft n a
    | Some ac_name ->
	log_and_parse log ac_name a msg values
***)

  


(* main loop *)
let _ =
  let xml_ground = ExtXml.child conf_xml "ground" in
  let ivy_bus = ref (ExtXml.attrib xml_ground "ivy_bus") in
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), (sprintf "Bus\tDefault is %s" !ivy_bus)] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: ";

  Ivy.init "Paparazzi receive" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (* Opens the log file *)
  let log = logger () in

  (* Waits for new simulated aircrafts *)
  listen_sims log;

  (* Sends periodically alive aircrafts *)
  ignore (Glib.Timeout.add aircrafts_msg_period (fun () -> send_aircrafts_msg (); true));

  (* Waits for client requests on the Ivy bus *)
  ivy_server ();
  
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
