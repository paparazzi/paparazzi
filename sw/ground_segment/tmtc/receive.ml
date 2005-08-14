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

let my_id = "ground"
let gps_mode_3D = 3

open Printf
open Latlong
module U = Unix

module Tele_Class = struct let name = "telemetry_ap" end
module Ground = struct let name = "ground" end
module Tele_Pprz = Pprz.Protocol(Tele_Class)
module Ground_Pprz = Pprz.Protocol(Ground)

let (//) = Filename.concat
let logs_path = Env.paparazzi_home // "var" // "logs"
let conf_xml = Xml.parse_file (Env.paparazzi_home // "conf" // "conf.xml")
let srtm_path = Env.paparazzi_home // "data" // "srtm"

type ac_cam = {
    mutable phi : float; (* Rad, right = >0 *)
    mutable theta : float; (* Rad, front = >0 *)
  }

type aircraft = { 
    id : string;
    mutable pos : Latlong.utm;
    mutable roll    : float;
    mutable pitch   : float;
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
    cam : ac_cam;
    mutable gps_mode : int
  }

(** The aircrafts store *)
let aircrafts = Hashtbl.create 3

(** Broadcast of the received aircrafts *)
let aircrafts_msg_period = 5000 (* ms *)
let aircraft_msg_period = 1000 (* ms *)
let traffic_info_period = 2000 (* ms *)
let send_aircrafts_msg = fun _asker _values ->
  assert(_values = []);
  let names = String.concat "," (Hashtbl.fold (fun k v r -> k::r) aircrafts []) ^ "," in
  ["ac_list", Pprz.String names]

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
  let value = fun x -> try List.assoc x values with Not_found -> failwith (sprintf "Error: field '%s' not found\n" x) in
  let fvalue = fun x -> fvalue (value x)
  and ivalue = fun x -> ivalue (value x) in
    match msg.Pprz.name with
      "GPS" ->
	a.pos <- { utm_x = fvalue "utm_east" /. 100.;
		   utm_y = fvalue "utm_north" /. 100.;
		   utm_zone = ivalue "utm_zone" };
	a.gspeed  <- fvalue "speed";
	a.course  <- fvalue "course";
	a.alt     <- fvalue "alt";
	a.climb   <- fvalue "climb";
	a.gps_mode <- ivalue "mode"
    | "DESIRED" ->
	a.desired_east <- fvalue "desired_x";
	a.desired_north <- fvalue "desired_y"
    | "NAVIGATION_REF" ->
	a.nav_ref_east <- fvalue "utm_east";
	a.nav_ref_north <- fvalue "utm_north"
    | "ATTITUDE" ->
	a.roll <- (Deg>>Rad) (fvalue "phi");
	a.pitch <- (Deg>>Rad) (fvalue "theta")
    | "NAVIGATION" -> 
	a.cur_block <- ivalue "cur_block";
	a.cur_stage <- ivalue "cur_stage"
    | "BAT" ->
	a.throttle <- fvalue "desired_gaz" /. 9600. *. 100.;
	a.rpm <- a.throttle *. 100.;
	a.bat <- fvalue "voltage" /. 10.
    | "PPRZ_MODE" ->
	a.ap_mode <- ivalue "ap_mode";
	a.ap_altitude <- ivalue "ap_altitude";
	a.if_calib_mode <- ivalue "if_calib_mode";
	a.mcu1_status <- ivalue "mcu1_status";
	a.lls_calib <- ivalue "lls_calib"
    | "CAM" ->
	a.cam.phi <- (Deg>>Rad) (fvalue  "phi");
	a.cam.theta <- (Deg>>Rad) (fvalue  "theta");
    | _ -> ()

(** Callback for a message from a registered A/C *)
let ac_msg = fun log ac_name a m ->
  try
    let (msg_id, values) = Tele_Pprz.values_of_string m in
    let msg = Tele_Pprz.message_of_id msg_id in
    log_and_parse log ac_name a msg values
  with
    Pprz.Unknown_msg_name x ->
      fprintf stderr "Unknown message %s from %s: %s\n" x ac_name m
  | x -> prerr_endline (Printexc.to_string x)

let soi = string_of_int

let send_cam_status = fun a ->
  if a.gps_mode = gps_mode_3D then
    let h = a.alt -. float (Srtm.of_utm a.pos) in
    let east = a.pos.utm_x +. h *. tan (a.cam.phi -. a.roll)
    and north = a.pos.utm_y +. h *. tan (a.cam.theta +. a.pitch) in
    let values = ["east", Pprz.Float east; "north", Pprz.Float north] in
    Ground_Pprz.message_send my_id "CAM_STATUS" values


let send_aircraft_msg = fun ac ->
  try
    let sof = fun f -> sprintf "%.1f" f in
    let a = Hashtbl.find aircrafts ac in
    let f = fun x -> Pprz.Float x in
    let values = ["ac_id", Pprz.String ac;
		  "roll", f (Geometry_2d.rad2deg a.roll);
		  "pitch", f (Geometry_2d.rad2deg a.pitch);
		  "east", f a.pos.utm_x;
		  "north", f a.pos.utm_y;
		  "speed", f a.gspeed;
		  "course", f (Geometry_2d.rad2deg a.course);
		  "alt", f a.alt;
		  "climb", f a.climb] in
    Ground_Pprz.message_send my_id "FLIGHT_PARAM" values;

    let values = ["ac_id", Pprz.String ac; 
		  "cur_block", Pprz.Int a.cur_block;
		  "cur_stage", Pprz.Int a.cur_stage;
		  "target_east", f (a.nav_ref_east+.a.desired_east);
		  "target_north", f (a.nav_ref_north+.a.desired_north)] in
    Ground_Pprz.message_send my_id "NAV_STATUS" values;

    let values = ["ac_id", Pprz.String ac; 
		  "throttle", f a.throttle;
		  "rpm", f a.rpm;
		  "temp", f a.temp;
		  "bat", f a.bat;
		  "amp", f a.amp;
		  "energy", f a.energy] in
    Ground_Pprz.message_send my_id "ENGINE_STATUS" values;

    let values = ["ac_id", Pprz.String ac; 
		  "mode", Pprz.Int a.ap_mode; 
		  "v_mode", Pprz.Int a.ap_altitude;
		  "gps_mode", Pprz.Int a.gps_mode] in
    Ground_Pprz.message_send my_id "AP_STATUS" values;

    send_cam_status a
  with
    Not_found -> prerr_endline ac
  | x -> prerr_endline (Printexc.to_string x)
      
let new_aircraft = fun id ->
    { id = id ; roll = 0.; pitch = 0.; nav_ref_east = 0.; nav_ref_north = 0.; desired_east = 0.; desired_north = 0.; gspeed=0.; course = 0.; alt=0.; climb=0.; cur_block=0; cur_stage=0; throttle = 0.; rpm = 0.; temp = 0.; bat = 0.; amp = 0.; energy = 0.; ap_mode=0; ap_altitude=0; if_calib_mode=0; mcu1_status=0; lls_calib=0; gps_mode =0;
      pos = { utm_x = 0.; utm_y = 0.; utm_zone = 0 };
      cam = { phi = 0.; theta = 0. }
    }
    
let register_aircraft = fun name a ->
  Hashtbl.add aircrafts name a;
  ignore (Glib.Timeout.add aircraft_msg_period (fun () -> send_aircraft_msg name; true))


(** Identifying message from a A/C *)
let ident_msg = fun log id name ->
  if not (Hashtbl.mem aircrafts name) then begin
    let ac = new_aircraft id in
    let b = Ivy.bind (fun _ args -> ac_msg log name ac args.(0)) (sprintf "^%s +(.*)" id) in
    register_aircraft name ac;
    Ground_Pprz.message_send my_id "NEW_AIRCRAFT" ["ac_id", Pprz.String id]
  end

(* Waits for new aircrafts *)
let listen_acs = fun log ->
  ignore (Ivy.bind (fun _ args -> ident_msg log args.(0) args.(1)) "^(.*) IDENT +(.*)")

let send_config = fun _asker args ->
  match args with
    ["ac_id", Pprz.String ac_id] -> begin
      try
	let conf = ExtXml.child conf_xml "aircraft" ~select:(fun x -> ExtXml.attrib x "ac_id" = ac_id) in
	let prefix = fun s -> sprintf "file://%s/conf/%s" Env.paparazzi_home s in
	let fp = prefix (ExtXml.attrib conf "flight_plan")
	and af = prefix (ExtXml.attrib conf "airframe")
	and rc = prefix (ExtXml.attrib conf "radio") in
	["ac_id", Pprz.String ac_id;
	 "flight_plan", Pprz.String fp;
	 "airframe", Pprz.String af;
	 "radio", Pprz.String rc]
      with
	Not_found ->
	  failwith (sprintf "ground UNKNOWN %s" ac_id)     
    end
  | _ ->
      let s = String.concat " " (List.map (fun (a,v) -> a^"="^Pprz.string_of_value v) args) in
      failwith (sprintf "Error, Receive.send_config: %s" s)
    
let ivy_server = fun () ->
  ignore (Ground_Pprz.message_answerer my_id "AIRCRAFTS" send_aircrafts_msg);
  ignore (Ground_Pprz.message_answerer my_id "CONFIG" send_config)



(* main loop *)
let _ =
  let xml_ground = ExtXml.child conf_xml "ground" in
  let ivy_bus = ref (ExtXml.attrib xml_ground "ivy_bus") in
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), (sprintf "Bus\tDefault is %s" !ivy_bus)] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: ";

  Srtm.add_path srtm_path;

  Ivy.init "Paparazzi receive" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (* Opens the log file *)
  let log = logger () in

  (* Waits for new simulated aircrafts *)
  listen_acs log;

  (* Waits for client requests on the Ivy bus *)
  ivy_server ();
  
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
