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


let rec norm_course =
  let _2pi = 2. *. Latlong.pi in
  fun c ->
    if c < 0. then norm_course (c +. _2pi)
    else if c >= _2pi then norm_course (c -. _2pi)
    else c

(** FIXME: Should be read from messages.xml *)
let ap_modes = [|"MANUAL";"AUTO1";"AUTO2";"HOME"|]
let gaz_modes = [|"MANUAL";"GAZ";"CLIMB";"ALT"|]
let lat_modes = [|"MANUAL";"ROLL_RATE";"ROLL";"COURSE"|]
let gps_modes = [|"NOFIX";"DRO";"2D";"3D";"GPSDRO"|]
let gps_hybrid_modes = [|"OFF";"ON"|]
let if_modes = [|"OFF";"DOWN";"UP"|]
let horiz_modes = [|"WAYPOINT";"ROUTE";"CIRCLE"|]

let check_index = fun i t where ->
  if i < 0 || i >= Array.length t then begin
    Debug.call 'E' (fun f -> fprintf f "Wrong index in %s: %d" where i);
    -1
  end else
    i

let get_indexed_value = fun t i ->
  if i >= 0 then t.(i) else "UNK"
  
type ac_cam = {
    mutable phi : float; (* Rad, right = >0 *)
    mutable theta : float; (* Rad, front = >0 *)
  }

type inflight_calib = {
    mutable if_mode : int; (* DOWN|OFF|UP *)
    mutable if_val1 : float;
    mutable if_val2 : float
  }

type contrast_status = string (** DEFAULT|WAITING|SET *)
type infrared = {
    mutable gps_hybrid_mode : int;
    mutable gps_hybrid_factor : float;
    mutable contrast_status : contrast_status;
    mutable contrast_value : int
  }

type rc_status = string (** OK, LOST, REALLY_LOST *)
type rc_mode = string (** MANUAL, AUTO, FAILSAFE *)
type fbw = {
    mutable rc_status : rc_status;
    mutable rc_mode : rc_mode;
  }

let gps_nb_channels = 16
type svinfo = {  
    svid : int;
    flags : int;
    qi : int;
    cno : int;
    elev : int;
    azim : int
  }

let svinfo_init = {  
    svid = 0 ;
    flags = 0;
    qi = 0;
    cno = 0;
    elev = 0;
    azim = 0;
  }

type horiz_mode = 
    Circle of utm * int
  | Segment of utm * utm
  | UnknownHorizMode

type aircraft = { 
    id : string;
    mutable pos : Latlong.utm;
    mutable roll    : float;
    mutable pitch   : float;
    mutable nav_ref    : Latlong.utm;
    mutable desired_east    : float;
    mutable desired_north    : float;
    mutable desired_altitude    : float;
    mutable desired_course : float;
    mutable desired_climb : float;
    mutable gspeed  : float;
    mutable course  : float;
    mutable alt     : float;
    mutable climb   : float;
    mutable cur_block : int;
    mutable cur_stage : int;
    mutable throttle : float;
    mutable throttle_accu : float;
    mutable rpm  : float;
    mutable temp : float;
    mutable bat  : float;
    mutable amp : float;
    mutable energy  : float;
    mutable ap_mode : int;
    mutable gaz_mode : int;
    mutable lateral_mode : int;
    mutable horizontal_mode : int;
    cam : ac_cam;
    mutable gps_mode : int;
    inflight_calib : inflight_calib;
    infrared : infrared;
    fbw : fbw;
    svinfo : svinfo array;
    mutable flight_time : int;
    mutable stage_time : int;
    mutable block_time : int;
    mutable horiz_mode : horiz_mode
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

let make_element = fun t a c -> Xml.Element (t,a,c)

let expand_ac_xml = fun ac_conf ->
  let prefix = fun s -> sprintf "%s/conf/%s" Env.paparazzi_home s in
  let parse = fun a ->
    try
      Xml.parse_file (prefix (ExtXml.attrib ac_conf a))
    with
      Xml.File_not_found _ -> make_element "file_not_found" ["file",a] [] in
  let fp = parse "flight_plan"
  and af = parse "airframe"
  and rc = parse "radio" in
  let children = Xml.children ac_conf@[fp; af; rc] in
  make_element (Xml.tag ac_conf) (Xml.attribs ac_conf) children
  
let log_xml = fun timeofday data_file ->
  let conf_children = 
    List.map
      (fun x ->	if Xml.tag x = "aircraft" then expand_ac_xml x else x)
      (Xml.children conf_xml) in
  let expanded_conf = make_element (Xml.tag conf_xml) (Xml.attribs conf_xml) conf_children in
  make_element 
    "configuration"
    ["time_of_day", string_of_float timeofday; "data_file", data_file]
    [expanded_conf; Pprz.messages_xml ()]
			 
  
let start_time = U.gettimeofday ()

(* Opens the log files *)
let logger = fun () ->
  let d = U.localtime start_time in
  let basename = sprintf "%02d_%02d_%02d__%02d_%02d_%02d" (d.U.tm_year mod 100) (d.U.tm_mon+1) (d.U.tm_mday) (d.U.tm_hour) (d.U.tm_min) (d.U.tm_sec) in
  if not (Sys.file_exists logs_path) then begin
    printf "Creating '%s'\n" logs_path; flush stdout;
    Unix.mkdir logs_path 0o755
  end;
  let log_name = sprintf "%s.log" basename
  and data_name = sprintf "%s.data" basename in
  let f = open_out (logs_path // log_name) in
  output_string f (Xml.to_string_fmt (log_xml start_time data_name));
  close_out f;
  open_out (logs_path // data_name)

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
  let t = U.gettimeofday () -. start_time in
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
	a.gspeed  <- fvalue "speed" /. 100.;
	a.course  <- norm_course ((Deg>>Rad)(fvalue "course" /. 10.));
	a.alt     <- fvalue "alt" /. 100.;
	a.climb   <- fvalue "climb" /. 100.;
	a.gps_mode <- check_index (ivalue "mode") gps_modes "GPS_MODE"
    | "DESIRED" ->
	a.desired_east <- fvalue "desired_x";
	a.desired_north <- fvalue "desired_y";
	a.desired_altitude <- fvalue "desired_altitude";
	a.desired_climb <- fvalue "desired_climb"
    | "NAVIGATION_REF" ->
	a.nav_ref <- { utm_x = fvalue "utm_east"; utm_y = fvalue "utm_north"; utm_zone = a.pos.utm_zone }
    | "ATTITUDE" ->
	a.roll <- (Deg>>Rad) (fvalue "phi");
	a.pitch <- (Deg>>Rad) (fvalue "theta")
    | "NAVIGATION" -> 
	a.cur_block <- ivalue "cur_block";
	a.cur_stage <- ivalue "cur_stage";
	a.desired_course <- norm_course ((Deg>>Rad)(fvalue "desired_course" /. 10.))
    | "BAT" ->
	a.throttle <- fvalue "desired_gaz" /. 9600. *. 100.;
	a.flight_time <- ivalue "flight_time";
	a.rpm <- a.throttle *. 100.;
	a.bat <- fvalue "voltage" /. 10.;
	a.stage_time <- ivalue "stage_time";
	a.block_time <- ivalue "block_time";
	if a.flight_time > 0 && a.infrared.contrast_status = "WAITING" then
	  a.infrared.contrast_status <- "SKIPPED"
    | "PPRZ_MODE" ->
	a.ap_mode <- check_index (ivalue "ap_mode") ap_modes "AP_MODE";
	a.gaz_mode <- check_index (ivalue "ap_gaz") gaz_modes "AP_GAZ";
	a.lateral_mode <- check_index (ivalue "ap_lateral") lat_modes "AP_LAT";
	a.horizontal_mode <- check_index (ivalue "ap_horizontal") horiz_modes "AP_HORIZ";
	a.inflight_calib.if_mode <- check_index (ivalue "if_calib_mode") if_modes "IF_MODE";
	let mcu1_status = ivalue "mcu1_status" in
	(** c.f. link_autopilot.h *)
	a.fbw.rc_status <- 
	  if mcu1_status land 0b1 > 0
	  then "OK"
	  else if mcu1_status land 0b10 > 0
	  then "REALLY_LOST"
	  else "LOST";
	a.fbw.rc_mode <-
	  if mcu1_status land 0b1000 > 0
	  then "FAILSAFE"
	  else if mcu1_status land 0b100 > 0
	  then "AUTO"
	  else "MANUAL";
	a.infrared.gps_hybrid_mode <- check_index (ivalue "lls_calib") gps_hybrid_modes "HYBRID MODE"
    | "CAM" ->
	a.cam.phi <- (Deg>>Rad) (fvalue  "phi");
	a.cam.theta <- (Deg>>Rad) (fvalue  "theta");
    | "RAD_OF_IR" ->
	a.infrared.gps_hybrid_factor <- fvalue "rad_of_ir"
    | "CALIB_START" ->
	a.infrared.contrast_status <- "WAITING"
    | "CALIB_CONTRAST" ->
	a.infrared.contrast_value <- ivalue "adc";
	a.infrared.contrast_status <- "SET"
    | "SETTINGS" ->
	a.inflight_calib.if_val1 <- fvalue "slider_1_val";
	a.inflight_calib.if_val2 <- fvalue "slider_2_val";
    | "SVINFO" ->
	let i = ivalue "chn" in
	assert(i < Array.length a.svinfo);
	a.svinfo.(i) <- {  
	  svid = ivalue "SVID";
	  flags = ivalue "Flags";
	  qi = ivalue "QI";
	  cno = ivalue "CNO";
	  elev = ivalue "Elev";
	  azim = ivalue "Azim";
	}
    | "CIRCLE" ->
	a.horiz_mode <- Circle (Latlong.utm_add a.nav_ref (fvalue "center_east") (fvalue "center_north"), ivalue "radius")
    | "SEGMENT" ->
	let p1 = Latlong.utm_add a.nav_ref (fvalue "segment_east_1") (fvalue "segment_north_1")
	and p2 = Latlong.utm_add a.nav_ref (fvalue "segment_east_2") (fvalue "segment_north_2") in
	a.horiz_mode <- Segment (p1, p2)
    | "CALIBRATION" ->
	a.throttle_accu <- fvalue "climb_sum_err"
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
    let dx = h *. tan (a.cam.phi -. a.roll)
    and dy = h *. tan (a.cam.theta +. a.pitch) in
    let alpha = -. a.course in
    let east = a.pos.utm_x +. dx *. cos alpha -. dy *. sin alpha
    and north = a.pos.utm_y +. dx *. sin alpha +. dy *. cos alpha in
    let values = ["ac_id", Pprz.String a.id; "cam_east", Pprz.Float east; "cam_north", Pprz.Float north] in
    Ground_Pprz.message_send my_id "CAM_STATUS" values

let send_if_calib = fun a ->
  let if_mode = get_indexed_value if_modes a.inflight_calib.if_mode in
  let values = ["ac_id", Pprz.String a.id;
		"if_mode", Pprz.String if_mode;
		"if_value1", Pprz.Float a.inflight_calib.if_val1;
		"if_value2", Pprz.Float a.inflight_calib.if_val2] in
  Ground_Pprz.message_send my_id "INFLIGH_CALIB" values

let send_fbw = fun a ->
  let values = [ "ac_id", Pprz.String a.id;
		 "rc_mode", Pprz.String a.fbw.rc_mode;
		 "rc_status", Pprz.String a.fbw.rc_status ] in
  Ground_Pprz.message_send my_id "FLY_BY_WIRE"  values

let send_infrared = fun a ->
  let gps_hybrid_mode = get_indexed_value gps_hybrid_modes a.infrared.gps_hybrid_mode in
  let values = [ "ac_id", Pprz.String a.id;
		 "gps_hybrid_mode", Pprz.String gps_hybrid_mode;
		 "gps_hybrid_factor", Pprz.Float a.infrared.gps_hybrid_factor;
		 "contrast_status", Pprz.String a.infrared.contrast_status;
		 "contrast_value", Pprz.Int a.infrared.contrast_value
	       ] in
  Ground_Pprz.message_send my_id "INFRARED"  values

let send_svsinfo = fun a ->
  let svid = ref ""
  and flags= ref ""
  and qi = ref ""
  and cno = ref ""
  and elev = ref ""
  and azim = ref "" in
  for i = 0 to gps_nb_channels - 1 do
    let concat = fun ref v ->
      ref := !ref ^ string_of_int v ^ "," in
    concat svid a.svinfo.(i).svid;
    concat flags a.svinfo.(i).flags;
      concat qi a.svinfo.(i).qi;
    concat cno a.svinfo.(i).cno;
    concat elev a.svinfo.(i).elev;
    concat azim a.svinfo.(i).azim
  done;
  let f = fun s r -> (s, Pprz.String !r) in
  let vs = ["ac_id", Pprz.String a.id; 
	    f "svid" svid; f "flags" flags; f "qi" qi; 
	    f "cno" cno; f "elev" elev; f "azim" azim] in
  Ground_Pprz.message_send my_id "SVSINFO" vs

let send_horiz_status = fun a ->
  match a.horiz_mode with
    Circle (utm, r) ->
      let vs = [ "ac_id", Pprz.String a.id; 
		 "circle_east", Pprz.Float utm.utm_x;
		 "circle_north", Pprz.Float utm.utm_y;
		 "radius", Pprz.Int r ] in
       Ground_Pprz.message_send my_id "CIRCLE_STATUS" vs
  | Segment (u1, u2) ->
      let vs = [ "ac_id", Pprz.String a.id; 
		 "segment1_east", Pprz.Float u1.utm_x;
		 "segment1_north", Pprz.Float u1.utm_y;
		 "segment2_east", Pprz.Float u2.utm_x;
		 "segment2_north", Pprz.Float u2.utm_y ] in
      Ground_Pprz.message_send my_id "SEGMENT_STATUS" vs
  | UnknownHorizMode -> ()


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
		  "stage_time", Pprz.Int a.stage_time;
		  "block_time", Pprz.Int a.block_time;
		  "target_east", f (a.nav_ref.utm_x+.a.desired_east);
		  "target_north", f (a.nav_ref.utm_y+.a.desired_north);
		  "target_alt", Pprz.Float a.desired_altitude;
		  "target_climb", Pprz.Float a.desired_climb;
		  "target_course", Pprz.Float ((Rad>>Deg)a.desired_course)
		] in
    Ground_Pprz.message_send my_id "NAV_STATUS" values;

    let values = ["ac_id", Pprz.String ac; 
		  "throttle", f a.throttle;
		  "throttle_accu", f a.throttle_accu;
		  "rpm", f a.rpm;
		  "temp", f a.temp;
		  "bat", f a.bat;
		  "amp", f a.amp;
		  "energy", f a.energy] in
    Ground_Pprz.message_send my_id "ENGINE_STATUS" values;
    
    let ap_mode = get_indexed_value ap_modes a.ap_mode in
    let gaz_mode = get_indexed_value gaz_modes a.gaz_mode in
    let lat_mode = get_indexed_value lat_modes a.lateral_mode in
    let horiz_mode = get_indexed_value horiz_modes a.horizontal_mode in
    let gps_mode = get_indexed_value gps_modes a.gps_mode in
    let values = ["ac_id", Pprz.String ac; 
		  "flight_time", Pprz.Int a.flight_time;
		  "ap_mode", Pprz.String ap_mode; 
		  "gaz_mode", Pprz.String gaz_mode;
		  "lat_mode", Pprz.String lat_mode;
		  "horiz_mode", Pprz.String horiz_mode;
		  "gps_mode", Pprz.String gps_mode] in
    Ground_Pprz.message_send my_id "AP_STATUS" values;

    send_cam_status a;
    send_if_calib a;
    send_fbw a;
    send_infrared a;
    send_svsinfo a;
    send_horiz_status a
  with
    Not_found -> prerr_endline ac
  | x -> prerr_endline (Printexc.to_string x)
      
let new_aircraft = fun id ->
  let infrared_init = { gps_hybrid_mode = 0; gps_hybrid_factor = 0. ;
			contrast_status = "DEFAULT"; contrast_value = 0} in
    { id = id ; roll = 0.; pitch = 0.; desired_east = 0.; desired_north = 0.; 
      desired_course = 0.;
	gspeed=0.; course = 0.; alt=0.; climb=0.; cur_block=0; cur_stage=0;
      throttle = 0.; throttle_accu = 0.; rpm = 0.; temp = 0.; bat = 0.; amp = 0.; energy = 0.; ap_mode= -1;
      gaz_mode= -1; lateral_mode= -1;
      gps_mode =0;
      desired_altitude = 0.;
      desired_climb = 0.;
      pos = { utm_x = 0.; utm_y = 0.; utm_zone = 0 };
      nav_ref = { utm_x = 0.; utm_y = 0.; utm_zone = 0 };
      cam = { phi = 0.; theta = 0. };
      inflight_calib = { if_mode = 1 ; if_val1 = 0.; if_val2 = 0.};
      infrared = infrared_init;
      fbw = { rc_status = "???"; rc_mode = "???" };
      svinfo = Array.create gps_nb_channels svinfo_init;
      flight_time = 0; stage_time = 0; block_time = 0;
      horiz_mode = UnknownHorizMode;
      horizontal_mode = 0
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
	let ac_name = ExtXml.attrib conf "name" in
	let prefix = fun s -> sprintf "http://%s:8889/%s" (Unix.gethostname ()) s in
	(** Expanded flight plan has been compiled in var/ *)
	let fp = prefix ("var" // ac_name // "flight_plan.xml")
	and af = prefix ("conf" // ExtXml.attrib conf "airframe")
	and rc = prefix ("conf" // ExtXml.attrib conf "radio") in
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
