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
let max_nb_dl_setting_values = 256 (** indexed iwth an uint8 (messages.xml)  *)

open Printf
open Latlong
open Aircraft
module U = Unix

module Tele_Class = struct let name = "telemetry" end
module Ground = struct let name = "ground" end
module Tele_Pprz = Pprz.Messages(Tele_Class)
module Ground_Pprz = Pprz.Messages(Ground)
module Alerts_Pprz = Pprz.Messages(struct let name = "alert" end)


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
let ap_modes = [|"MANUAL";"AUTO1";"AUTO2";"HOME";"NOGPS"|]
let _AUTO2 = 2
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

(** The aircrafts store *)
let aircrafts = Hashtbl.create 3

(** Broadcast of the received aircrafts *)
let aircraft_msg_period = 500 (* ms *)
let wind_msg_period = 5000 (* ms *)
let aircraft_alerts_period = 1000 (* ms *)
let send_aircrafts_msg = fun _asker _values ->
  assert(_values = []);
  let names = String.concat "," (Hashtbl.fold (fun k _v r -> k::r) aircrafts []) ^ "," in
  ["ac_list", Pprz.String names]

let make_element = fun t a c -> Xml.Element (t,a,c)

let expand_ac_xml = fun ac_conf ->
  let prefix = fun s -> sprintf "%s/conf/%s" Env.paparazzi_home s in
  let parse = fun a ->
    let file = prefix (ExtXml.attrib ac_conf a) in
    try
      Xml.parse_file file
    with
      Xml.File_not_found _ ->
	prerr_endline (sprintf "File not found: %s" file);
	make_element "file_not_found" ["file",a] []
    | Xml.Error e ->
	let s = Xml.error e in
	prerr_endline (sprintf "Parse error in %s: %s" file s);
	make_element "cannot_parse" ["file",file;"error", s] [] in
  let fp = parse "flight_plan" in
  let af = parse "airframe" in
  let rc = parse "radio" in
  let children = Xml.children ac_conf@[fp; af; rc] in
  make_element (Xml.tag ac_conf) (Xml.attribs ac_conf) children
  
let log_xml = fun timeofday data_file ->
  let conf_children = 
    List.map
      (fun x ->	  if Xml.tag x = "aircraft" then expand_ac_xml x else x)
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
  | Pprz.Int32 x -> Int32.to_int x 
  | _ -> failwith "Receive.log_and_parse: int expected"

let update_waypoint = fun ac wp_id p alt ->
  Hashtbl.replace ac.waypoints wp_id {altitude = alt; wp_utm = p }


let format_string_field = fun s ->
  let s = String.copy s in
  for i = 0 to String.length s - 1 do
    match s.[i] with
      ' ' -> s.[i] <- '_'
    | _ -> ()
  done;
  s



let log_and_parse = fun logging ac_name (a:Aircraft.aircraft) msg values ->
  let s = String.concat " " (List.map (fun (_, v) -> Pprz.string_of_value v) values) in
  begin
    match logging with
      Some log ->
	let t = U.gettimeofday () -. start_time in
	fprintf log "%.2f %s %s %s\n" t ac_name msg.Pprz.name s; flush log
    | None -> ()
  end;
  let value = fun x -> try Pprz.assoc x values with Not_found -> failwith (sprintf "Error: field '%s' not found\n" x) in

  let fvalue = fun x -> 
    let f = fvalue (value x) in
      match classify_float f with
	FP_infinite | FP_nan ->
	  let msg = sprintf "Non normal number: %f in '%s %s %s'" f ac_name msg.Pprz.name s in
	  Ground_Pprz.message_send my_id "TELEMETRY_ERROR" ["ac_id", Pprz.String ac_name;"message", Pprz.String (format_string_field msg)];
	  failwith msg
      | _ -> f
  and ivalue = fun x -> ivalue (value x) in
  match msg.Pprz.name with
    "GPS" ->
      a.pos <- { utm_x = fvalue "utm_east" /. 100.;
		 utm_y = fvalue "utm_north" /. 100.;
		 utm_zone = ivalue "utm_zone" };
      a.gspeed  <- fvalue "speed" /. 100.;
      a.course  <- norm_course ((Deg>>Rad)(fvalue "course" /. 10.));

      a.agl     <- a.alt -. float (try Srtm.of_utm a.pos with _ -> 0);
      a.gps_mode <- check_index (ivalue "mode") gps_modes "GPS_MODE";
      if a.gspeed > 3. && a.ap_mode = _AUTO2 then
	Wind.update ac_name a.gspeed a.course
  | "GPS_SOL" ->
      a.gps_Pacc <- ivalue "Pacc"
  | "ESTIMATOR" ->
      a.alt     <- fvalue "z";
      a.climb   <- fvalue "z_dot"
  | "DESIRED" ->
      a.desired_east <- fvalue "desired_x";
      a.desired_north <- fvalue "desired_y";
      a.desired_altitude <- fvalue "desired_altitude";
      a.desired_climb <- fvalue "desired_climb"
  | "NAVIGATION_REF" ->
      a.nav_ref <- Some { utm_x = fvalue "utm_east"; utm_y = fvalue "utm_north"; utm_zone = ivalue "utm_zone" }
  | "ATTITUDE" ->
      a.roll <- (Deg>>Rad) (fvalue "phi");
      a.pitch <- (Deg>>Rad) (fvalue "theta")
  | "NAVIGATION" -> 
      a.cur_block <- ivalue "cur_block";
      a.cur_stage <- ivalue "cur_stage";
      a.desired_course <- norm_course ((Deg>>Rad)(fvalue "desired_course" /. 10.));
  | "BAT" ->
      a.last_bat_msg_date <- U.gettimeofday ();
      a.throttle <- fvalue "throttle" /. 9600. *. 100.;
      a.kill_mode <- ivalue "kill_auto_throttle" <> 0;
      a.flight_time <- ivalue "flight_time";
      a.rpm <- a.throttle *. 100.;
      a.bat <- fvalue "voltage" /. 10.;
      a.stage_time <- ivalue "stage_time";
      a.block_time <- ivalue "block_time";
      a.energy <- ivalue "energy";
      if a.flight_time > 0 && a.infrared.contrast_status = "WAITING" then
	a.infrared.contrast_status <- "SKIPPED"
  | "FBW_STATUS" ->
      a.bat <- fvalue "vsupply" /. 10.		
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
	then "NONE"
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
      a.cam.target <- (fvalue  "target_x", fvalue  "target_y")
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
	age = 0
      }
  | "CIRCLE" ->
      begin
	match a.nav_ref, a.horizontal_mode with
	  Some nav_ref, 2 -> (** FIXME *)
	    a.horiz_mode <- Circle (Latlong.utm_add nav_ref (fvalue "center_east", fvalue "center_north"), ivalue "radius")
	| _ -> ()
      end
  | "SEGMENT" ->
      begin
	match a.nav_ref, a.horizontal_mode with
	  Some nav_ref, 1 -> (** FIXME *)
	    let p1 = Latlong.utm_add nav_ref (fvalue "segment_east_1", fvalue "segment_north_1")
	    and p2 = Latlong.utm_add nav_ref (fvalue "segment_east_2",  fvalue "segment_north_2") in
	    a.horiz_mode <- Segment (p1, p2)
	| _ -> ()
      end
  | "SURVEY" ->
      begin
	match a.nav_ref with
	  Some nav_ref ->
	    let p1 = Latlong.utm_add nav_ref (fvalue "west", fvalue "south")
	    and p2 = Latlong.utm_add nav_ref (fvalue "east",  fvalue "north") in
	    a.survey <- Some (Latlong.of_utm WGS84 p1, Latlong.of_utm WGS84 p2)
	| None -> ()
      end
  | "CALIBRATION" ->
      a.throttle_accu <- fvalue "climb_sum_err"
  | "DL_VALUE" ->
      let i = ivalue "index" in
      if i < max_nb_dl_setting_values then begin
	a.dl_setting_values.(i) <- fvalue "value";
	a.nb_dl_setting_values <- max a.nb_dl_setting_values (i+1)
      end else
	failwith "Too much dl_setting values !!!"
  | "WP_MOVED" ->
      begin
	match a.nav_ref with
	  Some nav_ref ->
	    let p = { Latlong.utm_x = fvalue "utm_east";
		      utm_y = fvalue "utm_north";
		      utm_zone = ivalue "utm_zone" } in
	    update_waypoint a (ivalue "wp_id") p (fvalue "alt")
	| None -> () (** Can't use this message  *)
      end
  | _ -> ()

(** Callback for a message from a registered A/C *)
let ac_msg = fun log ac_name ac m ->
  try
    let (msg_id, values) = Tele_Pprz.values_of_string m in
    let msg = Tele_Pprz.message_of_id msg_id in
    log_and_parse log ac_name ac msg values
  with
    Pprz.Unknown_msg_name (x, c) ->
      fprintf stderr "Unknown message %s in class %s from %s: %s\n" x c ac_name m
  | x -> prerr_endline (Printexc.to_string x)


(** If you are 1km above the ground, an angle of 89 degrees between the vertical and
your camera axis allow you to look 57km away: it should be enough ! **)
let cam_max_angle = (Deg>>Rad) 89.

let send_cam_status = fun a ->
  if a.gps_mode = gps_mode_3D then
    match a.nav_ref with
      None -> () (* No geo ref for camera target *)
    | Some nav_ref ->
	let h = a.agl in
	let phi_absolute = a.cam.phi -. a.roll
	and theta_absolute = a.cam.theta +. a.pitch in
	if phi_absolute > -. cam_max_angle && phi_absolute < cam_max_angle &&
	  theta_absolute > -. cam_max_angle && theta_absolute < cam_max_angle then
	  let dx = h *. tan phi_absolute
	  and dy = h *. tan theta_absolute in
	  let alpha = -. a.course in
	  let east = dx *. cos alpha -. dy *. sin alpha
	  and north = dx *. sin alpha +. dy *. cos alpha in
	  let utm = Latlong.utm_add a.pos (east, north) in
	  let wgs84 = Latlong.of_utm WGS84 utm in
	  let utm_target = Latlong.utm_add nav_ref a.cam.target in
	  let twgs84 =  Latlong.of_utm WGS84 utm_target in
	  let values = ["ac_id", Pprz.String a.id; 
			"cam_lat", Pprz.Float ((Rad>>Deg)wgs84.posn_lat);
			"cam_long", Pprz.Float ((Rad>>Deg)wgs84.posn_long); 
			"cam_target_lat", Pprz.Float ((Rad>>Deg)twgs84.posn_lat);
			"cam_target_long", Pprz.Float ((Rad>>Deg)twgs84.posn_long)] in
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

let send_dl_values = fun a ->
  if a.nb_dl_setting_values > 0 then
    let csv = ref "" in
    for i = 0 to a.nb_dl_setting_values - 1 do
      csv := sprintf "%s%f," !csv a.dl_setting_values.(i)
    done;
    let vs = ["ac_id", Pprz.String a.id; "values", Pprz.String !csv] in
    Ground_Pprz.message_send my_id "DL_VALUES" vs

let send_svsinfo = fun a ->
  let svid = ref ""
  and flags= ref ""
  and qi = ref ""
  and cno = ref ""
  and elev = ref ""
  and azim = ref ""
  and age = ref "" in
  let concat = fun ref v ->
    ref := !ref ^ string_of_int v ^ "," in
  for i = 0 to gps_nb_channels - 1 do
    concat svid a.svinfo.(i).svid;
    concat flags a.svinfo.(i).flags;
    concat qi a.svinfo.(i).qi;
    concat cno a.svinfo.(i).cno;
    concat elev a.svinfo.(i).elev;
    concat azim a.svinfo.(i).azim;
    concat age a.svinfo.(i).age
  done;
  let f = fun s r -> (s, Pprz.String !r) in
  let vs = ["ac_id", Pprz.String a.id; 
	    "pacc", Pprz.Int a.gps_Pacc;
	    f "svid" svid; f "flags" flags; f "qi" qi;  f "msg_age" age; 
	    f "cno" cno; f "elev" elev; f "azim" azim] in
  Ground_Pprz.message_send my_id "SVSINFO" vs

let send_horiz_status = fun a ->
  match a.horiz_mode with
    Circle (utm, r) ->
      let wgs84 = Latlong.of_utm WGS84 utm in
      let vs = [ "ac_id", Pprz.String a.id; 
		 "circle_lat", Pprz.Float ((Rad>>Deg)wgs84.posn_lat);
		 "circle_long", Pprz.Float ((Rad>>Deg)wgs84.posn_long);
		 "radius", Pprz.Int r ] in
       Ground_Pprz.message_send my_id "CIRCLE_STATUS" vs
  | Segment (u1, u2) ->
      let geo1 = Latlong.of_utm WGS84 u1 in
      let geo2 = Latlong.of_utm WGS84 u2 in
      let vs = [ "ac_id", Pprz.String a.id; 
		 "segment1_lat", Pprz.Float ((Rad>>Deg)geo1.posn_lat);
		 "segment1_long", Pprz.Float ((Rad>>Deg)geo1.posn_long);
		 "segment2_lat", Pprz.Float ((Rad>>Deg)geo2.posn_lat);
		 "segment2_long", Pprz.Float ((Rad>>Deg)geo2.posn_long) ] in
      Ground_Pprz.message_send my_id "SEGMENT_STATUS" vs
  | UnknownHorizMode -> ()

let send_survey_status = fun a ->
  match a.survey with
    None -> ()
  | Some (geo1, geo2) ->
      let vs = [ "ac_id", Pprz.String a.id; 
		 "south_lat", Pprz.Float ((Rad>>Deg)geo1.posn_lat);
		 "west_long", Pprz.Float ((Rad>>Deg)geo1.posn_long);
		 "north_lat", Pprz.Float ((Rad>>Deg)geo2.posn_lat);
		 "east_long", Pprz.Float ((Rad>>Deg)geo2.posn_long) ] in
      Ground_Pprz.message_send my_id "SURVEY_STATUS" vs
      


let send_wind = fun a ->
  let id = a.id in
  try
    let (wind_cap_rad, wind_polar, mean, stddev, _nb_sample) = Wind.get id in
    let vs =
      ["ac_id", Pprz.String id;
       "dir", Pprz.Float ((Rad>>Deg)wind_cap_rad);
       "wspeed", Pprz.Float wind_polar;
       "mean_aspeed", Pprz.Float mean;
       "stddev", Pprz.Float stddev] in
    Ground_Pprz.message_send my_id "WIND" vs
  with
    _exc -> ()

let send_telemetry_status = fun a ->
  let id = a.id in
  try
    let vs =
      ["ac_id", Pprz.String id;
       "time_since_last_bat_msg", Pprz.Float (U.gettimeofday () -. a.last_bat_msg_date)] in
    Ground_Pprz.message_send my_id "TELEMETRY_STATUS" vs
  with
    _exc -> ()

let send_moved_waypoints = fun a ->
  Hashtbl.iter
    (fun wp_id wp ->
      let geo = Latlong.of_utm WGS84 wp.wp_utm in
      let vs =
	["ac_id", Pprz.String a.id;
	 "wp_id", Pprz.Int wp_id;
	 "long", Pprz.Float ((Rad>>Deg)geo.posn_long);
	 "lat", Pprz.Float ((Rad>>Deg)geo.posn_lat);
	 "alt", Pprz.Float wp.altitude] in
      Ground_Pprz.message_send my_id "WAYPOINT_MOVED" vs)
    a.waypoints
	 

let send_aircraft_msg = fun ac ->
  try
    let a = Hashtbl.find aircrafts ac in
    let f = fun x -> Pprz.Float x in
    let wgs84 = Latlong.of_utm WGS84 a.pos in
    let values = ["ac_id", Pprz.String ac;
		  "roll", f (Geometry_2d.rad2deg a.roll);
		  "pitch", f (Geometry_2d.rad2deg a.pitch);
		  "lat", f ((Rad>>Deg)wgs84.posn_lat);
		  "long", f ((Rad>>Deg) wgs84.posn_long);
		  "speed", f a.gspeed;
		  "course", f (Geometry_2d.rad2deg a.course);
		  "alt", f a.alt;
		  "agl", f a.agl;
		  "climb", f a.climb] in
    Ground_Pprz.message_send my_id "FLIGHT_PARAM" values;

    begin
      match a.nav_ref with
	Some nav_ref ->
	  let target_utm = Latlong.utm_add nav_ref (a.desired_east, a.desired_north) in
	  let target_wgs84 = Latlong.of_utm WGS84 target_utm in 
	  let values = ["ac_id", Pprz.String ac; 
			"cur_block", Pprz.Int a.cur_block;
			"cur_stage", Pprz.Int a.cur_stage;
			"stage_time", Pprz.Int a.stage_time;
			"block_time", Pprz.Int a.block_time;
			"target_lat", f ((Rad>>Deg)target_wgs84.posn_lat);
			"target_long", f ((Rad>>Deg)target_wgs84.posn_long);
			"target_alt", Pprz.Float a.desired_altitude;
			"target_climb", Pprz.Float a.desired_climb;
			"target_course", Pprz.Float ((Rad>>Deg)a.desired_course)
		      ] in
	  Ground_Pprz.message_send my_id "NAV_STATUS" values
      | None -> () (* No nav_ref yet *)
    end;

    let values = ["ac_id", Pprz.String ac; 
		  "throttle", f a.throttle;
		  "throttle_accu", f a.throttle_accu;
		  "rpm", f a.rpm;
		  "temp", f a.temp;
		  "bat", f a.bat;
		  "amp", f a.amp;
		  "energy", Pprz.Int a.energy] in
    Ground_Pprz.message_send my_id "ENGINE_STATUS" values;
    
    let ap_mode = get_indexed_value ap_modes a.ap_mode in
    let gaz_mode = get_indexed_value gaz_modes a.gaz_mode in
    let lat_mode = get_indexed_value lat_modes a.lateral_mode in
    let horiz_mode = get_indexed_value horiz_modes a.horizontal_mode in
    let gps_mode = get_indexed_value gps_modes a.gps_mode
    and kill_mode = if a.kill_mode then "ON" else "OFF" in
    let values = ["ac_id", Pprz.String ac; 
		  "flight_time", Pprz.Int a.flight_time;
		  "ap_mode", Pprz.String ap_mode; 
		  "gaz_mode", Pprz.String gaz_mode;
		  "lat_mode", Pprz.String lat_mode;
		  "horiz_mode", Pprz.String horiz_mode;
		  "gps_mode", Pprz.String gps_mode;
		  "kill_mode", Pprz.String kill_mode
		] in
    Ground_Pprz.message_send my_id "AP_STATUS" values;

    send_cam_status a;
    send_if_calib a;
    send_fbw a;
    send_infrared a;
    send_svsinfo a;
    send_horiz_status a;
    send_survey_status a;
    send_dl_values a;
    send_moved_waypoints a;
    send_telemetry_status a
  with
    Not_found -> prerr_endline ac
  | x -> prerr_endline (Printexc.to_string x)
      
let new_aircraft = fun id ->
  let infrared_init = { gps_hybrid_mode = 0; gps_hybrid_factor = 0. ;
			contrast_status = "DEFAULT"; contrast_value = 0} in
  let svsinfo_init = Array.init gps_nb_channels (fun _ -> svinfo_init ()) in
  let update = fun () ->
    for i = 0 to Array.length svsinfo_init - 1 do
      svsinfo_init.(i).age <-  svsinfo_init.(i).age + 1;
    done in

  ignore (Glib.Timeout.add 1000 (fun _ -> update (); true));
  { id = id ; roll = 0.; pitch = 0.; desired_east = 0.; desired_north = 0.; 
    desired_course = 0.;
    gspeed=0.; course = 0.; alt=0.; climb=0.; cur_block=0; cur_stage=0;
    throttle = 0.; throttle_accu = 0.; rpm = 0.; temp = 0.; bat = 42.; amp = 0.; energy = 0; ap_mode= -1; agl = 0.;
    gaz_mode= -1; lateral_mode= -1;
    gps_mode =0; gps_Pacc = 0; periodic_callbacks = [];
    desired_altitude = 0.;
    desired_climb = 0.;
    pos = { utm_x = 0.; utm_y = 0.; utm_zone = 0 };
    nav_ref = None;
    cam = { phi = 0.; theta = 0. ; target=(0.,0.)};
    inflight_calib = { if_mode = 1 ; if_val1 = 0.; if_val2 = 0.};
    infrared = infrared_init; kill_mode = false;
    fbw = { rc_status = "???"; rc_mode = "???" };
    svinfo = svsinfo_init;
    dl_setting_values = Array.create max_nb_dl_setting_values 42.;
      nb_dl_setting_values = 0;
    flight_time = 0; stage_time = 0; block_time = 0;
    horiz_mode = UnknownHorizMode;
    horizontal_mode = 0;
    waypoints = Hashtbl.create 3; survey = None; last_bat_msg_date = 0.
  }

let check_alerts = fun a ->
  let send = fun level ->
    let vs = [ "ac_id", Pprz.String a.id; 
	       "level", Pprz.String level; 
	       "value", Pprz.Float a.bat] in
    Alerts_Pprz.message_send my_id "BAT_LOW" vs in
  if a.bat < 9. then send "CATASTROPHIC"
  else if a.bat < 10. then send "CRITIC"
  else if a.bat < 10.5 then send "WARNING"

let wind_clear = fun _sender vs ->
  Wind.clear (Pprz.string_assoc "ac_id" vs)

let periodic = fun period cb ->
  Glib.Timeout.add period (fun () -> cb (); true)


let register_periodic = fun ac x ->
      ac.periodic_callbacks <- x :: ac.periodic_callbacks

(** add the periodic airprox check for the aircraft (name) on all aircraft     *)
(**    already known                                                           *)
let periodic_airprox_check = fun name ->
  let thisac = Hashtbl.find aircrafts name in
  let aircrafts2list = fun () ->
    let list = ref [] in
    Hashtbl.iter (fun name _a -> list := name :: !list) aircrafts;
    !list in
  let ac_names = List.filter (fun a -> a <> name) (aircrafts2list ()) in
  let list_ac = List.map (fun name -> Hashtbl.find aircrafts name) ac_names in

  let check_airprox = fun ac ->
    try
      match Airprox.check_airprox thisac ac with
	None -> ()
      | Some level ->
	  let vs =
	    ["ac_id", Pprz.String (thisac.id ^ "," ^ ac.id) ; "level", Pprz.String level] in
	  Alerts_Pprz.message_send my_id "AIR_PROX" vs
    with
      x -> fprintf stderr "check_airprox: %s\n%!" (Printexc.to_string x)

 in
  
  List.iter 
    (fun ac ->
      register_periodic thisac (periodic aircraft_alerts_period (fun () -> check_airprox ac))
    ) list_ac

let register_aircraft = fun name a ->
  Hashtbl.add aircrafts name a;
  register_periodic a (periodic aircraft_msg_period (fun () -> send_aircraft_msg name));
  register_periodic a (periodic aircraft_alerts_period (fun () -> check_alerts a));
  periodic_airprox_check name;
  register_periodic a (periodic wind_msg_period (fun () -> send_wind a));
  Wind.new_ac name 1000;
  ignore(Ground_Pprz.message_bind "WIND_CLEAR" wind_clear)


(** Identifying message from an A/C *)
let ident_msg = fun log name ->
  if not (Hashtbl.mem aircrafts name) then begin
    let ac = new_aircraft name in
    let _b = Ivy.bind (fun _ args -> ac_msg log name ac args.(0)) (sprintf "^%s +(.*)" name) in
    register_aircraft name ac;
    Ground_Pprz.message_send my_id "NEW_AIRCRAFT" ["ac_id", Pprz.String name]
  end

let new_color = fun () ->
  sprintf "#%02x%02x%02x" (Random.int 256) (Random.int 256) (Random.int 256)

(* Waits for new aircrafts *)
let listen_acs = fun log ->
  (** Wait for any message (they all are identified with the A/C) *)
  ignore (Ivy.bind (fun _ args -> ident_msg log args.(0)) "^(.*) PPRZ_MODE")

(** c.f. sw/logalizer/play.ml *)
let replayed = fun s ->
  let n = String.length s in
  if n > 6 && String.sub s 0 6 = "replay" then
    Some (String.sub s 6 (n - 6))
  else
    None

let send_config = fun http _asker args ->
  let ac_id' = Pprz.string_assoc "ac_id" args in
  try
    let ac_id, root_dir, conf_xml =
      match replayed ac_id' with
	Some ac_id -> 
	  ac_id, "var/replay/", Xml.parse_file (Env.paparazzi_home // "var/replay/conf/conf.xml")
      | None -> ac_id', "", conf_xml in

    let conf = ExtXml.child conf_xml "aircraft" ~select:(fun x -> ExtXml.attrib x "ac_id" = ac_id) in
    let ac_name = ExtXml.attrib conf "name" in
    let protocol =
      if http then
	sprintf "http://%s:8889" (Unix.gethostname ())
      else
	sprintf "file://%s" Env.paparazzi_home in
    let prefix = fun s -> sprintf "%s/%s%s" protocol root_dir s in

    (** Expanded flight plan and settings have been compiled in var/ *)
    let fp = prefix ("var" // ac_name // "flight_plan.xml")
    and af = prefix ("conf" // ExtXml.attrib conf "airframe")
    and rc = prefix ("conf" // ExtXml.attrib conf "radio")
    and settings = prefix  ("var" // ac_name // "settings.xml") in
    let col = try Xml.attrib conf "gui_color" with _ -> new_color () in
    let ac_name = try Xml.attrib conf "name" with _ -> "" in
    ["ac_id", Pprz.String ac_id;
     "flight_plan", Pprz.String fp;
     "airframe", Pprz.String af;
     "radio", Pprz.String rc;
     "settings", Pprz.String settings;
     "default_gui_color", Pprz.String col;
     "ac_name", Pprz.String ac_name
   ]
  with
    Not_found ->
      failwith (sprintf "ground UNKNOWN %s" ac_id')     
    
let ivy_server = fun http ->
  ignore (Ground_Pprz.message_answerer my_id "AIRCRAFTS" send_aircrafts_msg);
  ignore (Ground_Pprz.message_answerer my_id "CONFIG" (send_config http))



(* main loop *)
let _ =
  let ivy_bus = ref "127.255.255.255:2010"
  and logging = ref true
  and http = ref false in

  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), (sprintf "Bus\tDefault is %s" !ivy_bus);
      "-n", Arg.Clear logging, "Disable log";
      "-http", Arg.Set http, "Send http: URLs (default is file:)"] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "%s: Warning: Don't do anything with '%s' argument\n" Sys.argv.(0) x)
    "Usage: ";

  Srtm.add_path srtm_path;

  Ivy.init "Paparazzi server" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  if !logging then
    (* Opens the log file *)
    let log = logger () in
    (* Waits for new simulated aircrafts *)
    listen_acs (Some log)
  else
    listen_acs None;
  (* Waits for client requests on the Ivy bus *)
  ivy_server !http;
  
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
