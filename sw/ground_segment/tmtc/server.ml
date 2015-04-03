(*
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
let no_md5_check = ref false
let replay_old_log = ref false

open Printf
open Latlong
open Server_globals
open Aircraft
module U = Unix
module LL = Latlong

module Ground = struct let name = "ground" end
module Ground_Pprz = Pprz.Messages(Ground)
module Tm_Pprz = Pprz.Messages (struct let name = "telemetry" end)
module Alerts_Pprz = Pprz.Messages(struct let name = "alert" end)
module Dl_Pprz = Pprz.Messages (struct let name = "datalink" end)



let (//) = Filename.concat
let logs_path = Env.paparazzi_home // "var" // "logs"
let conf_xml = Xml.parse_file (Env.paparazzi_home // "conf" // "conf.xml")
let srtm_path = Env.paparazzi_home // "data" // "srtm"

let get_indexed_value = fun t i ->
  if i >= 0 then t.(i) else "UNK"

let modes_of_type = fun vt ->
  match vt with
      FixedWing -> fixedwing_ap_modes
    | Rotorcraft -> rotorcraft_ap_modes
    | UnknownVehicleType -> [| |]

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


let expand_aicraft x =
  let ac_name = ExtXml.attrib x "name" in
  try
    Env.expand_ac_xml x
  with Failure msg ->
    begin
      prerr_endline ("A failure occurred while processing aircraft '"^ac_name^"'");
      prerr_endline (" - Fail with : "^msg);
      prerr_endline (" - '"^ac_name^"' will be ignored by the server");
      prerr_endline " - Please remove it from 'conf.xml' or fix its parameter(s)";
      flush stderr;
      (*failwith msg*)
      Xml.Element ("ignoring_aircraft",["name", ac_name],[])
    end

let make_element = fun t a c -> Xml.Element (t,a,c)

let log_xml = fun timeofday data_file ->
  let conf_children =
    List.map
      (fun x ->   if Xml.tag x = "aircraft" then expand_aicraft x else x)
      (Xml.children conf_xml) in
  let expanded_conf = make_element (Xml.tag conf_xml) (Xml.attribs conf_xml) conf_children in
  make_element
    "configuration"
    ["time_of_day", string_of_float timeofday; "data_file", data_file]
    [expanded_conf; Pprz.messages_xml ()]


let start_time = U.gettimeofday ()

(* Run a command and return its results as a string. *)
let read_process command =
  let buffer_size = 2048 in
  let buffer = Buffer.create buffer_size in
  let string = String.create buffer_size in
  let in_channel = Unix.open_process_in command in
  let chars_read = ref 1 in
  while !chars_read <> 0 do
    chars_read := input in_channel string 0 buffer_size;
    Buffer.add_substring buffer string 0 !chars_read
  done;
  ignore (Unix.close_process_in in_channel);
  Buffer.contents buffer

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
  (* version string with whitespace/newline at the end stripped *)
  let version_str =
    try
      Str.replace_first (Str.regexp "[ \n]+$") "" (read_process (Env.paparazzi_src ^ "/paparazzi_version"))
    with _ -> "UNKNOWN" in
  output_string f ("<!-- logged with runtime paparazzi_version " ^ version_str ^ " -->\n");
  let build_str =
    try
      let f = open_in (Env.paparazzi_home ^ "/var/build_version.txt") in
      let s = try input_line f with _ -> "UNKNOWN" in
      close_in f;
      s
    with _ -> "UNKNOWN" in
  output_string f ("<!-- logged with build paparazzi_version " ^ build_str ^ " -->\n");
  output_string f (Xml.to_string_fmt (log_xml start_time data_name));
  close_out f;
  open_out (logs_path // data_name)






let log = fun ?timestamp logging ac_name msg_name values ->
  match logging with
      Some log ->
        let s = string_of_values values in
        let t =
          match timestamp with
              Some x -> x
            | None   -> U.gettimeofday () -. start_time in
        fprintf log "%.3f %s %s %s\n" t ac_name msg_name s; flush log
    | None -> ()


(** Callback for a message from a registered A/C *)
let ac_msg = fun messages_xml logging ac_name ac ->
  let module Tele_Pprz = Pprz.MessagesOfXml(struct let xml = messages_xml let name="telemetry" end) in
  fun ts m ->
    try
      let timestamp = try Some (float_of_string ts) with _ -> None in
      let (msg_id, values) = Tele_Pprz.values_of_string m in
      let msg = Tele_Pprz.message_of_id msg_id in
      log ?timestamp logging ac_name msg.Pprz.name values;
      Fw_server.log_and_parse ac_name ac msg values;
      Rotorcraft_server.log_and_parse ac_name ac msg values
    with
        Telemetry_error (ac_name, msg) ->
          Ground_Pprz.message_send my_id "TELEMETRY_ERROR" ["ac_id", Pprz.String ac_name;"message", Pprz.String msg];
          prerr_endline msg
      | Pprz.Unknown_msg_name (x, c) ->
        fprintf stderr "Unknown message %s in class %s from %s: %s\n%!" x c ac_name m
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
          let wgs84 = Aircraft.add_pos_to_nav_ref (Geo a.pos) (east, north) in
          let twgs84 = Aircraft.add_pos_to_nav_ref nav_ref a.cam.target in
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
                 "rc_status", Pprz.String a.fbw.rc_status;
                 "rc_rate", Pprz.Int a.fbw.rc_rate ] in
  Ground_Pprz.message_send my_id "FLY_BY_WIRE"  values

let send_dl_values = fun a ->
  if a.nb_dl_setting_values > 0 then
    let csv = ref "" in
    for i = 0 to a.nb_dl_setting_values - 1 do
      match a.dl_setting_values.(i) with
      | None -> csv := sprintf "%s?," !csv
      | Some s -> csv := sprintf "%s%f," !csv s
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
      Circle (geo, r) ->
        let vs = [ "ac_id", Pprz.String a.id;
                   "circle_lat", Pprz.Float ((Rad>>Deg)geo.posn_lat);
                   "circle_long", Pprz.Float ((Rad>>Deg)geo.posn_long);
                   "radius", Pprz.Int r ] in
        Ground_Pprz.message_send my_id "CIRCLE_STATUS" vs
    | Segment (geo1, geo2) ->
      let vs = [ "ac_id", Pprz.String a.id;
                 "segment1_lat", Pprz.Float ((Rad>>Deg)geo1.posn_lat);
                 "segment1_long", Pprz.Float ((Rad>>Deg)geo1.posn_long);
                 "segment2_lat", Pprz.Float ((Rad>>Deg)geo2.posn_lat);
                 "segment2_long", Pprz.Float ((Rad>>Deg)geo2.posn_long) ] in
      Ground_Pprz.message_send my_id "SEGMENT_STATUS" vs
    | UnknownHorizMode -> ()

let send_survey_status = fun a ->
  match a.survey with
      None ->
        Ground_Pprz.message_send my_id "SURVEY_STATUS" ["ac_id", Pprz.String a.id]
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
  let tl_payload = fun link_id datalink_status link_status ->
    [ "ac_id", Pprz.String id;
      "link_id", Pprz.String link_id;
      "time_since_last_msg", Pprz.Float (U.gettimeofday () -. a.last_msg_date); (* don't use rx_lost_time from LINK_REPORT so it also works in simulation *)
      "rx_bytes", Pprz.Int64 (Int64.of_int link_status.rx_bytes);
      "rx_msgs", Pprz.Int64 (Int64.of_int link_status.rx_msgs);
      "rx_bytes_rate", Pprz.Float link_status.rx_bytes_rate;
      "tx_msgs", Pprz.Int64 (Int64.of_int link_status.tx_msgs);
      "uplink_lost_time", Pprz.Int64 (Int64.of_int datalink_status.uplink_lost_time);
      "uplink_msgs", Pprz.Int datalink_status.uplink_msgs;
      "downlink_msgs", Pprz.Int datalink_status.downlink_msgs;
      "downlink_rate", Pprz.Int datalink_status.downlink_rate;
      "ping_time", Pprz.Float link_status.ping_time]
  in
  (* if no link send anyway for rx_lost_time with special link id *)
  if Hashtbl.length a.link_status = 0 then
    begin
      let vs = tl_payload "no_id" a.datalink_status (Aircraft.link_status_init ()) in
      Ground_Pprz.message_send my_id "TELEMETRY_STATUS" vs
    end
  else
    (* send telemetry status for each link *)
    Hashtbl.iter (fun link_id link_status ->
      try
        let vs = tl_payload (string_of_int link_id) a.datalink_status link_status in
        Ground_Pprz.message_send my_id "TELEMETRY_STATUS" vs
      with
          _exc -> ()
    ) a.link_status

let send_moved_waypoints = fun a ->
  Hashtbl.iter
    (fun wp_id wp ->
      let geo = wp.wp_geo in
      let vs =
        ["ac_id", Pprz.String a.id;
         "wp_id", Pprz.Int wp_id;
         "long", Pprz.Float ((Rad>>Deg)geo.posn_long);
         "lat", Pprz.Float ((Rad>>Deg)geo.posn_lat);
         "alt", Pprz.Float wp.altitude;
         "ground_alt", Pprz.Float (try float (Srtm.of_wgs84 geo) with _ -> a.ground_alt)] in
      Ground_Pprz.message_send my_id "WAYPOINT_MOVED" vs)
    a.waypoints





let send_aircraft_msg = fun ac ->
  try
    let a = Hashtbl.find aircrafts ac in
    let f = fun x -> Pprz.Float x in
    let wgs84 = try a.pos with _ -> LL.make_geo 0. 0. in
    let values = ["ac_id", Pprz.String ac;
                  "roll", f (Geometry_2d.rad2deg a.roll);
                  "pitch", f (Geometry_2d.rad2deg a.pitch);
                  "heading", f (Geometry_2d.rad2deg a.heading);
                  "lat", f ((Rad>>Deg)wgs84.posn_lat);
                  "long", f ((Rad>>Deg) wgs84.posn_long);
                  "unix_time", f a.unix_time;
                  "itow", Pprz.Int64 a.itow;
                  "speed", f a.gspeed;
                  "airspeed", f a.airspeed; (* negative value is sent if no airspeed available *)
                  "course", f (Geometry_2d.rad2deg a.course);
                  "alt", f a.alt;
                  "agl", f a.agl;
                  "climb", f a.climb] in
    Ground_Pprz.message_send my_id "FLIGHT_PARAM" values;

    (** send ACINFO messages if more than one A/C registered *)
    if Hashtbl.length aircrafts > 1 then
      begin
        let cm_of_m_32 = fun f -> Pprz.Int32 (Int32.of_int (truncate (100. *. f))) in
        let cm_of_m = fun f -> Pprz.Int (truncate (100. *. f)) in
        let pos = LL.utm_of WGS84 a.pos in
        let ac_info = ["ac_id", Pprz.String ac;
                       "utm_east", cm_of_m_32 pos.utm_x;
                       "utm_north", cm_of_m_32 pos.utm_y;
                       "course", Pprz.Int (truncate (10. *. (Geometry_2d.rad2deg a.course)));
                       "alt", cm_of_m_32 a.alt;
                       "speed", cm_of_m a.gspeed;
                       "climb", cm_of_m a.climb;
                       "itow", Pprz.Int64 a.itow] in
        Dl_Pprz.message_send my_id "ACINFO" ac_info;
      end;

    if !Kml.enabled then
      Kml.update_ac a;

    begin
      match a.nav_ref with
          Some nav_ref ->
            let values = ["ac_id", Pprz.String ac;
                          "cur_block", Pprz.Int a.cur_block;
                          "cur_stage", Pprz.Int a.cur_stage;
                          "stage_time", Pprz.Int64 (Int64.of_int a.stage_time);
                          "block_time", Pprz.Int64 (Int64.of_int a.block_time);
                          "target_lat", f ((Rad>>Deg)a.desired_pos.posn_lat);
                          "target_long", f ((Rad>>Deg)a.desired_pos.posn_long);
                          "target_alt", Pprz.Float a.desired_altitude;
                          "target_climb", Pprz.Float a.desired_climb;
                          "target_course", Pprz.Float ((Rad>>Deg)a.desired_course);
                          "dist_to_wp", Pprz.Float a.dist_to_wp
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

    let ap_mode = get_indexed_value (modes_of_type a.vehicle_type) a.ap_mode in
    let gaz_mode = get_indexed_value gaz_modes a.gaz_mode in
    let lat_mode = get_indexed_value lat_modes a.lateral_mode in
    let horiz_mode = get_indexed_value horiz_modes a.horizontal_mode in
    let gps_mode = get_indexed_value gps_modes a.gps_mode in
    let state_filter_mode = get_indexed_value state_filter_modes a.state_filter_mode
    and kill_mode = if a.kill_mode then "ON" else "OFF" in
    let values = ["ac_id", Pprz.String ac;
                  "flight_time", Pprz.Int64 (Int64.of_int a.flight_time);
                  "ap_mode", Pprz.String ap_mode;
                  "gaz_mode", Pprz.String gaz_mode;
                  "lat_mode", Pprz.String lat_mode;
                  "horiz_mode", Pprz.String horiz_mode;
                  "gps_mode", Pprz.String gps_mode;
                  "state_filter_mode", Pprz.String state_filter_mode;
                  "kill_mode", Pprz.String kill_mode
                 ] in
    Ground_Pprz.message_send my_id "AP_STATUS" values;

    send_cam_status a;
    send_if_calib a;
    send_fbw a;
    send_svsinfo a;
    send_horiz_status a;

    a.time_since_last_survey_msg <- a.time_since_last_survey_msg +. float aircraft_msg_period /. 1000.;
    if a.time_since_last_survey_msg > 5. then (* FIXME Two missed messages *)
      a.survey <- None;

    send_survey_status a;
    send_dl_values a;
    send_moved_waypoints a;
    if !Kml.enabled then
      Kml.update_waypoints a;
    send_telemetry_status a
  with
      Not_found -> prerr_endline ac
    | x -> prerr_endline (Printexc.to_string x)

(** Check if it is a replayed A/C (c.f. sw/logalizer/play.ml) *)
let replayed = fun ac_id ->
  let n = String.length ac_id in
  if n > 6 && String.sub ac_id 0 6 = "replay" then
    (true, String.sub ac_id 6 (n - 6), "/var/replay/",  Xml.parse_file (Env.paparazzi_home // "var/replay/conf/conf.xml"))
  else
    (false, ac_id, "", conf_xml)

(* Store of unknown received A/C ids. To be able to report an error only once *)
let unknown_aircrafts = Hashtbl.create 5

let get_conf = fun real_id id conf_xml ->
  try
    ExtXml.child conf_xml "aircraft" ~select:(fun x -> ExtXml.attrib x "ac_id" = id)
  with
      Not_found ->
        Hashtbl.add unknown_aircrafts real_id ();
        failwith (sprintf "Error: A/C '%s' not found" id)

let check_md5sum = fun ac_name alive_md5sum aircraft_conf_dir ->
  (* Read the digest generated by sw/tools/gen_aircraft *)
  let f = open_in (aircraft_conf_dir // "aircraft.md5") in
  let md5sum = input_line f in
  close_in f;

  try
    match alive_md5sum with
        Pprz.Array array ->
          let n = Array.length array in
          assert(n = String.length md5sum / 2);
          for i = 0 to n - 1 do
            let x = int_of_string (sprintf "0x%c%c" md5sum.[2*i] md5sum.[2*i+1]) in
            assert (x = Pprz.int_of_value array.(i))
          done
      | _ -> failwith "Array expected here"
  with _ ->
    try
      match alive_md5sum with
          Pprz.Array array ->
            let n = Array.length array in
            assert(n = String.length md5sum / 2);
            for i = 0 to n - 1 do
              let x = 0 in
              assert (x = Pprz.int_of_value array.(i))
            done;
            fprintf stderr "MD5 is ZERO, be carefull with configurations\n%!"
        | _ -> failwith "Array expected here"
    with _ ->
      let error_message = sprintf "WARNING: live md5 signature for %s does not match current configuration, please reload your code (disable check with -no_md5_check option)" ac_name in
      if !no_md5_check then
        fprintf stderr "%s; continuing anyway as requested\n%!" error_message
      else
        failwith  error_message


let new_aircraft = fun get_alive_md5sum real_id ->
  let is_replayed, id, root_dir, conf_xml = replayed real_id in
  let conf = get_conf real_id id conf_xml in
  let ac_name = ExtXml.attrib conf "name" in
  let var_aircraft_dir = Env.paparazzi_home // root_dir // "var" // "aircrafts" // ac_name in

  if not (Sys.file_exists var_aircraft_dir) then begin
  (* Let's look for a backup configuration with the md5 signature *)
  end;

  let fp_file =  var_aircraft_dir // "flight_plan.xml" in
  let xml_fp = ExtXml.child (Xml.parse_file fp_file) "flight_plan" in

  let aircraft_conf_dir = var_aircraft_dir // "conf" in
  let airframe_file =  aircraft_conf_dir // ExtXml.attrib conf "airframe" in
  let airframe_xml = Xml.parse_file airframe_file in

  if not is_replayed then
    check_md5sum real_id (get_alive_md5sum ()) aircraft_conf_dir;

  let ac = Aircraft.new_aircraft real_id ac_name xml_fp airframe_xml in
  let update = fun () ->
    for i = 0 to Array.length ac.svinfo - 1 do
      ac.svinfo.(i).age <-  ac.svinfo.(i).age + 1;
    done in

  ignore (Glib.Timeout.add 1000 (fun _ -> update (); true));

  let messages_xml = Xml.parse_file (Env.paparazzi_home // root_dir // "conf" // "messages.xml") in
  ac, messages_xml

let check_alerts = fun a ->
  let bat_section = ExtXml.child a.airframe ~select:(fun x -> Xml.attrib x "name" = "BAT") "section" in
  let fvalue = fun name default ->
    try ExtXml.float_attrib (ExtXml.child bat_section ~select:(fun x -> ExtXml.attrib x "name" = name) "define") "value" with _ -> default in

  let catastrophic_level = fvalue "CATASTROPHIC_BAT_LEVEL" 9.
  and critic_level = fvalue "CRITIC_BAT_LEVEL" 10.
  and warning_level = fvalue "LOW_BAT_LEVEL" 10.5 in

  let send = fun level ->
    let vs = [ "ac_id", Pprz.String a.id;
               "level", Pprz.String level;
               "value", Pprz.Float a.bat] in
    Alerts_Pprz.message_send my_id "BAT_LOW" vs in
  if a.bat < 1. then send "INVALID"
  else if a.bat < catastrophic_level then send "CATASTROPHIC"
  else if a.bat < critic_level then send "CRITIC"
  else if a.bat < warning_level then send "WARNING"

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
  Wind.new_ac name 36;
  ignore(Ground_Pprz.message_bind "WIND_CLEAR" wind_clear);

  if !Kml.enabled then
    Kml.build_files a


(** Identifying message from an A/C *)
let ident_msg = fun log timestamp name vs ->
  try
    if not (Hashtbl.mem aircrafts name) &&
      not (Hashtbl.mem unknown_aircrafts name) then
      let get_md5sum = fun () -> Pprz.assoc "md5sum" vs in
      let ac, messages_xml = new_aircraft get_md5sum name in
      let ac_msg_closure = ac_msg messages_xml log name ac in
      let tsregexp = if timestamp then "(([0-9]+\\.[0-9]+) )?" else "" in
      let _b =
        Ivy.bind (fun _ args -> if timestamp then ac_msg_closure args.(1) args.(2) else ac_msg_closure "" args.(0))
        (sprintf "^%s%s +(.*)" tsregexp name) in
      register_aircraft name ac;
      Ground_Pprz.message_send my_id "NEW_AIRCRAFT" ["ac_id", Pprz.String name]
  with
      exc -> prerr_endline (Printexc.to_string exc)

let new_color = fun () ->
  sprintf "#%02x%02x%02x" (Random.int 256) (Random.int 256) (Random.int 256)

(* Waits for new aircrafts *)
let listen_acs = fun log timestamp ->
  (** Wait for any message (they all are identified with the A/C) *)
  ignore (Tm_Pprz.message_bind "ALIVE" (ident_msg log timestamp));
  if !replay_old_log then
    ignore (Tm_Pprz.message_bind "PPRZ_MODE" (ident_msg log timestamp))


let send_config = fun http _asker args ->
  let ac_id' = Pprz.string_assoc "ac_id" args in
  try
    let _is_replayed, ac_id, root_dir, conf_xml = replayed ac_id' in

    let conf = ExtXml.child conf_xml "aircraft" ~select:(fun x -> ExtXml.attrib x "ac_id" = ac_id) in
    let ac_name = ExtXml.attrib conf "name" in
    let protocol =
      if http then
        sprintf "http://%s:8889" (Unix.gethostname ())
      else
        sprintf "file://%s" Env.paparazzi_home in
    let prefix = fun s -> sprintf "%s/%s%s" protocol root_dir s in

    (** Expanded flight plan and settings have been compiled in var/ *)
    let fp = prefix ("var" // "aircrafts" // ac_name // "flight_plan.xml")
    and af = prefix ("conf" // ExtXml.attrib conf "airframe")
    and rc = prefix ("conf" // ExtXml.attrib conf "radio")
    and settings = if not _is_replayed then prefix ("var" // "aircrafts" // ac_name //
                                                       "settings.xml") else "file://replay" in
    let col = try Xml.attrib conf "gui_color" with _ -> new_color () in
    let ac_name = try Xml.attrib conf "name" with _ -> "" in
    [ "ac_id", Pprz.String ac_id;
      "flight_plan", Pprz.String fp;
      "airframe", Pprz.String af;
      "radio", Pprz.String rc;
      "settings", Pprz.String settings;
      "default_gui_color", Pprz.String col;
      "ac_name", Pprz.String ac_name ]
  with
      Not_found ->
        failwith (sprintf "ground UNKNOWN %s" ac_id')

let ivy_server = fun http ->
  ignore (Ground_Pprz.message_answerer my_id "AIRCRAFTS" send_aircrafts_msg);
  ignore (Ground_Pprz.message_answerer my_id "CONFIG" (send_config http))

(** Convert to cm, with rounding *)
let cm_of_m = fun f -> Pprz.Int (truncate ((100. *. f) +. 0.5))

(** Convert to mm, with rounding *)
let mm_of_m_32 = fun f -> Pprz.Int32 (Int32.of_int (truncate ((1000. *. f) +. 0.5)))

let dl_id = "ground_dl" (* Hack, should be [my_id] *)

(** Got a ground.MOVE_WAYPOINT and send a datalink.MOVE_WP *)
let move_wp = fun logging _sender vs ->
  let f = fun a -> List.assoc a vs
  and ac_id = Pprz.string_assoc "ac_id" vs
  and deg7 = fun f -> Pprz.Int32 (Int32.of_float (Pprz.float_assoc f vs *. 1e7)) in
  let vs = [ "wp_id", f "wp_id";
             "ac_id", Pprz.String ac_id;
             "lat", deg7 "lat";
             "lon", deg7 "long";
             "alt", mm_of_m_32 (Pprz.float_assoc "alt" vs) ] in
  Dl_Pprz.message_send dl_id "MOVE_WP" vs;
  log logging ac_id "MOVE_WP" vs

(** Got a DL_SETTING, and send an SETTING *)
let setting = fun logging _sender vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs in
  let vs = [ "index", List.assoc "index" vs;
             "ac_id", Pprz.String ac_id;
             "value", List.assoc "value" vs] in
  Dl_Pprz.message_send dl_id "SETTING" vs;
  log logging ac_id "SETTING" vs;
  (* mark the setting as not yet confirmed *)
  let ac = Hashtbl.find aircrafts ac_id in
  let idx = Pprz.int_of_value (List.assoc "index" vs) in
  ac.dl_setting_values.(idx) <- None


(** Got a GET_DL_SETTING, and send an GET_SETTING *)
let get_setting = fun logging _sender vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs in
  let vs = [ "index", List.assoc "index" vs;
             "ac_id", Pprz.String ac_id ] in
  Dl_Pprz.message_send dl_id "GET_SETTING" vs;
  log logging ac_id "GET_SETTING" vs;
  (* mark the setting as not yet confirmed *)
  let ac = Hashtbl.find aircrafts ac_id in
  let idx = Pprz.int_of_value (List.assoc "index" vs) in
  ac.dl_setting_values.(idx) <- None


(** Got a JUMP_TO_BLOCK, and send an BLOCK *)
let jump_block = fun logging _sender vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs in
  let vs = ["block_id", List.assoc "block_id" vs; "ac_id", Pprz.String ac_id] in
  Dl_Pprz.message_send dl_id "BLOCK" vs;
  log logging ac_id "BLOCK" vs

(** Got a RAW_DATALINK, send its contents *)
let raw_datalink = fun logging _sender vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs
  and m = Pprz.string_assoc "message" vs in
  for i = 0 to String.length m - 1 do
    if m.[i] = ';' then m.[i] <- ' '
  done;
  let msg_id, vs = Dl_Pprz.values_of_string m in
  let msg = Dl_Pprz.message_of_id msg_id in
  Dl_Pprz.message_send dl_id msg.Pprz.name vs;
  log logging ac_id msg.Pprz.name vs

(** Got a LINK_REPORT, update state but don't send (done asynchronously) *)
let link_report = fun logging _sender vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs
  and link_id = Pprz.int_assoc "link_id" vs in
  try
    let ac = Hashtbl.find aircrafts ac_id in
    let link_status = {
      Aircraft.rx_lost_time = Pprz.int_assoc "rx_lost_time" vs;
      rx_bytes = Pprz.int_assoc "rx_bytes" vs;
      rx_msgs = Pprz.int_assoc "rx_msgs" vs;
      rx_bytes_rate = Pprz.float_assoc "rx_bytes_rate" vs;
      tx_msgs = Pprz.int_assoc "tx_msgs" vs;
      ping_time = Pprz.float_assoc "ping_time" vs;
    } in
    Hashtbl.replace ac.link_status link_id link_status;
    log logging ac_id "LINK_REPORT" vs
  with _ -> ()


(** Get the 'ground' uplink messages, log them and send 'datalink' messages *)
let ground_to_uplink = fun logging ->
  let bind_log_and_send = fun name handler ->
    ignore (Ground_Pprz.message_bind name (handler logging)) in
  bind_log_and_send "MOVE_WAYPOINT" move_wp;
  bind_log_and_send "DL_SETTING" setting;
  bind_log_and_send "GET_DL_SETTING" get_setting;
  bind_log_and_send "JUMP_TO_BLOCK" jump_block;
  bind_log_and_send "RAW_DATALINK" raw_datalink;
  bind_log_and_send "LINK_REPORT" link_report


(* main loop *)
let () =
  let ivy_bus = ref Defivybus.default_ivy_bus
  and logging = ref true
  and http = ref false
  and timestamp = ref false in

  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), (sprintf "Bus\tDefault is %s" !ivy_bus);
      "-hostname", Arg.Set_string hostname, "<hostname> Set the address for the http server";
      "-http", Arg.Set http, "Send http: URLs (default is file:)";
      "-kml", Arg.Set Kml.enabled, "Enable KML file updating";
      "-kml_no_http", Arg.Set Kml.no_http, "KML without web server (local files only)";
      "-kml_port", Arg.Set_int Kml.port, (sprintf "Port for KML files (default is %d)" !Kml.port);
      "-n", Arg.Clear logging, "Disable log";
      "-timestamp", Arg.Set timestamp, "Bind on timestampped messages";
      "-no_md5_check", Arg.Set no_md5_check, "Disable safety matching of live and current configurations";
      "-replay_old_log", Arg.Set replay_old_log, "Enable aircraft registering on PPRZ_MODE messages"] in

  Arg.parse
    options
    (fun x -> Printf.fprintf stderr "%s: Warning: Don't do anything with '%s' argument\n" Sys.argv.(0) x)
    "Usage: ";

  Srtm.add_path srtm_path;

  Ivy.init "Paparazzi server" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  let logging =
    if !logging then
      (* Opens the log file *)
      Some (logger ())
    else
      None in

  (* Waits for new aircrafts *)
  listen_acs logging !timestamp;

  (* Forward messages from ground agents to vehicles *)
  ground_to_uplink logging;

  (* Waits for client configurations requests on the Ivy bus *)
  ivy_server !http;

  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
