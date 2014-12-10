(*
 * Server part specific to fixed wing vehicles
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
open Server_globals
open Aircraft
open Latlong
module LL = Latlong
module U = Unix
module Dl_Pprz = Pprz.Messages (struct let name = "datalink" end)


(* FIXME: bound the loop *)
let rec norm_course =
  let _2pi = 2. *. LL.pi in
  fun c ->
    if c < 0. then norm_course (c +. _2pi)
    else if c >= _2pi then norm_course (c -. _2pi)
    else c


let fvalue = fun x ->
  match x with
      Pprz.Float x -> x
    | Pprz.Int32 x -> Int32.to_float x
    | Pprz.Int64 x -> Int64.to_float x
    | Pprz.Int x -> float_of_int x
    | _ -> failwith (sprintf "Receive.log_and_parse: float expected, got '%s'" (Pprz.string_of_value x))


let ivalue = fun x ->
  match x with
      Pprz.Int x -> x
    | Pprz.Int32 x -> Int32.to_int x
    | Pprz.Int64 x -> Int64.to_int x
    | _ -> failwith "Receive.log_and_parse: int expected"

let format_string_field = fun s ->
  let s = String.copy s in
  for i = 0 to String.length s - 1 do
    match s.[i] with
        ' ' -> s.[i] <- '_'
      | _ -> ()
  done;
  s

let check_index = fun i t where ->
  if i < 0 || i >= Array.length t then begin
    Debug.call 'E' (fun f -> fprintf f "Wrong index in %s: %d" where i);
    -1
  end else
    i


let update_waypoint = fun ac wp_id p alt ->
  let new_wp = { altitude = alt; wp_geo = p } in
  try
    let prev_wp = Hashtbl.find ac.waypoints wp_id in
    if new_wp <> prev_wp then
      Hashtbl.replace ac.waypoints wp_id new_wp
  with
      Not_found ->
        Hashtbl.add ac.waypoints wp_id new_wp



let heading_from_course = ref false

let log_and_parse = fun ac_name (a:Aircraft.aircraft) msg values ->
  let value = fun x -> try Pprz.assoc x values with Not_found -> failwith (sprintf "Error: field '%s' not found\n" x) in

  let fvalue = fun x ->
    let f = fvalue (value x) in
    match classify_float f with
        FP_infinite | FP_nan ->
          let msg = sprintf "Non normal number: %f in '%s %s %s'" f ac_name msg.Pprz.name (string_of_values values) in
          raise (Telemetry_error (ac_name, format_string_field msg))

      | _ -> f
  and ivalue = fun x -> ivalue (value x) in
  if not (msg.Pprz.name = "DOWNLINK_STATUS") then
    a.last_msg_date <- U.gettimeofday ();
  match msg.Pprz.name with
      "GPS" ->
        a.gps_mode <- check_index (ivalue "mode") gps_modes "GPS_MODE";
        if a.gps_mode = _3D then begin
          let p = { LL.utm_x = fvalue "utm_east" /. 100.;
                    utm_y = fvalue "utm_north" /. 100.;
                    utm_zone = ivalue "utm_zone" } in
          a.pos <- LL.of_utm WGS84 p;
          a.unix_time <- LL.unix_time_of_tow (truncate (fvalue "itow" /. 1000.));
          a.itow <- Int32.of_float (fvalue "itow");
          a.gspeed  <- fvalue "speed" /. 100.;
          a.course  <- norm_course ((Deg>>Rad)(fvalue "course" /. 10.));
          if !heading_from_course then
            a.heading <- a.course;
          a.agl     <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt);
          if a.gspeed > 3. && a.ap_mode = _AUTO2 then
            Wind.update ac_name a.gspeed a.course
        end
    | "GPS_LLA" ->
      let lat = ivalue "lat"
      and lon = ivalue "lon" in
      let geo = make_geo_deg (float lat /. 1e7) (float lon /. 1e7) in
      a.pos <- geo;
      a.unix_time <- LL.unix_time_of_tow (truncate (fvalue "itow" /. 1000.));
      a.itow <- Int32.of_float (fvalue "itow");
      a.gspeed  <- fvalue "speed" /. 100.;
      a.course  <- norm_course ((Deg>>Rad)(fvalue "course" /. 10.));
      if !heading_from_course then
        a.heading <- a.course;
      a.agl     <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt);
      a.gps_mode <- check_index (ivalue "mode") gps_modes "GPS_MODE";
      if a.gspeed > 3. && a.ap_mode = _AUTO2 then
        Wind.update ac_name a.gspeed a.course
    | "GPS_SOL" ->
      a.gps_Pacc <- ivalue "Pacc"
    | "ESTIMATOR" ->
      a.alt     <- fvalue "z";
      a.climb   <- fvalue "z_dot";
      if a.gps_mode = _3D then
        a.agl <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt)
      else
        a.agl <- a.alt -. a.ground_alt
    | "AIRSPEED" ->
      a.airspeed <- fvalue "airspeed"
    | "DESIRED" ->
      (* Trying to be compatible with old logs ... *)
      begin match a.nav_ref with
          Some nav_ref ->
            let x = (try fvalue "x" with _ -> fvalue "desired_x")
            and y = (try fvalue "y" with _ -> fvalue "desired_y") in
            a.desired_pos <- Aircraft.add_pos_to_nav_ref nav_ref (x, y);
        | None -> ()
      end;
      a.desired_altitude <- (try fvalue "altitude" with _ -> fvalue "desired_altitude");
      a.desired_climb <- (try fvalue "climb" with _ -> fvalue "desired_climb");
      begin try a.desired_course <- norm_course (fvalue "course") with _ -> () end
    | "NAVIGATION_REF" ->
      a.nav_ref <- Some (Utm { utm_x = fvalue "utm_east"; utm_y = fvalue "utm_north"; utm_zone = ivalue "utm_zone" });
      a.ground_alt <- fvalue "ground_alt";
      if a.gps_mode = _3D then
        a.agl <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt)
      else
        a.agl <- a.alt -. a.ground_alt
    | "NAVIGATION_REF_LLA" ->
      let lat = ivalue "lat"
      and lon = ivalue "lon" in
      let geo = make_geo_deg (float lat /. 1e7) (float lon /. 1e7) in
      a.nav_ref <- Some (Geo geo);
      a.ground_alt <- fvalue "ground_alt";
      if a.gps_mode = _3D then
        a.agl <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt)
      else
        a.agl <- a.alt -. a.ground_alt
    | "ATTITUDE" ->
      let roll = fvalue "phi"
      and pitch = fvalue "theta" in
      if (List.assoc "phi" msg.Pprz.fields).Pprz._type = Pprz.Scalar "int16" then begin (* Compatibility with old message in degrees *)
        a.roll <- roll /. 180. *. pi;
        a.pitch <- pitch /. 180. *. pi;
        heading_from_course := true; (* Awfull hack to get heading from GPS *)
      end else begin
        a.roll <- roll;
        a.pitch <- pitch;
        a.heading  <- norm_course (fvalue "psi")
      end
    | "NAVIGATION" ->
      a.cur_block <- ivalue "cur_block";
      a.cur_stage <- ivalue "cur_stage";
      a.dist_to_wp <- (try sqrt (fvalue "dist2_wp") with _ -> fvalue "dist_wp");
    | "BAT" ->
      a.throttle <- fvalue "throttle" /. 9600. *. 100.;
      a.kill_mode <- ivalue "kill_auto_throttle" <> 0;
      a.flight_time <- ivalue "flight_time";
      a.rpm <- a.throttle *. 100.;
      a.bat <- fvalue "voltage" /. 10.;
      a.stage_time <- ivalue "stage_time";
      a.block_time <- ivalue "block_time";
      a.energy <- ivalue "energy"
    | "FBW_STATUS" ->
      a.bat <- fvalue "vsupply" /. 10.;
      a.fbw.pprz_mode_msgs_since_last_fbw_status_msg <- 0;
      a.fbw.rc_rate <- ivalue "frame_rate";
      let fbw_rc_mode = ivalue "rc_status" in
      a.fbw.rc_status <- (
        match fbw_rc_mode with
            2 -> "NONE"
          | 1 -> "LOST"
          | _ -> "OK" );
      let fbw_mode = ivalue "mode" in
      a.fbw.rc_mode <- (
        match fbw_mode with
            2 -> "FAILSAFE"
          | 1 -> "AUTO"
          | _ -> "MANUAL" )
    | "STATE_FILTER_STATUS" ->
      a.state_filter_mode <- check_index (ivalue "state_filter_mode") state_filter_modes "STATE_FILTER_MODES"
    | "PPRZ_MODE" ->
      a.vehicle_type <- FixedWing;
      a.gaz_mode <- check_index (ivalue "ap_gaz") gaz_modes "AP_GAZ";
      a.lateral_mode <- check_index (ivalue "ap_lateral") lat_modes "AP_LAT";
      a.horizontal_mode <- check_index (ivalue "ap_horizontal") horiz_modes "AP_HORIZ";
      a.inflight_calib.if_mode <- check_index (ivalue "if_calib_mode") if_modes "IF_MODE";
      let mcu1_status = ivalue "mcu1_status" in
      (** c.f. link_autopilot.h *)
      if a.fbw.pprz_mode_msgs_since_last_fbw_status_msg < 10 then
        a.fbw.pprz_mode_msgs_since_last_fbw_status_msg <- a.fbw.pprz_mode_msgs_since_last_fbw_status_msg + 1;
      (* If we have recent direct information from the FBW, and it says FAILSAFE: then do not thrust the AP message PPRZ_MODE *)
      if ((a.fbw.pprz_mode_msgs_since_last_fbw_status_msg >= 10) && (a.fbw.rc_status <> "FAILSAFE")) then begin
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
      end ;
      if a.fbw.rc_mode = "FAILSAFE" then
        a.ap_mode <- 5 (* Override and set FAIL(Safe) Mode *)
      else
        a.ap_mode <- check_index (ivalue "ap_mode") fixedwing_ap_modes "AP_MODE"
    | "CAM" ->
      a.cam.phi <- (Deg>>Rad) (fvalue  "phi");
      a.cam.theta <- (Deg>>Rad) (fvalue  "theta");
      a.cam.target <- (fvalue  "target_x", fvalue  "target_y")
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
              a.horiz_mode <- Circle (Aircraft.add_pos_to_nav_ref nav_ref (fvalue "center_east", fvalue "center_north"), truncate (fvalue "radius"));
              if !Kml.enabled then Kml.update_horiz_mode a
          | _ -> ()
      end
    | "SEGMENT" ->
      begin
        match a.nav_ref, a.horizontal_mode with
            Some nav_ref, 1 -> (** FIXME *)
              let p1 = Aircraft.add_pos_to_nav_ref nav_ref (fvalue "segment_east_1", fvalue "segment_north_1")
              and p2 = Aircraft.add_pos_to_nav_ref nav_ref (fvalue "segment_east_2", fvalue "segment_north_2") in
              a.horiz_mode <- Segment (p1, p2);
              if !Kml.enabled then Kml.update_horiz_mode a
          | _ -> ()
      end
    | "SETTINGS" ->
      a.inflight_calib.if_val1 <- fvalue "slider_1_val";
      a.inflight_calib.if_val2 <- fvalue "slider_2_val";
    | "SURVEY" ->
      begin
        a.time_since_last_survey_msg <- 0.;
        match a.nav_ref with
            Some nav_ref ->
              let p1 = Aircraft.add_pos_to_nav_ref nav_ref (fvalue "west", fvalue "south")
              and p2 = Aircraft.add_pos_to_nav_ref nav_ref (fvalue "east", fvalue "north") in
              a.survey <- Some (p1, p2)
          | None -> ()
      end
    | "CALIBRATION" ->
      a.throttle_accu <- fvalue "climb_sum_err"
    | "DL_VALUE" ->
      let i = ivalue "index" in
      if i < max_nb_dl_setting_values then begin
        a.dl_setting_values.(i) <- Some (fvalue "value");
        a.nb_dl_setting_values <- max a.nb_dl_setting_values (i+1)
      end else
        failwith "Too much dl_setting values !!!"
    | "WP_MOVED" ->
      begin
        match a.nav_ref with
            Some Utm nav_ref ->
              let utm_zone = try ivalue "utm_zone" with _ -> nav_ref.utm_zone in
              let p = { LL.utm_x = fvalue "utm_east";
                        utm_y = fvalue "utm_north";
                        utm_zone = utm_zone } in
              update_waypoint a (ivalue "wp_id") (LL.of_utm WGS84 p) (fvalue "alt")
          | _ -> () (** Can't use this message  *)
      end
    | "WP_MOVED_LLA" ->
      let lat = ivalue "lat"
      and lon = ivalue "lon"
      and alt = ivalue "alt" in
      let geo = make_geo_deg (float lat /. 1e7) (float lon /. 1e7) in
      update_waypoint a (ivalue "wp_id") geo (float alt /. 100.)
    | "GENERIC_COM" ->
      let flight_time = ivalue "flight_time" in
      if flight_time >= a.flight_time then begin
        a.flight_time <- flight_time;
        let lat = fvalue "lat"
        and lon = fvalue "lon" in
        let geo = make_geo_deg (lat /. 1e7) (lon /. 1e7) in
        a.pos <- geo;
        a.alt <- fvalue "alt";
        a.gspeed  <- fvalue "gspeed" /. 100.;
        a.course  <- norm_course (fvalue "course" /. 1e3);
        if !heading_from_course then
          a.heading <- a.course;
        a.agl <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt);
        a.bat <- fvalue "vsupply" /. 10.;
        a.energy <- ivalue "energy" * 100;
        a.throttle <- fvalue "throttle";
        a.ap_mode <- check_index (ivalue "ap_mode") fixedwing_ap_modes "AP_MODE";
        a.cur_block <- ivalue "nav_block";
      end
    | "FORMATION_SLOT_TM" ->
      Dl_Pprz.message_send "ground_dl" "FORMATION_SLOT" values
    | "FORMATION_STATUS_TM" ->
      Dl_Pprz.message_send "ground_dl" "FORMATION_STATUS" values
    | "TCAS_RA" ->
      let vs = [
        "ac_id", Pprz.Int (ivalue "ac_id");
        "ac_id_conflict", Pprz.Int (int_of_string a.id);
        "resolve", Pprz.Int (ivalue "resolve")
      ] in
      Dl_Pprz.message_send "ground_dl" "TCAS_RESOLVE" vs
    | "DATALINK_REPORT" ->
      a.datalink_status.uplink_lost_time <- ivalue "uplink_lost_time";
      a.datalink_status.uplink_msgs <- ivalue "uplink_nb_msgs";
      a.datalink_status.downlink_rate <- ivalue "downlink_rate";
      a.datalink_status.downlink_msgs <- ivalue "downlink_nb_msgs"
    | _ -> ()

