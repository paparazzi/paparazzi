(*
 * Server part for v1 message set
 *
 * Copyright (C) ENAC
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
module Dl_Pprz = PprzLink.Messages (struct let name = "datalink" end)


(* FIXME: bound the loop *)
let rec norm_course =
  let _2pi = 2. *. LL.pi in
  fun c ->
    if c < 0. then norm_course (c +. _2pi)
    else if c >= _2pi then norm_course (c -. _2pi)
    else c


let fvalue = fun x ->
  match x with
      PprzLink.Float x -> x
    | PprzLink.Int32 x -> Int32.to_float x
    | PprzLink.Int64 x -> Int64.to_float x
    | PprzLink.Int x -> float_of_int x
    | _ -> failwith (sprintf "Receive.log_and_parse: float expected, got '%s'" (PprzLink.string_of_value x))


let ivalue = fun x ->
  match x with
      PprzLink.Int x -> x
    | PprzLink.Int32 x -> Int32.to_int x
    | PprzLink.Int64 x -> Int64.to_int x
    | _ -> failwith "Receive.log_and_parse: int expected"

(*
  let i32value = fun x ->
  match x with
  PprzLink.Int32 x -> x
  | _ -> failwith "Receive.log_and_parse: int32 expected"
*)

let foi32value = fun x ->
  match x with
      PprzLink.Int32 x -> Int32.to_float x
    | _ -> failwith "Receive.log_and_parse: int32 expected"

let format_string_field = fun s ->
  let s = Compat.bytes_copy s in
  for i = 0 to Compat.bytes_length s - 1 do
    match s.[i] with
        ' ' -> Compat.bytes_set s i '_'
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

(*let get_pprz_mode = fun ap_mode ->
  let mode = ref 0 in
  if ap_mode = 0 || ap_mode = 1 || ap_mode = 2 || ap_mode = 4 || ap_mode = 7 then mode := 0 (* MANUAL *)
  else if ap_mode = 3 || ap_mode = 5 || ap_mode = 6 || ap_mode = 8 then mode := 1 (* AUTO1 *)
  else if ap_mode = 9 || ap_mode = 10 || ap_mode = 11 || ap_mode = 12 then mode := 2; (* AUTO2 *)
  !mode
*)

let get_rc_status = fun rc_status ->
  let status = ref "" in
  if rc_status = 0 then status := "OK"
  else if rc_status = 1 then status := "LOST"
  else if rc_status = 2 then status := "NONE"; (* REALLY_LOST *)
  !status

let pos_frac = 2. ** 8.
let speed_frac = 2. ** 19.
let angle_frac = 2. ** 12.
let gps_frac = 1e7

let geo_hmsl_of_ltp = fun ned nav_ref d_hmsl ->
  match nav_ref with
    | Ltp nav_ref_ecef ->
      let (geo, alt) = LL.geo_of_ecef LL.WGS84 (LL.ecef_of_ned nav_ref_ecef ned) in
      (geo, alt +. d_hmsl)
    | _ -> (LL.make_geo 0. 0., 0.)

let hmsl_of_ref = fun nav_ref d_hmsl ->
  match nav_ref with
    | Ltp nav_ref_ecef ->
      let (_, alt) = LL.geo_of_ecef LL.WGS84 nav_ref_ecef in
      alt +. d_hmsl
    | _ -> 0.


let heading_from_course = ref false

let log_and_parse = fun ac_name (a:Aircraft.aircraft) msg values ->
  let value = fun x -> try PprzLink.assoc x values with Not_found -> failwith (sprintf "Error: field '%s' not found\n" x) in

  let fvalue = fun x ->
    let f = fvalue (value x) in
    match classify_float f with
        FP_infinite | FP_nan ->
          let msg = sprintf "Non normal number: %f in '%s %s %s'" f ac_name msg.PprzLink.name (string_of_values values) in
          raise (Telemetry_error (ac_name, format_string_field msg))

      | _ -> f
  and ivalue = fun x -> ivalue (value x)
  (*and i32value = fun x -> i32value (value x)*)
  and foi32value = fun x -> foi32value (value x) in
  if not (msg.PprzLink.name = "DOWNLINK_STATUS") then
    a.last_msg_date <- U.gettimeofday ();
  match msg.PprzLink.name with
      "GPS" ->
        a.gps_mode <- check_index (ivalue "mode") gps_modes "GPS_MODE";
        if a.gps_mode >= _3D then begin
          let p = { LL.utm_x = fvalue "utm_east" /. 100.;
                    utm_y = fvalue "utm_north" /. 100.;
                    utm_zone = ivalue "utm_zone" } in
          a.pos <- LL.of_utm WGS84 p;
          a.unix_time <- LL.unix_time_of_tow (truncate (fvalue "itow" /. 1000.));
          a.itow <- Int64.of_float (fvalue "itow");
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
      a.itow <- Int64.of_float (fvalue "itow");
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
    | "GPS_INT" ->
      a.unix_time <- LL.unix_time_of_tow (truncate (fvalue "tow" /. 1000.));
      a.itow <- Int64.of_float (fvalue "tow");
      a.gps_Pacc <- ivalue "pacc"
    | "ESTIMATOR" ->
      a.alt     <- fvalue "z";
      a.climb   <- fvalue "z_dot";
      if a.gps_mode = _3D then
        a.agl <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt)
      else
        a.agl <- a.alt -. a.ground_alt
    | "ROTORCRAFT_FP" ->
        begin match a.nav_ref with
            None -> (); (* No nav_ref yet *)
          | Some nav_ref ->
            let north = foi32value "north" /. pos_frac
            and east  = foi32value "east" /. pos_frac
            and up    = foi32value "up" /. pos_frac in
            let (geo, h) = geo_hmsl_of_ltp (LL.make_ned [| north; east; -. up |]) nav_ref a.d_hmsl in
            a.pos <- geo;
            a.alt <- h;
            let desired_east  = foi32value "carrot_east" /. pos_frac
            and desired_north = foi32value "carrot_north" /. pos_frac
            and desired_alt = foi32value "carrot_up" /. pos_frac in
            a.desired_pos <- Aircraft.add_pos_to_nav_ref nav_ref ~z:desired_alt (desired_east, desired_north);
            a.desired_altitude <- desired_alt +. (hmsl_of_ref nav_ref a.d_hmsl);
            a.desired_course   <- foi32value "carrot_psi" /. angle_frac
            (* a.desired_climb <-  ?? *)
        end;
        let veast  = foi32value "veast" /. speed_frac
        and vnorth = foi32value "vnorth" /. speed_frac in
        a.gspeed  <- sqrt(vnorth*.vnorth +. veast*.veast);
        a.climb   <- foi32value "vup" /. speed_frac;
        a.agl     <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt);
        a.course  <- norm_course (atan2 veast vnorth);
        a.heading <- norm_course (foi32value "psi" /. angle_frac);
        a.roll    <- foi32value "phi" /. angle_frac;
        a.pitch   <- foi32value "theta" /. angle_frac;
        a.throttle <- foi32value "thrust" /. 9600. *. 100.;
        a.flight_time   <- ivalue "flight_time";
        (*if a.gspeed > 3. && a.ap_mode = _AUTO2 then
          Wind.update ac_name a.gspeed a.course*)
    | "ROTORCRAFT_FP_MIN" ->
        begin match a.nav_ref with
            None -> (); (* No nav_ref yet *)
          | Some nav_ref ->
            let north = foi32value "north" /. pos_frac
            and east  = foi32value "east" /. pos_frac
            and up    = foi32value "up" /. pos_frac in
            let (geo, h) = geo_hmsl_of_ltp (LL.make_ned [| north; east; -. up |]) nav_ref a.d_hmsl in
            a.pos <- geo;
            a.alt <- h
        end;
        a.gspeed  <- fvalue "gspeed" /. 100.;
        a.agl     <- a.alt -. (try float (Srtm.of_wgs84 a.pos) with _ -> a.ground_alt)
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
    | "INS_REF" ->
      let x = foi32value "ecef_x0" /. 100.
      and y = foi32value "ecef_y0" /. 100.
      and z = foi32value "ecef_z0" /. 100.
      and alt = foi32value "alt0" /. 1000.
      and hmsl = foi32value "hmsl0" /. 1000. in
      let nav_ref_ecef = LL.make_ecef [| x; y; z |] in
      a.nav_ref <- Some (Ltp nav_ref_ecef);
      a.d_hmsl <- hmsl -. alt;
      a.ground_alt <- hmsl;
    | "ATTITUDE" ->
      let roll = fvalue "phi"
      and pitch = fvalue "theta" in
      if (List.assoc "phi" msg.PprzLink.fields).PprzLink._type = PprzLink.Scalar "int16" then begin (* Compatibility with old message in degrees *)
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
    | "ROTORCRAFT_NAV_STATUS" ->
      a.block_time <- ivalue "block_time";
      a.stage_time <- ivalue "stage_time";
      a.cur_block <- ivalue "cur_block";
      a.cur_stage <- ivalue "cur_stage";
      a.horizontal_mode <- check_index (ivalue "horizontal_mode") horiz_modes "AP_HORIZ";
      a.dist_to_wp <- (try fvalue "dist_wp" with _ -> 0.);
    | "ENERGY" ->
      a.throttle <- fvalue "throttle";
      a.bat <- fvalue "voltage";
      a.amp <- fvalue "current";
      a.power <- fvalue "power";
      a.charge <- fvalue "charge";
      a.energy <- fvalue "energy";
    | "FBW_STATUS" ->
      a.fbw.fbw_bat <- fvalue "vsupply";
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
        a.ap_mode <- -2 (* Override and set to FAIL (see server.ml) *)
      else
        a.ap_mode <- check_index (ivalue "ap_mode") (modes_of_aircraft a) "AP_MODE"
    | "ROTORCRAFT_STATUS" ->
      a.vehicle_type  <- Rotorcraft;
      a.fbw.rc_status <- get_rc_status (ivalue "rc_status");
      a.fbw.rc_rate   <- ivalue "frame_rate";
      a.gps_mode      <- check_index (ivalue "gps_status") gps_modes "GPS_MODE";
      a.ap_mode       <- check_index (ivalue "ap_mode") (modes_of_aircraft a) "ROTORCRAFT_AP_MODE";
      a.kill_mode     <- ivalue "ap_motors_on" == 0;
      a.bat           <- fvalue "vsupply"
    | "ROVER_STATUS" ->
      a.vehicle_type  <- Rover;
      a.fbw.rc_status <- get_rc_status (ivalue "rc_status");
      a.fbw.rc_rate   <- ivalue "frame_rate";
      a.gps_mode      <- check_index (ivalue "gps_status") gps_modes "GPS_MODE";
      a.ap_mode       <- check_index (ivalue "ap_mode") (modes_of_aircraft a) "ROVER_AP_MODE";
      a.kill_mode     <- ivalue "ap_motors_on" == 0;
      a.bat           <- fvalue "vsupply"
    | "STATE_FILTER_STATUS" ->
      a.state_filter_mode <- check_index (ivalue "state_filter_mode") state_filter_modes "STATE_FILTER_MODES"
    | "DATALINK_REPORT" ->
      a.datalink_status.uplink_lost_time <- ivalue "uplink_lost_time";
      a.datalink_status.uplink_msgs <- ivalue "uplink_nb_msgs";
      a.datalink_status.downlink_rate <- ivalue "downlink_rate";
      a.datalink_status.downlink_msgs <- ivalue "downlink_nb_msgs"
    | "CAM" ->
      a.cam.pan <- (Deg>>Rad) (fvalue  "pan");
      a.cam.tilt <- (Deg>>Rad) (fvalue  "tilt");
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
      update_waypoint a (ivalue "wp_id") geo (float alt /. 1000.)
    | "WP_MOVED_ENU" ->
      begin
        match a.nav_ref with
            Some nav_ref ->
              let east  = foi32value "east"   /. pos_frac
              and north = foi32value "north"  /. pos_frac
              and up    = foi32value "up"     /. pos_frac in
              let (geo, h) = geo_hmsl_of_ltp (LL.make_ned [| north; east; -. up |]) nav_ref a.d_hmsl in
              update_waypoint a (ivalue "wp_id") geo h;
          | None -> (); (** Can't use this message  *)
      end
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
        a.charge <- fvalue "charge" /. 10.;
        a.throttle <- fvalue "throttle";
        a.ap_mode <- check_index (ivalue "ap_mode") (modes_of_aircraft a) "AP_MODE";
        a.cur_block <- ivalue "nav_block";
      end
    | "FORMATION_SLOT_TM" ->
      Dl_Pprz.message_send "ground_dl" "FORMATION_SLOT" values
    | "FORMATION_STATUS_TM" ->
      Dl_Pprz.message_send "ground_dl" "FORMATION_STATUS" values
    | "TCAS_RA" ->
      let vs = [
        "ac_id", PprzLink.Int (ivalue "ac_id");
        "ac_id_conflict", PprzLink.Int (int_of_string a.id);
        "resolve", PprzLink.Int (ivalue "resolve")
      ] in
      Dl_Pprz.message_send "ground_dl" "TCAS_RESOLVE" vs
    | _ -> ()
