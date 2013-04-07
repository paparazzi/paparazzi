(*
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
 * Copyright (C) 2012 Xavier Gibert
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

(*
 * Server part for all aircrafts (version 2.0 of messages.xml)
 *
 *)

open Printf
open Server_globals
open Aircraft
open Latlong
module LL = Latlong
module U = Unix
module Dl_Pprz = Pprz.Messages_of_type (struct let class_type = "uplink" end)


let heading_from_course = ref false

let set_pos_speed_from_ned = fun (a:Aircraft.aircraft) ref north east down v_north v_east v_down ->
  let (geo, h) = geo_hmsl_of_ltp (LL.make_ned [| north; east; down |]) ref a.d_hmsl in
  a.pos     <- geo;
  a.alt     <- h;
  a.gspeed  <- sqrt(v_north*.v_north +. v_east*.v_east);
  a.climb   <- -. v_down;
  a.agl     <- a.alt -. float (try Srtm.of_wgs84 a.pos with _ -> 0);
  a.course  <- norm_course (atan2 v_north v_east)

let set_desired_pos_speed_from_ned = fun (a:Aircraft.aircraft) ref north east down v_north v_east v_down ->
  a.desired_pos       <- Aircraft.add_pos_to_nav_ref ref ~z:(-.down) (east, north);
  a.desired_altitude  <- hmsl_of_ref ref a.d_hmsl -. down;
  a.desired_climb     <- (-. v_down);
  a.desired_course    <- norm_course (atan2 v_north v_east)

let log_and_parse = fun ac_name (a:Aircraft.aircraft) msg values ->
  let value = fun x -> try Pprz.assoc x values with Not_found -> failwith (sprintf "Error: field '%s' not found\n" x) in

  let fvalue = fun x ->
    let f = fvalue (value x) in
      match classify_float f with
  FP_infinite | FP_nan ->
    let msg = sprintf "Non normal number: %f in '%s %s %s'" f ac_name msg.Pprz.name (string_of_values values) in
    raise (Telemetry_error (ac_name, format_string_field msg))

      | _ -> f
  and ivalue = fun x -> ivalue (value x)
  and foi32value = fun x -> foi32value (value x) in
  if not (msg.Pprz.name = "DOWNLINK_STATUS") then
    a.last_msg_date <- U.gettimeofday ();
  match msg.Pprz.name with
  | "ALIVE" ->
      let vehicle_type = match ivalue "type" with
        | 1 -> GCS
        | 2 -> FixedWing
        | 3 -> Rotorcraft
        | _ -> UnknownVehicleType
      in
      a.vehicle_type <- vehicle_type;
  | "SYSTEM_STATUS" ->
      a.state_filter_mode <- check_index (ivalue "attitude_filter_mode") state_filter_modes "STATE_FILTER_MODES";
      a.gps_Pacc <- ivalue "Pacc";
      a.gps_mode <- check_index (ivalue "position_filter_status") position_filter_modes "POS_FILTER_MODES";
      a.unix_time <- LL.unix_time_of_tow (truncate (fvalue "itow" /. 1000.));
      a.itow <- Int32.of_float (fvalue "itow");
  | "FIXEDWING_STATUS" ->
      a.gaz_mode <- check_index (ivalue "ap_gaz") gaz_modes "AP_GAZ";
      a.lateral_mode <- check_index (ivalue "ap_lateral") lat_modes "AP_LAT";
      a.kill_mode <- ivalue "kill_auto_throttle" <> 0;
      a.throttle <- fvalue "throttle" /. 9600. *. 100.;
      a.rpm <- a.throttle *. 100.;
      if a.fbw.rc_mode = "FAILSAFE" then
        a.ap_mode <- 5 (* Override and set FAIL(Safe) Mode *)
      else
        a.ap_mode <- check_index (ivalue "ap_mode") fixedwing_ap_modes "AP_MODE"
  | "FBW_STATUS" ->
      a.fbw.pprz_mode_msgs_since_last_fbw_status_msg <- 0;
      a.fbw.rc_rate <- ivalue "frame_rate";
      a.fbw.rc_status <- rc_modes.(ivalue "rc_status");
      a.fbw.rc_mode <- fbw_modes.(ivalue "mode");
  | "ROTORCRAFT_STATUS" ->
      a.fbw.rc_status <- rc_modes.(ivalue "rc_status");
      a.fbw.rc_rate   <- ivalue "frame_rate";
      a.ap_mode       <- check_index (ivalue "ap_mode") rotorcraft_ap_modes "ROTORCRAFT_AP_MODE";
      a.kill_mode     <- ivalue "ap_motors_on" == 0;
      a.throttle <- fvalue "throttle" /. 9600. *. 100.;
      a.rpm <- a.throttle *. 100.;
  | "ELECTRICAL_STATUS" ->
      a.bat <- fvalue "voltage" /. 100.;
      a.energy <- ivalue "energy"
  | "CAM_STATUS" ->
      a.cam.phi <- (Deg>>Rad) (fvalue  "phi");
      a.cam.theta <- (Deg>>Rad) (fvalue  "theta");
      a.cam.target <- (fvalue  "target_x", fvalue  "target_y")
  | "GLOBAL_ORIGIN_ECEF" ->
      let x = foi32value "x" /. 100.
      and y = foi32value "y" /. 100.
      and z = foi32value "z" /. 100.
      and msl = foi32value "msl" /. 100. in
      let nav_ref_ecef = LL.make_ecef [| x; y; z |] in
      a.nav_ref <- Some (Ltp nav_ref_ecef);
      a.d_hmsl <- msl;
  | "GLOBAL_ORIGIN_LLA" ->
      let lat = ivalue "lat"
      and lon = ivalue "lon" in
      let geo = make_geo_deg (float lat /. 1e7) (float lon /. 1e7) in
      a.nav_ref <- Some (Geo geo)
  | "GLOBAL_ORIGIN_UTM" ->
      a.nav_ref <- Some (Utm { utm_x = fvalue "utm_east"; utm_y = fvalue "utm_north"; utm_zone = ivalue "utm_zone" })
  | "LOCAL_POSITION" ->
      if a.gps_mode = _pos_OK then begin
        let frame = ivalue "frame" in
        match (a.nav_ref, frame) with
        | (None, _) -> () (* No nav ref yet *)
        | (Some nav_ref, 0) -> (* NED frame *)
            let n = fvalue "pos_x"
            and e = fvalue "pos_y"
            and d = fvalue "pos_z"
            and vn = fvalue "speed_x"
            and ve = fvalue "speed_y"
            and vd = fvalue "speed_z" in
            set_pos_speed_from_ned a nav_ref n e d vn ve vd
        | (Some nav_ref, 1) -> (* ENU frame *)
            let e = fvalue "pos_x"
            and n = fvalue "pos_y"
            and u = fvalue "pos_z"
            and ve = fvalue "speed_x"
            and vn = fvalue "speed_y"
            and vu = fvalue "speed_z" in
            set_pos_speed_from_ned a nav_ref n e (-.u) vn ve (-.vu)
        | (Some nav_ref, _) -> () (* not a valid frame *)
      end
  | "LOCAL_POSITION_INT" ->
      if a.gps_mode = _pos_OK then begin
        let frame = ivalue "frame" in
        match (a.nav_ref, frame) with
        | (None, _) -> () (* No nav ref yet *)
        | (Some nav_ref, 0) -> (* NED frame *)
            let n = foi32value "pos_x" /. pos_frac
            and e = foi32value "pos_y" /. pos_frac
            and d = foi32value "pos_z" /. pos_frac
            and vn = foi32value "speed_x" /. speed_frac
            and ve = foi32value "speed_y" /. speed_frac
            and vd = foi32value "speed_z" /. speed_frac in
            set_pos_speed_from_ned a nav_ref n e d vn ve vd
        | (Some nav_ref, 1) -> (* ENU frame *)
            let e = foi32value "pos_x" /. pos_frac
            and n = foi32value "pos_y" /. pos_frac
            and u = foi32value "pos_z" /. pos_frac
            and ve = foi32value "speed_x" /. speed_frac
            and vn = foi32value "speed_y" /. speed_frac
            and vu = foi32value "speed_z" /. speed_frac in
            set_pos_speed_from_ned a nav_ref n e (-.u) vn ve (-.vu)
        | (Some nav_ref, _) -> () (* not a valid frame *)
      end
  | "GLOBAL_POSITION_UTM" ->
      if a.gps_mode = _pos_OK then begin
        let p = { LL.utm_x = fvalue "utm_east" /. 100.;
          utm_y = fvalue "utm_north" /. 100.;
          utm_zone = ivalue "utm_zone" } in
        a.pos <- LL.of_utm WGS84 p;
        a.gspeed <- fvalue "gspeed" /. 100.;
        a.course <- norm_course (fvalue "course" /. 1e7);
        if !heading_from_course then
          a.heading <- a.course;
        a.alt <- fvalue "alt" /. 1000.;
        a.climb <- fvalue "climb" /. 100.;
        a.agl <- a.alt -. float (try Srtm.of_wgs84 a.pos with _ -> 0);
        if a.gspeed > 3. && a.ap_mode = _AUTO2 then
          Wind.update ac_name a.gspeed a.course
      end
  | "GLOBAL_POSITION_LLA" ->
      if a.gps_mode = _pos_OK then begin
        let lat = ivalue "lat"
        and lon = ivalue "lon" in
        let geo = make_geo (float lat /. 1e7) (float lon /. 1e7) in
        a.pos <- geo;
        a.gspeed <- fvalue "speed" /. 100.;
        let vx = fvalue "velned_x"
        and vy = fvalue "velned_y" in
        a.course <- norm_course (atan2 vx vy);
        if !heading_from_course then
          a.heading <- a.course;
        a.alt <- fvalue "alt" /. 1000.;
        a.climb <- -. fvalue "velned_z" /. 100.;
        a.agl <- a.alt -. float (try Srtm.of_wgs84 a.pos with _ -> 0);
        if a.gspeed > 3. && a.ap_mode = _AUTO2 then
          Wind.update ac_name a.gspeed a.course
      end
  | "ATTITUDE_EULER" ->
      a.roll <- fvalue "roll";
      a.pitch <- fvalue "pitch";
      a.heading <- norm_course (fvalue "yaw")
  | "ATTITUDE_EULER_INT" ->
      a.roll <- foi32value "roll" /. angle_frac;
      a.pitch <- foi32value "pitch" /. angle_frac;
      a.heading <- norm_course (foi32value "yaw" /. angle_frac)
  | "LOCAL_POSITION_DESIRED" ->
      let frame = ivalue "frame" in
      begin match (a.nav_ref, frame) with
      | (None, _) -> () (* No nav ref yet *)
      | (Some nav_ref, 0) -> (* NED frame *)
          let n = fvalue "pos_x"
          and e = fvalue "pos_y"
          and d = fvalue "pos_z"
          and vn = fvalue "speed_x"
          and ve = fvalue "speed_y"
          and vd = fvalue "speed_z" in
          set_desired_pos_speed_from_ned a nav_ref n e d vn ve vd
      | (Some nav_ref, 1) -> (* ENU frame *)
          let e = fvalue "pos_x"
          and n = fvalue "pos_y"
          and u = fvalue "pos_z"
          and ve = fvalue "speed_x"
          and vn = fvalue "speed_y"
          and vu = fvalue "speed_z" in
          set_desired_pos_speed_from_ned a nav_ref n e (-.u) vn ve (-.vu)
      | (Some nav_ref, _) -> () (* not a valid frame *)
      end
  | "LOCAL_POSITION_INT_DESIRED" ->
      let frame = ivalue "frame" in
      begin match (a.nav_ref, frame) with
      | (None, _) -> () (* No nav ref yet *)
      | (Some nav_ref, 0) -> (* NED frame *)
          let n = foi32value "pos_x" /. pos_frac
          and e = foi32value "pos_y" /. pos_frac
          and d = foi32value "pos_z" /. pos_frac
          and vn = foi32value "speed_x" /. speed_frac
          and ve = foi32value "speed_y" /. speed_frac
          and vd = foi32value "speed_z" /. speed_frac in
          set_desired_pos_speed_from_ned a nav_ref n e d vn ve vd
      | (Some nav_ref, 1) -> (* ENU frame *)
          let e = foi32value "pos_x" /. pos_frac
          and n = foi32value "pos_y" /. pos_frac
          and u = foi32value "pos_z" /. pos_frac
          and ve = foi32value "speed_x" /. speed_frac
          and vn = foi32value "speed_y" /. speed_frac
          and vu = foi32value "speed_z" /. speed_frac in
          set_desired_pos_speed_from_ned a nav_ref n e (-.u) vn ve (-.vu)
      | (Some nav_ref, _) -> () (* not a valid frame *)
      end
  | "GLOBAL_POSITION_LLA_DESIRED" ->
      let lat = ivalue "lat"
      and lon = ivalue "lon" in
      let geo = make_geo (float lat /. 1e7) (float lon /. 1e7) in
      a.desired_pos <- geo;
      a.desired_altitude <- fvalue "alt" /. 1000.;
      a.desired_course <- norm_course (fvalue "course" /. 1e7);
      a.desired_climb <- fvalue "climb" /. 100.
  | "GLOBAL_POSITION_UTM_DESIRED" ->
      let p = { LL.utm_x = fvalue "utm_east" /. 100.;
        utm_y = fvalue "utm_north" /. 100.;
        utm_zone = ivalue "utm_zone" } in
      a.desired_pos <- LL.of_utm WGS84 p;
      a.desired_altitude <- fvalue "alt" /. 1000.;
      a.desired_course <- norm_course (fvalue "course" /. 1e7);
      a.desired_climb <- fvalue "climb" /. 100.
  | "MISSION_STATUS" ->
      a.flight_time <- ivalue "flight_time";
      a.cur_block <- ivalue "cur_block";
      a.cur_stage <- ivalue "cur_stage";
      a.block_time <- ivalue "block_time";
      a.stage_time <- ivalue "stage_time";
      a.dist_to_wp <- sqrt (fvalue "dist2_wp");
      a.horizontal_mode <- check_index (ivalue "horizontal_mode") horiz_modes "AP_HORIZ"
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
  | "WP_MOVED_LLA" ->
      let lat = ivalue "lat"
      and lon = ivalue "lon"
      and alt = ivalue "alt" in
      let geo = make_geo_deg (float lat /. 1e7) (float lon /. 1e7) in
      update_waypoint a (ivalue "wp_id") geo (float alt /. 100.)
  | "WP_MOVED_UTM" ->
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
  | "DL_VALUE" ->
      let i = ivalue "index" in
      if i < max_nb_dl_setting_values then begin
        a.dl_setting_values.(i) <- fvalue "value";
        a.nb_dl_setting_values <- max a.nb_dl_setting_values (i+1)
      end else
        failwith "Too much dl_setting values !!!"
  | "RC_SETTINGS" ->
      a.inflight_calib.if_val1 <- fvalue "slider_1_val";
      a.inflight_calib.if_val2 <- fvalue "slider_2_val";
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
        a.agl <- a.alt -. float (try Srtm.of_wgs84 a.pos with _ -> 0);
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
  | _ -> ()


