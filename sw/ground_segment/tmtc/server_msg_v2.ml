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
      a.agl     <- a.alt -. float (try Srtm.of_wgs84 a.pos with _ -> 0);
      a.course  <- norm_course ((Rad>>Deg) (foi32value "psi" /. angle_frac));
      a.heading <- norm_course (foi32value "psi" /. angle_frac);
      a.roll    <- foi32value "phi" /. angle_frac;
      a.pitch   <- foi32value "theta" /. angle_frac;
      a.throttle <- foi32value "thrust" /. 2.; (* thrust / 200 * 100 *)
      a.flight_time   <- ivalue "flight_time";
      (*if a.gspeed > 3. && a.ap_mode = _AUTO2 then
          Wind.update ac_name a.gspeed a.course*)
  | "GPS_UTM" ->
      a.gps_mode <- check_index (ivalue "fix") gps_modes "GPS_MODE";
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
      a.agl     <- a.alt -. float (try Srtm.of_wgs84 a.pos with _ -> 0);
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
      a.agl     <- a.alt -. float (try Srtm.of_wgs84 a.pos with _ -> 0);
      a.gps_mode <- check_index (ivalue "fix") gps_modes "GPS_MODE";
      if a.gspeed > 3. && a.ap_mode = _AUTO2 then
        Wind.update ac_name a.gspeed a.course
  | "GPS_INT" ->
      a.unix_time <- LL.unix_time_of_tow (truncate (fvalue "tow" /. 1000.));
      a.itow <- Int32.of_float (fvalue "tow");
      a.gps_Pacc <- ivalue "pacc"
  | "ESTIMATOR" ->
      a.alt     <- fvalue "z";
      a.climb   <- fvalue "z_dot"
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
      begin try  a.desired_course <- norm_course (fvalue "course") with _ -> () end
  | "NAVIGATION_REF" ->
      a.nav_ref <- Some (Utm { utm_x = fvalue "utm_east"; utm_y = fvalue "utm_north"; utm_zone = ivalue "utm_zone" })
  | "NAVIGATION_REF_LLA" ->
      let lat = ivalue "lat"
      and lon = ivalue "lon" in
      let geo = make_geo_deg (float lat /. 1e7) (float lon /. 1e7) in
      a.nav_ref <- Some (Geo geo)
  | "ATTITUDE_EULER" ->
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
  | "MISSION_STATUS" ->
      a.gps_mode      <- check_index (ivalue "gps_status") gps_modes "GPS_MODE";
      a.flight_time <- ivalue "flight_time";
      a.stage_time <- ivalue "stage_time";
      a.block_time <- ivalue "block_time";
      a.cur_block <- ivalue "cur_block";
      a.cur_stage <- ivalue "cur_stage";
      a.horizontal_mode <- check_index (ivalue "horizontal_mode") horiz_modes "AP_HORIZ";
      a.dist_to_wp <- sqrt (fvalue "dist2_wp")
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
  | "V_CTL_CALIBRATION" ->
      a.throttle_accu <- fvalue "climb_sum_err"
  | "DL_VALUE" ->
      let i = ivalue "index" in
      if i < max_nb_dl_setting_values then begin
  a.dl_setting_values.(i) <- fvalue "value";
  a.nb_dl_setting_values <- max a.nb_dl_setting_values (i+1)
      end else
  failwith "Too much dl_setting values !!!"
  | "INS_REF" ->
      let x = foi32value "ecef_x0" /. 100.
      and y = foi32value "ecef_y0" /. 100.
      and z = foi32value "ecef_z0" /. 100.
      and alt = foi32value "alt0" /. 1000.
      and hmsl = foi32value "hmsl0" /. 1000. in
      let nav_ref_ecef = LL.make_ecef [| x; y; z |] in
      a.nav_ref <- Some (Ltp nav_ref_ecef);
      a.d_hmsl <- hmsl -. alt;
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
        let geo = make_geo (lat /. 1e7) (lon /. 1e7) in
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


