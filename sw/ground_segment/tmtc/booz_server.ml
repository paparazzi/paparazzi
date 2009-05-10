(*
 * $Id: fw_server.ml,v 1.1 2009/03/22 17:53:48 hecto Exp $
 *
 * Server part specific to booz vehicles
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
module Dl_Pprz = Pprz.Messages (struct let name = "datalink" end)

let nav_ref_ecef = ref (LL.make_ecef [|0.;0.;0.|])

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
    | Pprz.Int x -> float_of_int x 
    | _ -> failwith (sprintf "Receive.log_and_parse: float expected, got '%s'" (Pprz.string_of_value x))

	  
let ivalue = fun x ->
  match x with
    Pprz.Int x -> x
  | Pprz.Int32 x -> Int32.to_int x 
  | _ -> failwith "Receive.log_and_parse: int expected"

(*
let i32value = fun x ->
  match x with
    Pprz.Int32 x -> x
  | _ -> failwith "Receive.log_and_parse: int32 expected"
*)

let foi32value = fun x ->
  match x with
    Pprz.Int32 x -> Int32.to_float x
  | _ -> failwith "Receive.log_and_parse: int32 expected"

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
  let new_wp = { altitude = alt; wp_utm = p } in
  try
    let prev_wp = Hashtbl.find ac.waypoints wp_id in
    if new_wp <> prev_wp then
      Hashtbl.replace ac.waypoints wp_id new_wp
  with
    Not_found ->
      Hashtbl.add ac.waypoints wp_id new_wp

let get_pprz_mode = fun ap_mode ->
  let mode = ref 0 in
  if ap_mode = 0 || ap_mode = 1 || ap_mode = 2 || ap_mode = 4 || ap_mode = 7 then mode := 0 (* MANUAL *)
  else if ap_mode = 3 || ap_mode = 5 || ap_mode = 6 || ap_mode = 8 then mode := 1 (* AUTO1 *)
  else if ap_mode = 9 || ap_mode = 10 || ap_mode = 11 || ap_mode = 12 then mode := 2; (* AUTO2 *)
  !mode

let get_rc_status = fun rc_status ->
  let status = ref "" in
  if rc_status = 0 then status := "OK"
  else if rc_status = 1 then status := "LOST"
  else if rc_status = 2 then status := "NONE"; (* REALLY_LOST *)
  !status

let pos_frac = 2. ** 8.
let speed_frac = 2. ** 8.
let angle_frac = 2. ** 12.
let gps_frac = 1e7

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
  (*and i32value = fun x -> i32value (value x)*)
  and foi32value = fun x -> foi32value (value x) in
  if not (msg.Pprz.name = "DOWNLINK_STATUS") then
    a.last_bat_msg_date <- U.gettimeofday ();
  match msg.Pprz.name with
    "BOOZ2_FP" ->
      begin match a.nav_ref with
        None -> (); (* No nav_ref yet *)
        | Some nav_ref ->
          let x = foi32value "north" /. pos_frac
          and y = foi32value "east" /. pos_frac
          and z = foi32value "up" /. pos_frac in
          let (geo, _) = LL.geo_of_ecef LL.WGS84 (LL.ecef_of_ned !nav_ref_ecef (LL.make_ned [| x; y; -. z |])) in
          let (_, ref_alt) = LL.geo_of_ecef LL.WGS84 !nav_ref_ecef in
          a.pos <- LL.utm_of LL.WGS84 geo;
          a.alt <- z +. ref_alt;
          a.desired_east     <- foi32value "carrot_east" /. pos_frac;
          a.desired_north    <- foi32value "carrot_north" /. pos_frac;
          a.desired_altitude <- (foi32value "carrot_up" /. pos_frac) +. ref_alt;
          a.desired_course   <- foi32value "carrot_psi" /. angle_frac
          (* a.desired_climb <-  ?? *)
      end;
      let veast  = foi32value "veast" /. speed_frac /. 10000.
      and vnorth = foi32value "vnorth" /. speed_frac /. 10000. in
      a.gspeed  <- sqrt(vnorth*.vnorth +. veast*.veast);
      a.climb   <- foi32value "vup" /. speed_frac /. 10000.;
      a.agl     <- a.alt -. float (try Srtm.of_utm a.pos with _ -> 0);
      a.course  <- norm_course ((Rad>>Deg) (foi32value "psi" /. angle_frac));
      a.heading <- norm_course (foi32value "psi" /. angle_frac);
      a.roll    <- foi32value "phi" /. angle_frac;
      a.pitch   <- foi32value "theta" /. angle_frac;
      a.throttle <- foi32value "thrust" /. 2.; (* thrust / 200 * 100 *)
      (*a.unix_time <- LL.unix_time_of_tow (truncate (fvalue "itow" /. 1000.));
      a.itow <- Int32.of_float (fvalue "itow");
      a.flight_time <- ivalue "flight_time";*)
      if a.gspeed > 3. && a.ap_mode = _AUTO2 then
	      Wind.update ac_name a.gspeed a.course
  | "BOOZ_STATUS" ->
      a.fbw.rc_status <- get_rc_status (ivalue "rc_status");
      a.gps_mode      <- check_index (ivalue "gps_status") gps_modes "GPS_MODE";
      a.ap_mode       <- check_index (get_pprz_mode (ivalue "ap_mode")) ap_modes "BOOZ_AP_MODE";
      a.kill_mode     <- ivalue "ap_motors_on" == 0;
      a.bat           <- fvalue "vsupply" /. 10.;
  | "BOOZ2_NAV_REF" ->
      let x = (foi32value "x" /. 100.)
      and y = (foi32value "y" /. 100.)
      and z = (foi32value "z" /. 100.) in
      nav_ref_ecef := LL.make_ecef [| x; y; z |];
      let (geo, _) = LL.geo_of_ecef LL.WGS84 !nav_ref_ecef in
      a.nav_ref <- Some (LL.utm_of LL.WGS84 geo);
  | "BOOZ2_NAV_STATUS" ->
      a.block_time <- ivalue "block_time";
      a.stage_time <- ivalue "stage_time";
      a.cur_block <- ivalue "cur_block";
      a.cur_stage <- ivalue "cur_stage";
      a.horizontal_mode <- check_index (ivalue "horizontal_mode") horiz_modes "AP_HORIZ";
      (*a.dist_to_wp <- sqrt (fvalue "dist2_wp")*)
  | "WP_MOVED_LTP" ->
      begin
        match a.nav_ref with
        Some nav_ref ->
          let x = foi32value "x" /. pos_frac
          and y = foi32value "y" /. pos_frac
          and z = foi32value "z" /. pos_frac in
          let (geo, _) = LL.geo_of_ecef LL.WGS84 (LL.ecef_of_ned !nav_ref_ecef (LL.make_ned [| y; x;-.z |])) in
          let p = LL.utm_of LL.WGS84 geo in
          update_waypoint a (ivalue "wp_id") p z;
        | None -> (); (** Can't use this message  *)
      end;
  | "WP_MOVED_LLA" ->
      begin
        match a.nav_ref with
        Some nav_ref ->
          let lat = foi32value "lat" /. gps_frac
          and lon = foi32value "lon" /. gps_frac in
          let p = LL.utm_of LL.WGS84 (LL.make_geo_deg lat lon) in
          update_waypoint a (ivalue "wp_id") p (foi32value "alt" /. 100.)
        | None -> () (** Can't use this message  *)
      end
  | _ -> ()

