(*
 * Copyright (C) 2010-2012 ENAC
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

(* Global variables and utility functions for server parsers *)

open Printf
open Aircraft
open Latlong
module LL = Latlong

exception Telemetry_error of string * string

let hostname = ref "localhost"

(** FIXME: Should be read from messages.xml *)
let fixedwing_ap_modes = [|"MANUAL";"AUTO1";"AUTO2";"HOME";"NOGPS";"FAIL"|]
let rotorcraft_ap_modes = [|"SAFE";"KILL";"RATE";"ATT";"R_RCC";"A_RCC";"ATT_C";"R_ZH";"A_ZH";"HOVER";"HOV_C";"H_ZH";"NAV";"RC_D"|]
let _AUTO2 = 2
let rc_modes = [|"OK";"LOST";"NONE"|]
let fbw_modes = [|"MANUAL";"AUTO";"FAILSAFE"|]
let gaz_modes = [|"MANUAL";"GAZ";"CLIMB";"ALT"|]
let lat_modes = [|"MANUAL";"ROLL_RATE";"ROLL";"COURSE"|]
let gps_modes = [|"NOFIX";"DRO";"2D";"3D";"GPSDRO"|]
let state_filter_modes = [|"UNKNOWN";"INIT";"ALIGN";"OK";"GPS_LOST";"IMU_LOST";"COV_ERR";"IR_CONTRAST";"ERROR"|]
let attitude_filter_modes = [|"UNKNOWN";"INIT";"ALIGN";"OK";"IMU_LOST";"COV_ERR";"IR_CONTRAST_LOW";"ERROR"|]
let position_filter_modes = [|"UNKNOWN";"INIT";"OK";"GPS_LOST";"COV_ERR";"ERROR"|]
let _3D = 3
let gps_hybrid_modes = [|"OFF";"ON"|]
let horiz_modes = [|"WAYPOINT";"ROUTE";"CIRCLE";"ATTITUDE"|]
let if_modes = [|"OFF";"DOWN";"UP"|]

(** Resolution for integer values *)
let pos_frac = 2. ** 8.
let speed_frac = 2. ** 19.
let angle_frac = 2. ** 12.
let gps_frac = 1e7

(** Utility functions *)
let string_of_values = fun values ->
  String.concat " " (List.map (fun (_, v) -> Pprz.string_of_value v) values)

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
  let new_wp = { altitude = alt; wp_geo = p } in
  try
    let prev_wp = Hashtbl.find ac.waypoints wp_id in
    if new_wp <> prev_wp then
      Hashtbl.replace ac.waypoints wp_id new_wp
  with
    Not_found ->
      Hashtbl.add ac.waypoints wp_id new_wp

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


