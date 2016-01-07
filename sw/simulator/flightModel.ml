(*
 * Basic flight model for simulation
 *
 * Copyright (C) 2004-2006 Pascal Brisset, Antoine Drouin
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

open Stdlib
open Printf

let ios = fun x ->
  try int_of_string x with _ -> failwith (Printf.sprintf "int_of_string: '%s'" x)

type meter = float
type meter_s = float
type radian = float
type radian_s = float
type state = {
    start : float;
    mutable t : float;
    mutable x : meter;
    mutable y : meter;
    mutable z : meter;
    mutable z_dot : meter_s;
    mutable psi : radian; (* Trigonometric *)
    mutable phi : radian;
    mutable theta : radian;
    mutable theta_dot : radian_s;
    mutable phi_dot : radian_s;
    mutable psi_dot : radian_s;
    mutable delta_a : float;
    mutable delta_b : float;
    mutable thrust : float;
    mutable air_speed : meter_s
  }

module type SIG =
  sig
    val init : radian -> state
    val do_commands : state -> Stdlib.pprz_t array -> unit
    val nb_commands : int
    val nominal_airspeed : float (* m/s *)
    val max_bat_level : float (* V *)
    val roll_neutral_default : float (* rad *)
    val pitch_neutral_default : float (* rad *)
    val state_update : state -> float -> float * float * float -> float -> float -> unit
	(** [state_update nom_airspeed state (wind_x, wind_y, wind_z) on_ground dt] With m/s for wind and s for
	    dt *)
  end

let get_xyz state = (state.x, state.y, state.z)
let get_time state = state.t
let get_attitude state = (state.phi, state.theta, state.psi)
let get_pqr state = (state.phi_dot, state.theta_dot, state.psi_dot)

let get_air_speed state = state.air_speed
let set_air_speed state = fun s -> state.air_speed <- s

let g = 9.81

module Make(A:Data.MISSION) = struct
  open Data

  let section = fun name ->
    try
      ExtXml.child A.ac.airframe ~select:(fun x -> ExtXml.attrib x "name" = name) "section"
    with
      Not_found ->
	failwith (Printf.sprintf "Child 'section' with 'name=%s' expected in '%s'\n" name (Xml.to_string A.ac.airframe))

  let tag_define = fun sect name ->
    try
      (ExtXml.child sect ~select:(fun x -> ExtXml.attrib x "name" = name) "define")
    with
        Not_found ->
	      failwith (Printf.sprintf "Child 'define' with 'name=%s' expected in '%s'\n" name (Xml.to_string sect))

  let defined_value = fun sect name ->
    try
      (Xml.attrib (tag_define sect name) "value")
    with
        Not_found ->
	      failwith (Printf.sprintf "Child 'define' with 'name=%s' in '%s' has no value\n" name (Xml.to_string sect))

  let float_value = fun section s ->
    let x = (defined_value section s) in
    try float_of_string x with Failure "float_of_string" ->
      failwith (sprintf "float_of_string: %s" x)

  (* FIXME: refactor code_unit_scale of tag to pprz.ml *)
  let code_unit_scale_of_tag = function t ->
    (* if unit attribute is not specified don't even attempt to convert the units *)
    let u = try ExtXml.attrib t "unit" with _ -> failwith "Unit conversion error" in
    let cu = try ExtXml.attrib t "code_unit" with _ -> "" in
    (* default value for code_unit is rad[/s] when unit is deg[/s] *)
    try match (u, cu) with
        ("deg", "") -> PprzLink.scale_of_units u "rad" (* implicit conversion to rad *)
      | ("deg/s", "") -> PprzLink.scale_of_units u "rad/s" (* implicit conversion to rad/s *)
      | (_, "") -> failwith "Unit conversion error" (* code unit is not defined and no implicit conversion *)
      | (_,_) -> PprzLink.scale_of_units u cu (* try to convert *)
    with
        PprzLink.Unit_conversion_error s -> prerr_endline (sprintf "Unit conversion error: %s" s); flush stderr; exit 1
      | PprzLink.Unknown_conversion (su, scu) -> prerr_endline (sprintf "Warning: unknown unit conversion: from %s to %s" su scu); flush stderr; failwith "Unknown unit conversion"
      | _ -> failwith "Unit conversion error"

  let code_value = fun section s ->
    let t = (tag_define section s) in
    try
      let coef = try (code_unit_scale_of_tag t) with _ -> 1. in
      (ExtXml.float_attrib t "value") *. coef
    with
        _ ->
          failwith (Printf.sprintf "Can't convert 'define' with 'name=%s' in '%s' to floating point value\n" s (Xml.to_string section))

  let simu_section =
    try section "SIMU" with _ -> Xml.Element("", [], [])

  let roll_response_factor =
    try float_value simu_section "ROLL_RESPONSE_FACTOR" with _ -> 15.

  let pitch_response_factor =
    try float_value simu_section "PITCH_RESPONSE_FACTOR" with _ -> 1.

  let yaw_response_factor =
    try float_value simu_section "YAW_RESPONSE_FACTOR" with _ -> 1.

  let weight =
    try float_value simu_section "WEIGHT" with _ -> 1.

  let max_bat_level =
    try float_value (section "BAT") "MAX_BAT_LEVEL" with _ -> 12.5

  let h_ctrl_section =
    try section "HORIZONTAL CONTROL" with _ -> Xml.Element("",[],[])

  let max_phi =
    try code_value h_ctrl_section "ROLL_MAX_SETPOINT" with _ -> 0.7 (* rad *)
  let max_phi_dot = 0.25 (* rad/s *)
  let bound = fun x mi ma -> if x > ma then ma else if x < mi then mi else x



  let commands =
    try
      let l = ExtXml.child A.ac.airframe "commands" in
      let rec loop i = function
	  [] -> []
	| x::xs -> (ExtXml.attrib x "name", i)::loop (i+1) xs in
      loop 0 (Xml.children l)
    with
      Not_found ->
	failwith (Printf.sprintf "Child 'commands' expected in '%s'\n" (Xml.to_string A.ac.airframe))

  let command = fun n ->
    try List.assoc n commands with
      Not_found -> failwith (sprintf "Unknown command '%s'" n)

  let misc_section = section "MISC"

  let infrared_section = try section "INFRARED" with _ -> Xml.Element("",[],[])

  let nominal_airspeed = code_value misc_section "NOMINAL_AIRSPEED"
  let maximum_airspeed = try code_value misc_section "MAXIMUM_AIRSPEED" with _ -> nominal_airspeed *. 1.5
  let max_power = try code_value misc_section "MAXIMUM_POWER" with _ -> 5. *. maximum_airspeed *. weight

  let roll_neutral_default = try code_value infrared_section "ROLL_NEUTRAL_DEFAULT" with _ -> 0.
  let pitch_neutral_default = try code_value infrared_section "PITCH_NEUTRAL_DEFAULT" with _ -> 0.


  let vert_ctrl_section = try section "VERTICAL CONTROL" with _ -> Xml.Element("",[],[])
  let cruise_thrust = try float_value vert_ctrl_section "AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE" with _ -> 0.45

  let min_thrust =  0
  let max_thrust =  max_pprz

  let command_throttle = command "THROTTLE"
  let command_roll = command "ROLL"
  let command_pitch = command "PITCH"

  let float_attrib = fun x a -> float_of_string (ExtXml.attrib x a)
  let int_attrib = fun x a -> int_of_string (ExtXml.attrib x a)


  let do_commands = fun state commands ->
    let c_lda = 4e-5 in (* FIXME *)
    state.delta_a <- c_lda *. float commands.(command_roll);
    state.delta_b <- float commands.(command_pitch);
    state.thrust <- (float (commands.(command_throttle) - min_thrust) /. float (max_thrust - min_thrust))

  let nb_commands = 10 (* FIXME *)

  let init route = {
    start = Unix.gettimeofday (); t = 0.; x = 0.; y = 0. ; z = 0.;
    psi = route; phi = 0.; phi_dot = 0.; theta_dot = 0.; psi_dot = 0.;
    delta_a = 0.; delta_b = 0.; thrust = 0.; air_speed = 0.;
    theta = 0.; z_dot = 0.
  }

(* Minimum complexity *)
(*
   Johnson, E.N., Fontaine, S.G., and Kahn, A.D.,
   “Minimum Complexity Uninhabited Air Vehicle Guidance And Flight Control System,”
   Proceedings of the 20th Digital Avionics Systems Conference, 2001.
   http://www.ae.gatech.edu/~ejohnson/JohnsonFontaineKahn.pdf

   Johnson, E.N. and Fontaine, S.G.,
   “Use Of Flight Simulation To Complement Flight Testing Of Low-Cost UAVs,”
   Proceedings of the AIAA Modeling and Simulation Technology Conference, 2001.
   http://www.ae.gatech.edu/~ejohnson/AIAA%202001-4059.pdf
 *)
  let state_update = fun state nominal_airspeed (wx, wy, wz) agl dt ->
    let now = state.t +. dt in
    if state.air_speed = 0. && state.thrust > 0. then
      state.air_speed <- nominal_airspeed;

    if agl >= -3. && state.air_speed > 0. then begin
      let v2 = state.air_speed**2.
      and vn2 = nominal_airspeed ** 2.  in

      let phi_dot_dot = roll_response_factor *. state.delta_a *. v2 /. vn2 -. state.phi_dot in
      state.phi_dot <- state.phi_dot +. phi_dot_dot *. dt;
      state.phi_dot <- bound state.phi_dot (-.max_phi_dot) max_phi_dot;
      state.phi <- norm_angle (state.phi +. state.phi_dot *. dt);
      state.phi <- bound state.phi (-.max_phi) max_phi;

      state.psi_dot <- -. g /. state.air_speed *. tan (yaw_response_factor *. state.phi);
      state.psi <- norm_angle (state.psi +. state.psi_dot *. dt);

      (* Aerodynamic pitching moment coeff, proportional to elevator;
        No Thrust moment, so null (0) for steady flight *)
      let c_m = 5e-7 *.state.delta_b in
      let theta_dot_dot = pitch_response_factor *. c_m *. v2 -. state.theta_dot in
      state.theta_dot <- state.theta_dot +. theta_dot_dot *. dt;
      state.theta <- state.theta +. state.theta_dot *. dt;

      (* Flight path angle *)
      let gamma = atan2 state.z_dot state.air_speed in

      (* Cz proportional to angle of attack *)
      let alpha = state.theta -. gamma in
      let c_z = 0.2 *. alpha +. g /. vn2 in

      (* Lift *)
      let lift = c_z *. state.air_speed**2. in
      let z_dot_dot = lift /. weight *. cos state.theta *. cos state.phi -. g in
      state.z_dot <- state.z_dot +. z_dot_dot *.dt;
      state.z <- state.z +. state.z_dot *. dt;

      (* Constant Cx, drag to get expected cruise and maximum throttle *)
      let drag = cruise_thrust +. (v2 -. vn2)*.(1.-. cruise_thrust)/.(maximum_airspeed ** 2. -. vn2) in
      let air_speed_dot = max_power /. state.air_speed *. (state.thrust -. drag) /. weight -. g *. sin gamma in
      state.air_speed <- state.air_speed +. air_speed_dot *. dt;
      state.air_speed <- max state.air_speed 10.; (* Avoid stall *)

      (* FIXME: wind effect should be in the forces *)
      let x_dot = state.air_speed *. cos state.psi +. wx
      and y_dot = state.air_speed *. sin state.psi +. wy in
      state.x <- state.x +. x_dot *. dt;
      state.y <- state.y +. y_dot *. dt;
      state.z <- state.z +. wz *. dt
    end;
    state.t <- now
end (* Make functor *)
