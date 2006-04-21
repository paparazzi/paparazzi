(*
 *  $Id$
 *
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
    mutable psi : radian; (* Trigonometric *)
    mutable phi : radian;
    mutable phi_dot : radian_s;
    mutable delta_a : float;
    mutable thrust : float;
    mutable air_speed : meter_s
  }

let init route = {
  start = Unix.gettimeofday (); t = 0.; x = 0.; y = 0. ; z = 0.;
  psi = route; phi = 0.; phi_dot = 0.;
  delta_a = 0.; thrust = 0.; air_speed = 0.
}

let get_xyz state = (state.x, state.y, state.z)
let get_time state = state.t
let get_phi state = state.phi
let get_attitude state = (state.phi, 0., state.psi)

let set_air_speed state x = state.air_speed <- x

let drag = 0.45
let c_lp = -10.
let g = 9.81

module Make(A:Data.MISSION) = struct
  open Data

  let section = fun name ->
    try
      ExtXml.child A.ac.airframe ~select:(fun x -> ExtXml.attrib x "name" = name) "section"
    with
      Not_found ->
	failwith (Printf.sprintf "Child 'section' with 'name=%s' expected in '%s'\n" name (Xml.to_string A.ac.airframe))


  let simu_section = section "SIMU"

  let defined_value = fun sect name ->
    try
      (Xml.attrib (ExtXml.child sect ~select:(fun x -> ExtXml.attrib x "name" = name) "define") "value")
    with
      Not_found ->
	failwith (Printf.sprintf "Child 'define' with 'name=%s' expected in '%s'\n" name (Xml.to_string sect))

  let float_value = fun section s ->  float_of_string (defined_value section s)

  let roll_response_factor = float_value simu_section "ROLL_RESPONSE_FACTOR"

  let yaw_response_factor = float_value simu_section "YAW_RESPONSE_FACTOR"

  let weight = float_value simu_section "WEIGHT"

  let max_phi = 0.7 (* rad *)
  let bound = fun x mi ma -> if x > ma then ma else if x < mi then mi else x



(* Minimum complexity *)
(*
   http://controls.ae.gatech.edu/papers/johnson_dasc_01.pdf
   http://controls.ae.gatech.edu/papers/johnson_mst_01.pdf
 *)
  let state_update = fun state (wx, wy) dt ->
    let now = state.t +. dt in
    if state.air_speed > 0. then begin
      let phi_dot_dot = roll_response_factor *. state.delta_a +. c_lp *. state.phi_dot /. state.air_speed in
      state.phi_dot <- state.phi_dot +. phi_dot_dot *. dt;
      state.phi <- bound (state.phi +. state.phi_dot *. dt) (-.max_phi) max_phi;
      let psi_dot = -. g /. state.air_speed *. tan (yaw_response_factor *. state.phi) in
      state.psi <- norm_angle (state.psi +. psi_dot *. dt);
      let dx = state.air_speed *. cos state.psi *. dt +. wx *. dt
      and dy = state.air_speed *. sin state.psi *. dt +. wy *. dt in
      state.x <- state.x +.dx ;
      state.y <- state.y +. dy;
      let gamma = (state.thrust -. drag) /. weight in
      let dz = sin gamma *. state.air_speed *. dt in
      state.z <- state.z +. dz
    end;
    state.t <- now




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

  let infrared_section = section "INFRARED"

  let nominal_airspeed = float_of_string (defined_value misc_section "NOMINAL_AIRSPEED")

  let adc_roll_neutral = ios (defined_value infrared_section "ADC_ROLL_NEUTRAL")
  let roll_neutral_default = rad_of_deg (float_value infrared_section "ROLL_NEUTRAL_DEFAULT")

  let min_thrust =  0
  let max_thrust =  max_pprz
      
  let command_throttle = command "THROTTLE"
  let command_roll = command "ROLL"
     
  let float_attrib = fun x a -> float_of_string (ExtXml.attrib x a)
  let int_attrib = fun x a -> int_of_string (ExtXml.attrib x a)


  let do_commands = fun state commands ->
    let c_lda = 4e-5 in (* FIXME *)
    state.delta_a <- -. c_lda *. float commands.(command_roll);
    state.thrust <- (float (commands.(command_throttle) - min_thrust) /. float (max_thrust - min_thrust))

  let nb_commands = 10 (* FIXME *)
end
