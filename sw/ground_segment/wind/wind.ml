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

(*
 * 
 * Estimate wind by analysing aircrafts trajectories 
 *
 * Author : Nicolas Barnier - barnier@recherche.enac.fr 
 *
 *)

let debug = false

open Printf

let (//) = Filename.concat
let conf_xml = Xml.parse_file (Env.paparazzi_home // "conf" // "conf.xml")
let xml_ground = ExtXml.child conf_xml "ground"
let ivy_bus = ref (ExtXml.attrib xml_ground "ivy_bus")

open Geometry_2d

type point_val = {p : pt_2D; f : float}

type triangle = {a: point_val; b: point_val; c: point_val}

let bary t = barycenter [t.a.p; t.b.p; t.c.p]

let init w_init step f =
  let pb = vect_add w_init {x2D = step; y2D = 0.}
  and pc = vect_add w_init {x2D = 0.; y2D = step} in
  {a = {p = w_init; f = f w_init};
   b = {p = pb; f = f pb};
   c = {p = pc; f = f pc}}

let shift pa fa t = {a = {p = pa; f = fa}; b = t.a; c = t.b}

let shiftpv pf t = shift pf.p pf.f t

let calcnew p b alpha = vect_add_mul_scal alpha b (vect_make b p)

let triangle_sort t =
  let abc = [|t.a; t.b; t.c|] in
  Array.sort (fun t1 t2 -> compare t2.f t1.f) abc;
  {a = abc.(0); b = abc.(1); c = abc.(2)}

let norme2 p = p.x2D *. p.x2D +. p.y2D *. p.y2D

let simplex p fmax step max_iter precision =
  let f x = -. (fmax x) in

  let rec loop num_iter vs =
    if num_iter < max_iter && norme2 (vect_make vs.a.p vs.c.p) > precision then begin
      begin if debug then
	let pa = cart2polar vs.a.p in
	Printf.printf "%f %f %f\n" pa.theta2D pa.r2D (-. vs.a.f) end;

      let vb = bary vs in
      let vr = calcnew vs.c.p vb (-1.) in
      let fvr = f vr in
      let new_vs =
	if fvr > vs.a.f then
	  let ve = calcnew vs.c.p vb (-2.) in
	  let fve = f ve in
	  if fve > fvr then shift ve fve vs
	  else shift vr fvr vs
	else
	  let vc = calcnew vs.c.p vb 0.5 in
	  let fvc = f vc in
	  if fvc > vs.b.f || fvr > vs.b.f then
	    let v = if fvr > fvc then {p = vr; f = fvr} else {p = vc; f = fvc} in
	    if v.f <= vs.b.f then {vs with c = v}
	    else if v.f > vs.a.f then shiftpv v vs
	    else {vs with b = v; c = vs.b}
	  else
	    let vcb = calcnew vs.b.p vs.a.p 0.5
	    and vcc = calcnew vs.c.p vs.a.p 0.5 in
	    triangle_sort {vs with b = {p = vcb; f = f vcb}; c = {p = vcc; f = f vcc}} in

      loop (num_iter + 1) new_vs end
    else vs.a in

  if debug then Printf.printf "%f %f %f\n" p.x2D p.y2D (fmax p);
  let vs = init p step f in
  let vs = triangle_sort vs in
  loop 0 vs


let isotropic_mean wind speeds =
  let n = Array.length speeds in
  let air_speeds = Array.map (fun speed -> cart2polar (vect_sub speed wind)) speeds in
  let weights =
    Array.map
      (fun air ->
	let sum =
	  Array.fold_left
	    (fun acc airj ->
	      acc +. norm_angle_rad (abs_float (air.theta2D -. airj.theta2D)) /. m_pi)
	    0. air_speeds in
	sum /. (float (n-1)))
      air_speeds in
  let mean = ref 0. in
  for i = 0 to n-1 do
    mean := !mean +. vect_norm (vect_sub speeds.(i) wind) *. weights.(i) done;
  !mean /. float n

let isotropic_wind wind_init speeds precision =
  let n = Array.length speeds in
  let mean wind =
    let air_speeds = Array.map (fun speed -> cart2polar (vect_sub speed wind)) speeds in
    let weights =
      Array.mapi
	(fun i airi ->
	  let sum = ref 0. in
	  for j = 0 to n-1 do
	    if j <> i then
	      sum := !sum +.
		  norm_angle_rad (abs_float (airi.theta2D -. air_speeds.(j).theta2D)) /. m_pi
	  done;
	  !sum /. (float (n-1)))
	air_speeds in
    let sum_weights = Array.fold_left (+.) 0. weights in
  
    let mean = ref 0. in
    for i = 0 to n-1 do
      mean := !mean +. vect_norm (vect_sub speeds.(i) wind) *. weights.(i) done;
    (!mean /. sum_weights, sum_weights, weights) in

  let nb_calls = ref 0 in
  let cost wind =
    incr nb_calls;
    let (m, sum_weights, weights) = mean wind in
    let sum = ref 0. in
    for i = 0 to n-1 do
      let err = weights.(i) *. (vect_norm (vect_sub speeds.(i) wind) -. m) in
      sum := !sum +. err *. err done;
    !sum /. sum_weights in

  let step = 2. and max_iter = 100 in
  let wind = simplex wind_init cost step max_iter precision in
  if debug then Printf.printf "nb calls: %d\n" !nb_calls;

  let (mean, _, _) = mean wind.p in
  (wind.p, mean, wind.f)


(* val wind : Geometry_2d.pt_2D -> Geometry_2d.pt_2D array -> float
  -> (Geometry_2d.pt_2Dfloat * float * float) *)
(** [wind wind_init speeds precision] returns the wind and air speed mean and std dev. *)

let wind wind_init speeds precision =
  let mean wind =
    let sum =
      Array.fold_left (fun acc speed -> acc +. vect_norm (vect_sub speed wind)) 0. speeds in
    sum /. float (Array.length speeds) in
  
  let nb_calls = ref 0 in
  let cost wind =
    incr nb_calls;
    let m = mean wind in
    let sum =
      Array.fold_left
	(fun acc speed ->
	  let err = vect_norm (vect_sub speed wind) -. m in
	  acc +. err *. err)
	0. speeds in
    sum /. float (Array.length speeds) in

  let step = 2. and max_iter = 100 in
  let wind = simplex wind_init cost step max_iter precision in
  if debug then Printf.printf "nb calls: %d\n" !nb_calls;

  (wind.p, mean wind.p, wind.f)


 
let _ =
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), (sprintf "Bus\tDefault is %s" !ivy_bus)] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anything with %s\n" x)
    "Usage: ";

  let precision = 1e-3 in
  let speeds = ref [] and wind_init = ref null_vector in

  let on_wind_command _ args =
    if Str.string_match (Str.regexp "clear") args.(0) 0 then begin
      speeds := []; 
      wind_init := null_vector
    end
  in

  let _on_wind_clear _ args = speeds := []; wind_init := null_vector in

  let on_flight_param = fun _ args ->
(*    Array.iter (printf "%s ") args; printf "\n%!"; *)
    let r = float_of_string args.(4)
    and theta = heading_of_to_angle_rad (deg2rad (float_of_string args.(5))) in
    let speed = polar2cart {r2D = r; theta2D = theta} in
    speeds := speed :: !speeds in

  let on_wind_req _ args =
    let speeds = Array.of_list !speeds in
    if Array.length speeds >= 3 then begin
      let (wind, mean, stddev) = wind !wind_init speeds precision in
      wind_init := wind;
      let wind_polar = cart2polar wind in
      let wind_cap_deg = rad2deg (wind_dir_from_angle_rad wind_polar.theta2D) in
      Ivy.send
	(sprintf "ground WIND_RES %s %f %f %f %f" args.(0) wind_cap_deg wind_polar.r2D mean stddev)
    end in

  let on_wind_iso _ args =
    let speeds = Array.of_list !speeds in
    if Array.length speeds >= 3 then begin
      let (wind, mean, stddev) = isotropic_wind !wind_init speeds precision in
      wind_init := wind;
      let wind_polar = cart2polar wind in
      let wind_cap_deg = rad2deg (wind_dir_from_angle_rad wind_polar.theta2D) in
      Ivy.send
	(sprintf "ground WIND_RES %s %f %f %f %f" args.(0) wind_cap_deg wind_polar.r2D mean stddev)
    end in

  let on_aircrafts = fun _ args ->
    let aclist =  args.(0) in
    let first_ac = aclist (*String.sub aclist 0 (String.index aclist ',')*) in
    ignore
      (Ivy.bind on_flight_param
	 (sprintf "%s +FLIGHT_PARAM (.*) (.*) (.*) (.*) (.*) (.*) (.*) (.*)" first_ac)) in
  
  Ivy.init "Paparazzi Wind" "READY" (fun _ _ -> ());
  ignore (Ivy.bind on_aircrafts "ground AIRCRAFTS (.*)");
  ignore (Ivy.bind on_wind_req "WIND_REQ (.*)");
  ignore (Ivy.bind on_wind_iso "WIND_ISO (.*)");
  ignore (Ivy.bind on_wind_command "WIND_COMMAND (.*)");
  Ivy.start !ivy_bus;

  GMain.Main.main ()
