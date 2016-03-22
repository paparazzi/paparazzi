(*
 * Wind estimation by analysing aircrafts trajectories
 *
 * Copyright (C) 2004 ENAC, Nicolas Barnier, Pascal Brisset, Antoine Drouin
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
  Wind speed and direction are estimated from a dataset of ground speeds,
  with the hypothesis that the airspeed is constant. This estimation is computed
  by solving an optimization problem. The Nelder-Mead method is used
  (http://en.wikipedia.org/wiki/Nelder-Mead_method).

  Let GS(i) a set of n recorded ground speed vectors and W the wind speed.
  The norm of the (hypothetically constant) mean airspeed is

  as = 1/n sum(norm(GS(i)-W))

  Let

  stderr = 1/n sum (norm(GS(i)-W)-as)^2

  The minimization of stderr, on the W decision variable, returns an estimation
  of W.

  Remarks:
  - GS(i) actually is the sequence of the _last_ recorded ground speeds.
  - In the "isotropic" implementation, each sample is weighted by its relative
  difference in direction to the other samples.
*)


type id = string

let (//) = Filename.concat
let conf_xml = ExtXml.parse_file (Env.paparazzi_home // "conf" // "conf.xml")

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


(** Nelder-Mead optimization *)
let simplex p fmax step max_iter precision =
  let f x = -. (fmax x) in

  let rec loop num_iter vs =
    if num_iter < max_iter && norme2 (vect_make vs.a.p vs.c.p) > precision then begin
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

  let vs = init p step f in
  let vs = triangle_sort vs in
  loop 0 vs


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

  let (mean, _, _) = mean wind.p in
  (wind.p, mean, -.wind.f)


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

  (wind.p, mean wind.p, -.wind.f)

type wind_ac = {
  speeds : Geometry_2d.pt_2D option array;
  mutable index : int;
  mutable length : int;
  mutable wind_init : Geometry_2d.pt_2D
}

let h = Hashtbl.create 17

let create_wind_ac max_nb_sample =
  {speeds = Array.make max_nb_sample None ; wind_init = null_vector; index = 0; length = 0}

let new_ac = fun id max_nb_sample ->
  Hashtbl.add h id (create_wind_ac max_nb_sample)

let clear id =
  let wind_ac = Hashtbl.find h id in
  wind_ac.index <- 0;
  wind_ac.length <- 0;
  wind_ac.wind_init <- null_vector

let precision = 1e-3

let update = fun id r course ->
  let theta = heading_of_to_angle_rad course in
  let speed = polar2cart {r2D = r; theta2D = theta} in
  let wind_ac = Hashtbl.find h id in
  let i = truncate (float (Array.length wind_ac.speeds) *. course /. 2. /. Latlong.pi) in
  (*  Printf.printf "i=%d\n%!" i; *)
  wind_ac.speeds.(i) <- Some speed

let compute = fun compute_wind id ->
  try
    let wind_ac = Hashtbl.find h id in
    let speeds = List.fold_right (fun s r -> match s with Some s -> s::r | None -> r) (Array.to_list wind_ac.speeds) [] in
    let speeds = Array.of_list speeds in
    (*     Printf.printf "l=%d\n%!" (Array.length speeds); *)
    if Array.length speeds >= 3 then begin
      let wind_init = wind_ac.wind_init in
      let (wind, mean, stddev) = compute_wind wind_init speeds precision in
      wind_ac.wind_init <- wind;
      let wind_polar = cart2polar wind in
      let wind_cap_rad = wind_dir_from_angle_rad wind_polar.theta2D
      and nb_sample = Array.length speeds in
      (wind_cap_rad, wind_polar.r2D, mean, stddev, nb_sample)
    end else
      failwith (Printf.sprintf "Wind.on_wind_compute: ac %s not enough data\n%!" id)
  with Not_found ->
    failwith (Printf.sprintf "Wind.on_wind_compute: ac %s unknown\n%!" id)


let get = fun id -> compute wind id
let get_iso = fun id -> compute isotropic_wind id

