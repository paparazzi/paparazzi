(*
 * Utilities for the simulators
 *
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

type pprz_t = int

let max_pprz = 9600

let pi = 4. *. atan 1.
let rec norm_angle = fun x ->
  if x > pi then norm_angle (x-.2.*.pi)
  else if x < -.pi then norm_angle (x+.2.*.pi)
  else x

let deg_of_rad = fun rad -> rad /. pi *. 180.

let rad_of_deg = fun x -> x /. 180. *. pi

let set_float = fun option var name ->
  (option, Arg.Set_float var, Printf.sprintf "%s (%f)" name !var)
let set_string = fun option var name ->
  (option, Arg.Set_string var, Printf.sprintf "%s (%s)" name !var)

let ms x = max 0 (truncate (1000.*.x))
(* Non derivating timer *)
class type value = object method value : float end

let timer ?scale p f =
  let scale =
    match scale with
      None -> object method value = 1. end
    | Some s -> (s :> value) in
  let rec loop = fun expected ->
    let next = expected +. p /. scale#value in
    let dt = ms (next -. Unix.gettimeofday()) in
    if dt < 1 then begin (* No timer needed, simply loop *)
      f (); loop next
    end else
      GMain.Timeout.add
	~ms:dt
	~callback:(fun () ->
	  ignore (loop next);
	  f ();
	  false) in
  ignore (loop (Unix.gettimeofday()))
