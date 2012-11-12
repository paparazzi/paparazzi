(*
 * Basic GPS parameters simulation
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

open Stdlib
open Latlong

type state = {
    mutable availability : bool;
    wgs84 : Latlong.geographic;
    alt : float;
    time : float;
    climb : float;
    gspeed : float;
    course : float
  }


let climb_noise =
  let ng = Ocaml_tools.make_1st_order_noise_generator 0.9 0.1 in
  fun c -> c +. ng ()

let state = fun pos0 alt0 ->
  let last_x = ref 0. and last_y = ref 0. and last_z = ref 0.
  and last_gspeed = ref 0. and last_course = ref 0. and last_climb = ref 0.
  and last_t = ref 0.
  and tow = float (Latlong.get_gps_tow ()) in

  fun (x, y, z) t ->
    let dt = t -. !last_t in

    if dt > 0. then begin (** Compute derivatives *)
      let dx = x -. !last_x
      and dy = y -. !last_y in
      last_gspeed := sqrt (dx*.dx +. dy*.dy) /. dt;
      last_course := norm_angle (pi/.2. -. atan2 dy dx);
      last_climb := (z -. !last_z) /. dt
    end; (** Else use previous derivatives *)

    let utm0 = utm_of WGS84 !pos0 in
    let utm = utm_add utm0 (x, y) in
    let wgs84 = of_utm WGS84 utm
    and alt = !alt0 +. z in

    last_x := x;
    last_y := y;
    last_z := z;
    last_t := t;

    let course = if !last_course < 0. then !last_course +. 2. *. pi else !last_course in

    {
     wgs84 = wgs84;
     alt = alt;
     time = t +. tow;
     climb = climb_noise !last_climb;
     gspeed = !last_gspeed;
     course = course;
     availability = true;
   }

