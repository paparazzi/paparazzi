(*
 *  $Id$
 *
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


let climb_noise = fun c -> c +. Random.float 1. -. 0.5

let leap_seconds = 14 (* http://www.leapsecond.com/java/gpsclock.htm *)

  
let state = fun pos0 alt0 ->
  let last_x = ref 0. and last_y = ref 0. 
  and last_t = ref 0. and last_z = ref 0. in
  let utc = Unix.gmtime (Unix.gettimeofday ()) in
  let tow = float (((utc.Unix.tm_wday*24 + utc.Unix.tm_hour)*60+utc.Unix.tm_min)*60+utc.Unix.tm_sec + leap_seconds) in

  fun (x, y, z) t ->
   let dx = x -. !last_x
    and dy = y -. !last_y
    and dt = t -. !last_t in
    let gspeed = sqrt (dx*.dx +. dy*.dy) /. dt
    and course = norm_angle (pi/.2. -. atan2 dy dx)
    and climb = (z -. !last_z) /. dt in

    let utm0 = utm_of WGS84 !pos0 in
    let utm = utm_add utm0 (x, y) in
    let wgs84 = of_utm WGS84 utm
    and alt = !alt0 +. z in

    last_x := x;
    last_y := y;
    last_z := z;
    last_t := t;

    let course = if course < 0. then course +. 2. *. pi else course in
    
    {
     wgs84 = wgs84;
     alt = alt;
     time = t +. tow;
     climb = climb_noise climb;
     gspeed = gspeed;
     course = course;
     availability = true;
   }
      
