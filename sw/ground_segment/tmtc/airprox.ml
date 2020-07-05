(*
 * Air Proximity Alert Module
 *
 * Copyright (C) ENAC, Pascal Brisset, Antoine Drouin
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

open Aircraft_server
open Latlong

module Alerts_Pprz = PprzLink.Messages(struct let name = "alert" end)

(** computes distance between 2d points                                       *)
let distance = fun (x1, y1) (x2, y2) ->
  sqrt (((x1 -. x2) **2.) +. ((y1 -. y2)**2.))


(** check if there is an airprox between aircraft1 and aircraft2              *)
(** if the altitude difference is less than 10 meters                         *)
(**    and if the horizontal distance is less than 100 meters (5s between     *)
(**    2 aircraft at 10m/s                                                    *)
let airprox = fun aircraft1 aircraft2 ->
  let p1 = Latlong.utm_of Latlong.WGS84 aircraft1.pos in
  let p2 = Latlong.utm_of Latlong.WGS84 aircraft2.pos in
  let x1 = p1.utm_x and x2 = p2.utm_x and
      y1 = p1.utm_y and y2 = p2.utm_y and
      z1 = aircraft1.alt and z2 = aircraft2.alt in
  let alt_difference = abs_float (z1 -. z2) and
      dist =  distance (x1, y1) (x2, y2) in
  ((alt_difference < 10.0) && (dist < 100.0))

(** return airprox level                                                      *)
(** level is warning if the distance between both aircraft is increasing      *)
(** level is crictical otherwise                                              *)
let airprox_level = fun aircraft1 aircraft2 ->
  let p1 = Latlong.utm_of Latlong.WGS84 aircraft1.pos in
  let p2 = Latlong.utm_of Latlong.WGS84 aircraft2.pos in
  let x1 = p1.utm_x and x2 = p2.utm_x and
      y1 = p1.utm_y and y2 = p2.utm_y in
  let d0 = distance (x1, y1) (x2, y2) in
  let course1 = aircraft1.course and course2 = aircraft2.course and
      speed1 = aircraft1.gspeed and speed2 = aircraft2.gspeed in
  (** FIXME: must have the real angle and not a course *)
  let halfpi = Latlong.pi /. 2. in
  let vx1 = speed1 *. (cos (halfpi -. course1)) and
      vx2 = speed2 *. (cos (halfpi -. course2)) and
      vy1 = speed1 *. (sin (halfpi -. course1)) and
      vy2 = speed2 *. (sin (halfpi -. course2)) in
  let d1 = distance
    (x1+. vx1 *. 0.2, x2+. vx2 *. 0.2)
    (y1+. vy1 *. 0.2, y2+. vy2 *. 0.2) in
  if d1 < d0 then "CRITICAL" else "WARNING"

(** send a airprox alert on ivy if there is an airprox between ac_name1 and   *)
(**    ac_name2                                                               *)
type alert_level = string option
let check_airprox = fun ac1 ac2 ->
  if airprox ac1 ac2 then Some (airprox_level ac1 ac2) else None
