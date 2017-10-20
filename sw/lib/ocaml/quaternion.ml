(*
 * Quaternion object for with conversion to and from Tait-Bryan angles
 *
 * Copyright (C) 2017 Rijesh Augustine
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

type quaternion = {
  w : float;
  x : float;
  y : float;
  z : float};;

type euler_angle = {
  r : float;
  p : float;
  y : float};;

let quaternion_from_angle roll pitch yaw = 
  let cy = cos (yaw *. 0.5)
  and sy = sin (yaw *. 0.5)
  and cr = cos (-.roll *. 0.5)
  and sr = sin (-.roll *. 0.5)
  and cp = cos (pitch *. 0.5)
  and sp = sin (pitch *. 0.5) in

  { w = cy *. cr *. cp +. sy *. sr *. sp;
    x = cy *. sr *. cp -. sy *. cr *. sp;
    y = cy *. cr *. sp +. sy *. sr *. cp;
    z = sy *. cr *. cp -. cy *. sr *. sp};;
  
let quaternion_to_angle q =
  let sinr = 2.0 *. (q.w *. q.x +. q.y *. q.z)
  and cosr = 1.0 -. 2.0 *. (q.x *. q.x +. q.y *. q.y) in
    
  let roll = atan2 sinr cosr in

  (*pitch (y-axis rotation) *)
  let sinp = 2.0 *. (q.w *. q.y -. q.z *. q.x) in

  let absF (f:float) = if f > 0.0 then f else (f *. -1.0) in

  let pitch =
    if absF sinp >= 1.0 then
      copysign (3.1415926535897932384626433832795 /. 2.0) sinp
    else
      asin sinp in

  (*yaw (z-axis rotation)*)
  let siny = +2.0 *. (q.w *. q.z +. q.x *. q.y)
  and cosy = +1.0 -. 2.0 *. (q.y *. q.y +. q.z *. q.z) in
  let yaw = atan2 siny cosy in

  {  r = roll;
     p = pitch;
     y = yaw};;

let multiply_quaternion q1 q2 = 
  { x = (  q1.x) *. q2.w +. q1.y *. q2.z -. q1.z *. q2.y +. q1.w *. q2.x;
    y = (-.q1.x) *. q2.z +. q1.y *. q2.w +. q1.z *. q2.x +. q1.w *. q2.y;
    z = (  q1.x) *. q2.y -. q1.y *. q2.x +. q1.z *. q2.w +. q1.w *. q2.z;
    w = (-.q1.x) *. q2.x -. q1.y *. q2.y -. q1.z *. q2.z +. q1.w *. q2.w};;
