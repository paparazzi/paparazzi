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
  z : float}

type euler_angle = {
  r : float;
  p : float;
  y : float}

val quaternion_from_angle : float -> float -> float -> quaternion

val quaternion_to_angle : quaternion -> euler_angle

val multiply_quaternion : quaternion -> quaternion -> quaternion