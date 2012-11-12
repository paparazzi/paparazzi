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

type meter = float
type meter_s = float
type radian = float
type radian_s = float
type state

val get_xyz : state -> meter * meter * meter
val get_time : state -> float
val get_attitude : state -> radian * radian * radian
val get_pqr : state -> radian_s * radian_s * radian_s

val set_air_speed : state -> meter_s -> unit
val get_air_speed : state -> meter_s

module type SIG =
  sig
    val init : radian -> state
    val do_commands : state -> Stdlib.pprz_t array -> unit
    val nb_commands : int
    val nominal_airspeed : float (* m/s *)
    val max_bat_level : float (* V *)
    val roll_neutral_default : float (* rad *)
    val pitch_neutral_default : float (* rad *)
    val state_update : state -> float -> float * float *float -> float -> float -> unit
	(** [state_update nom_airspeed state (wind_x, wind_y, wind_z) on_ground dt] With m/s for wind and s for
	    dt *)
  end

module Make : functor (A : Data.MISSION) -> SIG
