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
val max_pprz : pprz_t
val pi : float
val norm_angle : float -> float
val deg_of_rad : float -> float
val rad_of_deg : float -> float
val set_float : string -> float ref -> string -> string * Arg.spec * string
val set_string : string -> string ref -> string -> string * Arg.spec * string

class type value = object method value : float end

val timer : ?scale:#value -> float -> (unit -> 'a) -> unit
(** [timer ?time_accel period_s callback] Non derivating periodic timer *)
