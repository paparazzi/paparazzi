(*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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

open Latlong

type intruder = {
  id : string;
  name : string;
  mutable pos : Latlong.geographic;
  mutable unix_time : float;
  mutable itow : int64; (* ms *)
  mutable roll    : float;
  mutable pitch   : float;
  mutable heading  : float; (* rad, CW 0=N *)
  mutable gspeed  : float; (* m/s *)
  mutable airspeed : float; (* m/s *)
  mutable course : float; (* rad *)
  mutable alt     : float;
  mutable agl     : float;
  mutable climb   : float
}

let new_intruder = fun id name ->
  {
    id = id; name = name;
    pos = { Latlong.posn_lat = 0.; posn_long = 0. };
    unix_time = 0.; itow = Int64.of_int 0;
    roll=0.; pitch=0.; heading=0.;
    gspeed=0.; airspeed= -1.; course = 0.;
    alt=0.; climb=0.; agl = 0.
  }
