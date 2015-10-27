(*
 * Copyright (C) 2015 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 *)

(*open Latlong*)

type intruder = {
  intruder_track : MapTrack.track;
  mutable last_update : float
}

(*let intruders = (string, intruder) Hashtbl.t*)
let intruders = Hashtbl.create 1

let new_intruder = fun id name time geomap ->
  let track = new MapTrack.track ~size:200 ~icon:"intruder" ~name ~show_carrot:false id geomap in
  let intruder = { intruder_track = track; last_update = time } in
  Hashtbl.add intruders id intruder

let remove_intruder = fun id ->
  try
    let intruder = Hashtbl.find intruders id in
    intruder.intruder_track#destroy ();
    Hashtbl.remove intruders id
  with _ -> () (* no intruder *)

let update_intruder = fun id wgs84 heading alt speed climb time ->
  try
    let intruder = Hashtbl.find intruders id in
    intruder.intruder_track#move_icon wgs84 heading alt speed climb;
    intruder.last_update <- time;
  with _ -> () (* no intruder, add a new one ? *)

let intruder_exist = fun id ->
  Hashtbl.mem intruders id

(* remove old intruders after 10s *)
let remove_old_intruders = fun () ->
  Hashtbl.iter
    (fun id i ->
      if (Unix.gettimeofday () -. i.last_update) > 10.0 then
        remove_intruder id
    ) intruders

