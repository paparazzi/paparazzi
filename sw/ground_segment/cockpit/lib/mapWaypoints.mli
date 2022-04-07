(*
 * Waypoints objects
 *
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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

class group :
  ?color:string ->
  ?editable:bool ->
  ?show_moved:bool ->
  MapCanvas.widget ->
  object
    method color : string
    method editable : bool
    method show_moved : bool
    method geomap : MapCanvas.widget
    method group : GnoCanvas.group
   end

class waypoint :
  ?show:bool ->
  group ->
  string ->
  ?alt:float ->
  Latlong.geographic ->
  object
    method alt : float
    method geomap : MapCanvas.widget
    method delete : unit -> unit
    method edit : unit
    method pos : Latlong.geographic
    method event : GnoCanvas.item_event -> bool
    method item : GnoCanvas.rect
    method label : ContrastLabel.widget
    method move : float -> float -> unit
    method name : string
    method set : ?altitude:float -> ?update:bool -> Latlong.geographic -> unit
    method set_ground_alt : float -> unit
    method set_name : string -> unit
    method xy : float * float
    method zoom : float -> unit
    method moved : bool
    method reset_moved : unit -> unit
    method deleted : bool
    method connect : (unit -> unit) -> unit
    method set_commit_callback : (unit -> unit) -> unit
  end


val waypoint : ?show:bool -> group -> ?name:string -> ?alt:float -> Latlong.geographic -> waypoint
