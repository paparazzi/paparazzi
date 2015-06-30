(*
 * Displaying and editing a flight plan on a MapCanvas
 *
 * Copyright (C) 2004-2006 ENAC, Pascal Brisset, Antoine Drouin
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

(** [flight_plan geomap color dtd_tile xml] *)
class flight_plan :
  ?format_attribs:((string * string) list -> string) ->
  ?editable:bool ->
  show_moved:bool ->
  MapCanvas.widget ->
  string ->
  string ->
  Xml.xml ->
  object
    method add_waypoint : Latlong.geographic -> MapWaypoints.waypoint
    method destroy : unit -> unit
    method georef : Latlong.geographic
    method hide : unit -> unit
    method index : XmlEdit.node -> int * MapWaypoints.waypoint
    method get_wp : int -> MapWaypoints.waypoint (** May raise Not_found *)
    method show : unit -> unit
    method window : GObj.widget
    method waypoints : XmlEdit.node list
    method xml : Xml.xml
    method insert_path : (MapWaypoints.waypoint * float) list -> unit
    method highlight_stage : int -> int -> unit
    method connect_activated : (XmlEdit.node->unit) -> unit
    method update_sectors : string -> unit
  end

(** Extracts [lat0] and [Lon0] attributes *)
val georef_of_xml : Xml.xml -> Latlong.geographic
