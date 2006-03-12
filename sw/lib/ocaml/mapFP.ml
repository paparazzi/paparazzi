(*
 * $Id$
 *
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

open Latlong

let sof = string_of_float
let float_attr = fun xml a -> float_of_string (ExtXml.attrib xml a)
let rec assoc_nocase at = function
    [] -> raise Not_found
  | (a, v)::avs ->
      if String.uppercase at = String.uppercase a then v else assoc_nocase at avs

(** Connect a change in the XML editor to the graphical rep *)
let update_wp utm_ref wp = function
    XmlEdit.Deleted -> wp#delete
  | XmlEdit.New_child _ -> failwith "update_wp"
  | XmlEdit.Modified attribs ->
      try
	let float_attrib = fun a -> float_of_string (assoc_nocase a attribs) in
	let x = (float_attrib "x") and y = (float_attrib "y") in
	let wgs84 = Latlong.of_utm WGS84 (utm_add utm_ref (x, y)) in
	wp#set wgs84;
	wp#set_name (assoc_nocase "name" attribs)
      with
	_ -> ()

let waypoints_node = fun xml_tree ->
  let xml_root = XmlEdit.root xml_tree in
  XmlEdit.child xml_root "waypoints"

(** Connect a change from the graphical rep to the xml tree *)
let update_xml = fun xml_tree utm0 wp ->
  let xml_wpts = XmlEdit.children (waypoints_node xml_tree) in
  let node = List.find (fun w -> XmlEdit.attrib w "name" = wp#name) xml_wpts in
  let utm = utm_of WGS84 (wp#pos) in
  let (dx, dy) = utm_sub utm utm0 in
  XmlEdit.set_attribs node ["name",wp#name; "x",sof dx; "y",sof dy; "alt", sof wp#alt]

let new_wp = fun xml_tree waypoints utm_ref node ->
  let float_attrib = 
    fun a -> try float_of_string (XmlEdit.attrib node a) with _ -> 0. in
  let x = (float_attrib "x") and y = (float_attrib "y") in
  let wgs84 = Latlong.of_utm WGS84 (utm_add utm_ref (x, y)) in
  let alt = float_attrib "alt" in
  let name = XmlEdit.attrib node "name" in
  let wp = MapWaypoints.waypoint waypoints ~name ~alt wgs84 in
  XmlEdit.connect node (update_wp utm_ref wp);
  wp#connect (fun () -> update_xml xml_tree utm_ref wp);
  wp

let gensym =
  let x = ref 0 in
  fun p -> incr x; Printf.sprintf "%s%d" p !x

 
class flight_plan = fun geomap color fp_dtd xml ->
  (** Xml Editor *)
  let xml_tree_view, xml_window = XmlEdit.create (Dtd.parse_file fp_dtd) xml in
  let xml_root = XmlEdit.root xml_tree_view in
  let xml_wpts = XmlEdit.child xml_root "waypoints" in

  (** Geographic ref *)
  let lat0 = float_attr xml "lat0"
  and lon0 = float_attr xml "lon0"
  and alt0 = float_attr xml "alt" in
  let ref_wgs84 = {posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 } in
  let utm0 = utm_of WGS84 ref_wgs84 in

  (** The graphical waypoints *)
  let wpts_group = new MapWaypoints.group ~color ~editable:true geomap in

  let yaws = Hashtbl.create 5 in (* Yes Another Waypoints Store *)
  let create_wp =
    let i = ref 0 in
    fun node ->
      let w = new_wp xml_tree_view wpts_group utm0 node in
      Hashtbl.add yaws (XmlEdit.attrib node "name") (!i, w);
      w in

  let max_dist_from_home = float_attr xml "MAX_DIST_FROM_HOME" in

  let _ = List.iter
      (fun wp ->
	let w = create_wp wp in
	let name = XmlEdit.attrib wp "name" in
	if name = "HOME" then
	  ignore (geomap#circle ~color w#pos max_dist_from_home)) 
      (XmlEdit.children xml_wpts) in
  
  object
    val mutable max_dist_from_home = max_dist_from_home
    method georef = ref_wgs84
    method show_xml () = xml_window#show ()
    method destroy () = 
      wpts_group#group#destroy ();
      xml_window#destroy ()
    method show () = wpts_group#group#show ()
    method hide () = wpts_group#group#hide ()
    method index wp = Hashtbl.find yaws (XmlEdit.attrib wp "name")
    method waypoints = XmlEdit.children (waypoints_node xml_tree_view)
    method xml = XmlEdit.xml_of_view xml_tree_view
    method add_waypoint (geo:geographic) =
      let name = gensym "wp" in
      let utm = utm_of WGS84 geo in
      let (dx, dy) = utm_sub utm utm0 in
      let node = XmlEdit.add_child xml_wpts "waypoint" ["x",sof dx;"y",sof dy;"name",name] in
      create_wp node

    initializer (
  (** Create a graphic waypoint when it is created from the xml editor *)
      XmlEdit.connect xml_wpts (function XmlEdit.New_child node -> ignore (create_wp  node) | _ -> ())
     )
  end
