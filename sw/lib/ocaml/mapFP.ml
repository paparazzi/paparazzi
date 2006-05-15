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

let iter_stages = fun f xml_tree ->
  let xml_blocks = XmlEdit.child (XmlEdit.root xml_tree) "blocks" in
  let rec loop = fun n ->
    f n;
    List.iter loop (XmlEdit.children n) in
  loop xml_blocks

let try_replace_attrib = fun node tag prev_v v ->
  try
    if XmlEdit.attrib node tag = prev_v then
      XmlEdit.set_attrib node (tag, v)
  with
    Not_found -> ()

(** Update all the references to waypoint names (attribute "wp") *)
let update_wp_refs previous_name xml_tree = function
    XmlEdit.Deleted -> () (** FIXME *)
  | XmlEdit.New_child _ -> ()
  | XmlEdit.Modified attribs ->
      try
	let new_name = assoc_nocase "name" attribs in
	let update = fun node ->
	  try_replace_attrib node "wp" !previous_name new_name;
	  try_replace_attrib node "from" !previous_name new_name in
	iter_stages update xml_tree;
	previous_name := new_name
      with
	Not_found -> ()

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

let new_wp = fun xml_tree waypoints utm_ref ?(alt = 0.) node ->
  let float_attrib = fun a -> float_of_string (XmlEdit.attrib node a) in
  let x = (float_attrib "x") and y = (float_attrib "y") in
  let wgs84 = Latlong.of_utm WGS84 (utm_add utm_ref (x, y)) in
  let alt = try float_attrib "alt" with _ -> alt in
  let name = XmlEdit.attrib node "name" in
  let wp = MapWaypoints.waypoint waypoints ~name ~alt wgs84 in
  XmlEdit.connect node (update_wp utm_ref wp);
  XmlEdit.connect node (update_wp_refs (ref name) xml_tree); 
  wp#connect (fun () -> update_xml xml_tree utm_ref wp);
  wp

let gensym =
  let x = ref 0 in
  fun p -> incr x; Printf.sprintf "%s%d" p !x

let rec new_gensym = fun p l ->
  let s = gensym p in
  if List.mem s l then new_gensym p l else s

let georef_of_xml = fun xml ->
  let lat0 = float_attr xml "lat0"
  and lon0 = float_attr xml "lon0" in
  {posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 }

 
class flight_plan = fun ?edit geomap color fp_dtd xml ->
  (** Xml Editor *)
  let xml_tree_view, xml_window = XmlEdit.create ?edit (Dtd.parse_file fp_dtd) xml in
  let xml_root = XmlEdit.root xml_tree_view in
  let xml_wpts = XmlEdit.child xml_root "waypoints" in

  (** Geographic ref *)
  let alt = float_attr xml "alt" in
  let ref_wgs84 = georef_of_xml xml in
  let utm0 = utm_of WGS84 ref_wgs84 in

  (** The graphical waypoints *)
  let wpts_group = new MapWaypoints.group ~color ~editable:true geomap in

  let yaws = Hashtbl.create 5 in (* Yes Another Waypoints Store *)
  let create_wp =
    let i = ref 1 in
    fun node ->
      let w = new_wp xml_tree_view wpts_group utm0 ~alt node in
      Hashtbl.add yaws (XmlEdit.attrib node "name") (!i, w);
      incr i;
      w in

  let _ = List.iter
      (fun wp ->
	let w = create_wp wp in
	let name = XmlEdit.attrib wp "name" in
	if name = "HOME" then begin
	  let c = ref (GnoCanvas.ellipse geomap#canvas#root) in
	  let update = fun _ ->
	    try
	      let max_dist_from_home = float_of_string (XmlEdit.attrib xml_root "MAX_DIST_FROM_HOME") in
	      !c#destroy ();
	      c :=  geomap#circle ~width:5 ~color w#pos max_dist_from_home
	    with _ -> () in
	  update ();
	  w#connect update;
	  XmlEdit.connect wp update;
	  XmlEdit.connect xml_root update
	end)
      (XmlEdit.children xml_wpts) in

  let _ =
    XmlEdit.expand_node xml_tree_view xml_root;
    let blocks = XmlEdit.child xml_root "blocks" in
    XmlEdit.expand_node xml_tree_view blocks in
  
  object
    method georef = ref_wgs84
    method window = xml_window
    method destroy () = 
      wpts_group#group#destroy ();
      xml_window#destroy ()
    method show () = wpts_group#group#show ()
    method hide () = wpts_group#group#hide ()
    method index wp = Hashtbl.find yaws (XmlEdit.attrib wp "name")
    method waypoints = XmlEdit.children (waypoints_node xml_tree_view)
    method xml = XmlEdit.xml_of_view xml_tree_view
    method highlight_stage = fun block_no stage_no ->
      let block_no = string_of_int block_no in
      let stage_no = string_of_int stage_no in
      let blocks = XmlEdit.child xml_root "blocks" in
      List.iter
	(fun b ->
	  if XmlEdit.attrib b "no" = block_no then begin
	    XmlEdit.set_background ~all:true b "#00c000";
	    let rec f = fun s ->
	      try
		if XmlEdit.attrib s "no" = stage_no then
		  XmlEdit.set_background s "green"
		else
		  List.iter f (XmlEdit.children s)
	      with
		Not_found -> () in
	    List.iter f (XmlEdit.children b)
	  end else
	    XmlEdit.set_background ~all:true b "white")
	(XmlEdit.children blocks)
	  
    method add_waypoint (geo:geographic) =
      let wpt_names = List.map (fun n -> XmlEdit.attrib n "name") (XmlEdit.children xml_wpts) in
      let name = new_gensym "wp" wpt_names in
      let utm = utm_of WGS84 geo in
      let (dx, dy) = utm_sub utm utm0 in
      let node = XmlEdit.add_child xml_wpts "waypoint" ["x",sof dx;"y",sof dy;"name",name] in
      create_wp node

    method insert_path = fun path ->
      let xml_block =
	try XmlEdit.parent (XmlEdit.selection xml_tree_view) "block" with
	  _ ->
	    let xml_blocks = XmlEdit.child xml_root "blocks" in
	    XmlEdit.child xml_blocks "block" in
      let path_node = XmlEdit.add_child xml_block "path" ["radius", "42."] in
      List.iter
	(fun ((wp:MapWaypoints.waypoint), r) ->
	  let n = XmlEdit.add_child path_node "path_point" ["wp", wp#name; "radius", sof r] in
	  ()
	)
	path

    initializer (
  (** Create a graphic waypoint when it is created from the xml editor *)
      XmlEdit.connect xml_wpts (function XmlEdit.New_child node -> ignore (create_wp  node) | _ -> ())
     )
  end
