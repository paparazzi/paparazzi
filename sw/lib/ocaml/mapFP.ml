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

open Printf
open Latlong
let (//) = Filename.concat

let sof = string_of_float
let sof1 = fun x -> sprintf "%.1f" x
let sof6 = fun x -> sprintf "%.6f" x
let float_attr = fun xml a -> float_of_string (ExtXml.attrib xml a)
let rec assoc_nocase at = function
[] -> raise Not_found
  | (a, v)::avs ->
    if String.uppercase at = String.uppercase a then v else assoc_nocase at avs

(** Returns the WGS84 coordinates of a waypoint, either from its relative x and
    y coordinates or from its lat and long *)
let geo_of_xml = fun utm_ref get_attrib ->
  try
    let x = get_attrib "x"
    and y = get_attrib "y" in
    Latlong.of_utm WGS84 (utm_add utm_ref (x, y))
  with
      Not_found | Xml.No_attribute _ ->
        try
          let lat = get_attrib "lat"
          and lon = get_attrib "lon" in
          make_geo_deg lat lon
        with
            Not_found -> failwith (sprintf "x and y or lat and lon attributes expected in waypoint")


(** Connect a change in the XML editor to the graphical rep *)
let update_wp utm_ref (wp:MapWaypoints.waypoint) = function
XmlEdit.Deleted -> wp#delete ()
  | XmlEdit.New_child _ -> failwith "update_wp"
  | XmlEdit.Modified attribs ->
    try
      let float_attrib = fun a -> float_of_string (assoc_nocase a attribs) in

      let wgs84 = geo_of_xml utm_ref float_attrib in

      wp#geomap#edit_georef_name wp#name (assoc_nocase "name" attribs);
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
(** FIXME This function is disabled for now since it is making
 * a huge mess when reordering the waypoints *)
(*let update_wp_refs previous_name xml_tree = function
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
*)

let waypoints_node = fun xml_tree ->
  let xml_root = XmlEdit.root xml_tree in
  XmlEdit.child xml_root "waypoints"

let is_relative_waypoint = fun node ->
  try
    ignore (XmlEdit.attrib node "x");
    ignore (XmlEdit.attrib node "y");
    true
  with
      Not_found -> false


let absolute_coords = fun wp ->
  let wgs84 = wp#pos in
  [ "lat", sof6 ((Rad>>Deg) wgs84.posn_lat);
    "lon", sof6 ((Rad>>Deg) wgs84.posn_long) ]


(** Connect a change from the graphical rep to the xml tree *)
let update_xml = fun xml_tree utm0 wp id ->
  let xml_wpts = XmlEdit.children (waypoints_node xml_tree) in
  let node = List.find (fun w -> XmlEdit.id w = id) xml_wpts in
  let default_alt = float_of_string (XmlEdit.attrib (XmlEdit.root xml_tree) "alt") in
  if wp#deleted then begin
    XmlEdit.delete node
  end else
    let coords =
      if is_relative_waypoint node then
        let utm = utm_of WGS84 wp#pos in
        try
          let (dx, dy) = utm_sub utm utm0 in
          ["x",sof1 dx; "y",sof1 dy]
        with
            _ ->
              prerr_endline "MapFP.update_xml: waypoint too far from ref; using absolute geodetic coordinates";
              absolute_coords wp
      else (* Absolute waypoint: use lat and lon attributes *)
        absolute_coords wp in

    let alt_attrib =
      if abs_float (wp#alt -. default_alt) < 1. then [] else ["alt", sof1 wp#alt] in
    XmlEdit.set_attribs node (("name",wp#name) :: alt_attrib @ coords)




let new_wp = fun ?(editable = false) (geomap:MapCanvas.widget) xml_tree waypoints utm_ref ?(alt = 0.) node ->
  let float_attrib = fun a -> float_of_string (XmlEdit.attrib node a) in

  let wgs84 = geo_of_xml utm_ref float_attrib in

  let alt = try float_attrib "alt" with _ -> alt in
  let name = XmlEdit.attrib node "name" in
  let show = editable || name.[0] <> '_' in
  let wp = MapWaypoints.waypoint ~show waypoints ~name ~alt wgs84 in
  geomap#register_to_fit (wp:>MapCanvas.geographic);
  XmlEdit.connect node (update_wp utm_ref wp);
  (*XmlEdit.connect node (update_wp_refs (ref name) xml_tree);*) (* FIXME broken functionality *)
  let id = XmlEdit.id node in
  if editable then
    wp#connect (fun () -> update_xml xml_tree utm_ref wp id);
  wp

let gensym =
  let x = ref 0 in
  fun p -> incr x; Printf.sprintf "%s%d" p !x

let rec new_gensym = fun p l ->
  let s = gensym p in
  if List.mem s l then new_gensym p l else s

let georef_of_xml = fun xml ->
  let lat0 = Latlong.deg_of_string (ExtXml.attrib xml "lat0")
  and lon0 = Latlong.deg_of_string (ExtXml.attrib xml "lon0") in
  { posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 }


let display_lines = fun ?group color (geomap:MapCanvas.widget) points ->
  let n = Array.length points in
  let l = ref [] in
  for i = 0 to n - 1 do
    l := !l @ [(geomap#segment ?group ~width:3 ~fill_color:color points.(i) points.((i+1)mod n))]
  done;
  !l

let space_regexp = Str.regexp " "
let comma_regexp = Str.regexp ","
let wgs84_of_kml_point = fun s ->
  match Str.split comma_regexp s with
      [long; lat; altitude] ->
        let lat = float_of_string lat
        and long = float_of_string long in
        {posn_lat = (Deg>>Rad) lat; posn_long = (Deg>>Rad) long}
    | _ -> failwith (Printf.sprintf "wgs84_of_kml_point: %s" s)


(** It should be somewhere else ! *)
let display_kml = fun ?group color geomap xml ->
  try
    let document = ExtXml.child xml "Document" in
    let rec loop = fun child ->
      match String.lowercase (Xml.tag child) with
          "placemark" ->
            let linestring = ExtXml.child child "LineString" in
            let coordinates = ExtXml.child linestring "coordinates" in
            begin
              match Xml.children coordinates with
                  [Xml.PCData text] ->
                    let points = Str.split space_regexp text in
                    let points = List.map wgs84_of_kml_point points in
                    ignore(display_lines ?group color geomap (Array.of_list points))
                | _ -> failwith "coordinates expected"
            end

        | "folder" ->
          List.iter loop (Xml.children child)
        | _ -> () in
    List.iter loop (Xml.children document)
  with Xml.Not_element xml -> failwith (Xml.to_string xml)




class flight_plan = fun ?format_attribs ?editable ~show_moved geomap color fp_dtd xml ->
  (** Xml Editor *)
  let xml_tree_view, xml_window = XmlEdit.create ?format_attribs ?editable (Dtd.parse_file fp_dtd) xml in
  let xml_root = XmlEdit.root xml_tree_view in
  let xml_wpts = XmlEdit.child xml_root "waypoints" in

  (** Geographic ref *)
  let alt = float_attr xml "alt" in
  let ref_wgs84 = georef_of_xml xml in
  let utm0 = utm_of WGS84 ref_wgs84 in

  (** The graphical waypoints *)
  let wpts_group = new MapWaypoints.group ~show_moved ~color ?editable geomap in

  let array_of_waypoints = ref (Array.make 13 None) in
  let add_wp_to_array = fun index w ->
    let n = Array.length !array_of_waypoints in
    if index >= n then begin
      let new_array = Array.make (n*2) None in
      Array.blit !array_of_waypoints 0 new_array 0 n;
      array_of_waypoints := new_array
    end;
    !array_of_waypoints.(index) <- Some w in

  let yaws = Hashtbl.create 5 in (* Yes Another Waypoints Store *)
  let create_wp =
    let i = ref 1 in
    fun node ->
      let w = new_wp ?editable geomap xml_tree_view wpts_group utm0 ~alt node in
      Hashtbl.add yaws (XmlEdit.attrib node "name") (!i, w);
      add_wp_to_array !i w;
      incr i;
      w in

  (* The sectors *)
  (* Parse sectors and store dynamic ones *)
  let sectors =
    let waypoints = ExtXml.child xml "waypoints" in
    try
      List.fold_left (fun l x ->
        match String.lowercase (Xml.tag x) with
            "kml" ->
              let file = ExtXml.attrib x "file" in
              display_kml ~group:wpts_group#group color geomap (ExtXml.parse_file (Env.flight_plans_path // file));
              l
          | "sector" ->
            let wgs84 = fun wp_name ->
              let wp_name = Xml.attrib wp_name "name" in
              let select = fun wp -> Xml.attrib wp "name" = wp_name in
              let wp = ExtXml.child waypoints ~select "waypoint" in
              let float_attr = fun xml a -> float_of_string (Xml.attrib xml a) in
              geo_of_xml utm0 (float_attr wp) in
            let points = List.map wgs84 (Xml.children x) in
            let points = Array.of_list points in
            let color_sector = ExtXml.attrib_or_default x "color" color in
            let segments = display_lines ~group:wpts_group#group color_sector geomap points in
            let wp_names = List.map (fun wp -> Xml.attrib wp "name") (Xml.children x) in
            if ExtXml.attrib_or_default x "type" "" = "dynamic" then
              [(wp_names, segments, color_sector)] @ l
            else
              l
          | _ -> failwith "Unknown sectors child")
      [] (Xml.children (ExtXml.child xml "sectors"))
    with Not_found -> [] in

  (* The waypoints *)
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
            c :=  geomap#circle ~group:wpts_group#group ~width:5 ~color w#pos max_dist_from_home
          with _ -> () in
        update ();
        w#connect update;
        XmlEdit.connect wp update;
        XmlEdit.connect xml_root update
      end)
    (XmlEdit.children xml_wpts) in

  (** Expands the blocks *)
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
  method get_wp = fun i ->
    if i >= Array.length !array_of_waypoints then
      raise Not_found;
    match !array_of_waypoints.(i) with
        None -> raise Not_found
      | Some w -> w
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
                XmlEdit.set_background s "#00ff00"
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
        let _n = XmlEdit.add_child path_node "path_point" ["wp", wp#name; "radius", sof r] in
        ()
      )
      path

  method connect_activated = fun cb -> XmlEdit.connect_activated xml_tree_view cb

  method update_sectors = fun wp_name ->
    List.iter (fun (wps_name, segments, color) ->
      let wp_in_sector = List.exists (fun name -> name = wp_name) wps_name in
      if wp_in_sector then begin
        (* Build WP array *)
        let points = List.map (fun n -> let (_, w) = Hashtbl.find yaws n in w#pos) wps_name in
        let points = Array.of_list points in
        let segments = Array.of_list segments in
        let n = Array.length points in
        (* Update segments *)
        for i = 0 to n - 1 do
          let (x1, y1) = geomap#world_of points.(i)
          and (x2, y2) = geomap#world_of (points.((i+1)mod n)) in
          segments.(i)#set [`POINTS [|x1; y1; x2;  y2 |]]
        done
      end
    ) sectors

  initializer (
      (** Create a graphic waypoint when it is created from the xml editor *)
    XmlEdit.connect xml_wpts (function XmlEdit.New_child node -> ignore (create_wp  node) | _ -> ())
  )
end
