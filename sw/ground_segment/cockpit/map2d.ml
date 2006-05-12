(*
* $Id$
*
* Multi aircrafts map display and flight plan editor
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

module G2D = Geometry_2d
module G = MapCanvas
open Printf
open Latlong

let float_attr = fun xml a -> float_of_string (ExtXml.attrib xml a)
let int_attr = fun xml a -> int_of_string (ExtXml.attrib xml a)

let soi = string_of_int
let sof = string_of_float

let bat_max = 13.
let bat_min = 8.

let home = Env.paparazzi_home
let (//) = Filename.concat
let default_path_srtm = home // "data" // "srtm"
let default_path_maps = home // "data" // "maps" // ""
let path_fps = home // "conf" // "flight_plans" // ""
let fp_dtd = path_fps // "flight_plan.dtd"
let var_maps_path = home // "var" // "maps"
let _ = 
  ignore (Sys.command (sprintf "mkdir -p %s" var_maps_path))
let fp_example = path_fps // "example.xml"

let speech = ref false

let say = fun s ->
  if !speech then
    ignore (Sys.command (sprintf "spd-say '%s'&" s))

(** window for the strip panel *)
let strip_panel = GWindow.window ~title: "Strip Panel" ~width: 330 ~height:300 ()
let strip_scrolled = GBin.scrolled_window ~hpolicy: `AUTOMATIC ~vpolicy: `AUTOMATIC ~packing: strip_panel#add ()
let strip_table = GPack.table ~rows: 1 ~columns: 1 ~row_spacings: 5 ~packing: (strip_scrolled#add_with_viewport) ()

(** Dummy flight plan (for map calibration) *)
let dummy_fp = fun latlong ->
  Xml.Element("flight_plan",
	      ["lat0", sof ((Rad>>Deg)latlong.posn_lat);
	       "lon0", sof ((Rad>>Deg)latlong.posn_long);
	       "alt", "42.";
	       "MAX_DIST_FROM_HOME", "1000."],
	      [Xml.Element("waypoints", [],[])])


let set_georef_if_none = fun geomap wgs84 ->
  match geomap#georef with
    None -> geomap#set_georef wgs84
  | Some _ -> ()


(** Display a calibrated (XML) map *)
let display_map = fun (geomap:G.widget) xml_map ->
  try
    let dir = Filename.dirname xml_map in
    let xml_map = Xml.parse_file xml_map in
    let image = dir // ExtXml.attrib xml_map "file" in
    let map_projection = Xml.attrib xml_map "projection" in
    let current_projection = geomap#projection in
    if map_projection <> current_projection then
      GToolbox.message_box "Warning" (sprintf "You are loading a map in %s projection while the display use %s" map_projection current_projection);

    let pix_ref = fun p ->
      truncate (float_attr p "x"), truncate (float_attr p "y") in
    let geo_ref = fun p ->
      try Latlong.of_string (Xml.attrib p "geo") with
	_ -> (* Compatibility with the old UTM format *)
	  let utm_x = float_attr p "utm_x"
	  and utm_y = float_attr p "utm_y" in
	  let utm_zone = int_attr xml_map "utm_zone" in
	  let utm = {utm_x = utm_x;  utm_y = utm_y; utm_zone = utm_zone } in
	  Latlong.of_utm WGS84 utm in
    
    match Xml.children xml_map with
      p1::p2::_ ->
	let x1y1 = pix_ref p1
	and x2y2 = pix_ref p2
	and geo1 = geo_ref p1
	and geo2 = geo_ref p2 in
	
	(* Take this point as a reference for the display if none currently *)
	set_georef_if_none geomap geo1;
	
	ignore (geomap#display_pixbuf ((x1y1),geo1) ((x2y2),geo2) (GdkPixbuf.from_file image));
	geomap#center geo1
    | _ -> failwith (sprintf "display_map: two ref points required")
  with
    Xml.File_not_found f ->
      GToolbox.message_box "Error" (sprintf "File does not exist: %s" f)
  | ExtXml.Error s ->
      GToolbox.message_box "Error" (sprintf "Error in XML file: %s" s)
	


let load_map = fun (geomap:G.widget) () ->
  match GToolbox.select_file ~title:"Open Map" ~filename:(default_path_maps^"*.xml") () with
    None -> ()
  | Some f -> display_map geomap f


(** Load a mission. Returns the XML window *)
let load_mission = fun color geomap xml ->
  set_georef_if_none geomap (MapFP.georef_of_xml xml);
  new MapFP.flight_plan geomap color fp_dtd xml


(** Save the given pixbuf calibrated with NW and SE corners *)
let save_map = fun geomap ?(projection=geomap#projection) pixbuf nw se ->
  match GToolbox.select_file ~filename:(default_path_maps//".xml") ~title:"Save region map" () with
    None -> ()
  | Some xml_file  ->
      let jpg = Filename.chop_extension xml_file ^ ".png" in
      GdkPixbuf.save jpg "png" pixbuf;
      let point = fun (x,y) wgs84 ->
	Xml.Element ("point", ["x",soi x;"y",soi y;"geo", Latlong.string_of wgs84], []) in
      let width = GdkPixbuf.get_width pixbuf
      and height = GdkPixbuf.get_height pixbuf in
      let points = [point (0, 0) nw; point (width, height) se] in
      let xml = Xml.Element ("map", 
			     ["file", Filename.basename jpg;
			      "projection", projection],
			     points) in
      let f = open_out xml_file in
      Printf.fprintf f "%s\n" (Xml.to_string_fmt xml);
      close_out f



(****** Creates a calibrated map from the bitmap (selected region) ***********)
let map_from_region = fun (geomap:G.widget) () ->
  match geomap#region with
    None -> GToolbox.message_box "Error" "Select a region (shift-left drag)"
  | Some ((xw1,yw1), (xw2,yw2)) ->
      let xw1, xw2 = min xw1 xw2, max xw1 xw2
      and yw1, yw2 = min yw1 yw2, max yw1 yw2 in
      let (xc1, yc1) = geomap#canvas#w2c xw1 yw1
      and (xc2, yc2) = geomap#canvas#w2c xw2 yw2 in
      let width = xc2-xc1 and height = yc2-yc1 in
      let dest = GdkPixbuf.create width height () in
      let (x0, y0) = geomap#canvas#get_scroll_offsets in
      let src_x = xc1 - x0 and src_y = yc1 - y0 in
      GdkPixbuf.get_from_drawable ~dest ~width ~height ~src_x ~src_y
	geomap#canvas#misc#window;
      let nw = geomap#of_world (xw1,yw1)
      and se = geomap#of_world (xw2,yw2) in
      save_map geomap dest nw se
(************ Strip handling ***********************************************)
module Strip = struct
  type t = {
      ac_id: string;
      gauge: GRange.progress_bar ;
      labels: (string * GMisc.label) list
    }

  let panel = Hashtbl.create 3

  let pad = 4
  let i2s = string_of_int
  let labels_name =  [| 
    [| "AP" ; "alt" ; "->" |]; [| "RC"; "climb"; "/" |]; [| "GPS"; "speed"; "" |];
    [| "settings" ; "throttle"; "" |]; [| ""; "wind"; "dir" |];
  |]

  let labels_print = [| 
    [| "AP" ; "alt" ; "->" |]; [| "RC"; "climb"; "->" |]; [| "GPS"; "speed"; "" |];
    [| "settings" ; "throttle"; "" |]; [| ""; "wind"; "dir" |];
  |]
  let gen_int = let i = ref (-1) in fun () -> incr i; !i


  (** add a strip to the panel *)
  let add (widget: GPack.table) ac_id config color =
    (* number of the strip *)
    let strip_number = gen_int () in
    if strip_number > 1 then widget#set_rows strip_number;

    let strip_labels = ref  [] in
    let add_label = fun name value -> 
      strip_labels := (name, value) :: !strip_labels in

    let ac_name = Pprz.string_assoc "ac_name" config in

    (* frame of the strip *)
    let frame = GBin.frame ~shadow_type: `IN ~packing: (widget#attach ~top: (strip_number) ~left: 0) () in
    let strip = GPack.table ~rows: 2 ~columns: 2 ~col_spacings: 10 ~packing: frame#add () in
    add_label "name" (GMisc.label ~text: (ac_name) ~packing: (strip#attach ~top: 0 ~left: 0) ());
    
    let plane_color = GBin.event_box ~width:10 ~height:10 ~packing:(strip#attach ~top:0 ~left: 1) () in
    plane_color#coerce#misc#modify_bg [`NORMAL, `NAME color];

    (* battery and flight time *)
    let bat_table = GPack.table ~rows: 2 ~columns: 1 ~packing: (strip#attach ~top:1 ~left:0) () in
    let pb = GRange.progress_bar ~orientation: `BOTTOM_TO_TOP ~packing: (bat_table#attach ~top:0 ~left:0) () in
    pb#coerce#misc#modify_fg [`PRELIGHT, `NAME "green"];
    pb#coerce#misc#modify_font_by_name "sans 18";
    let ft = GMisc.label ~text: "00:00:00" ~packing: (bat_table#attach ~top:1 ~left:0) () in
    ft#set_width_chars 8;
    add_label ("flight_time_value") ft;
 
    let left_box = GPack.table ~rows: 5 ~columns: 6 ~col_spacings: 5 
      ~packing: (strip#attach ~top: 1 ~left: 1) () in

    Array.iteri 
      (fun i a ->
	Array.iteri
	  (fun j s ->
	    add_label s (GMisc.label ~text: labels_print.(i).(j) ~justify: `LEFT ~packing: (left_box#attach ~top: i ~left: (2*j)) ());
	    let lvalue = (GMisc.label ~text: "" ~justify: `RIGHT ~packing: (left_box#attach ~top: i ~left: (2*j+1)) ()) in
	    lvalue#set_width_chars 6;
	    add_label (s^"_value") lvalue;
	  ) a
      ) labels_name;
    Hashtbl.add panel ac_id {ac_id = ac_id; gauge=pb ; labels= (!strip_labels)}

  (** find an aircraft in the list of aircraft *)
  let find ac_id = Hashtbl.find panel ac_id

  (** set a label *)
  let set_label strip name value = 
    let l = List.assoc (name^"_value") strip.labels in
    l#set_label value

  (** set the battery *)
  let set_bat strip value =
    strip.gauge#set_text (sof value);
    let f = (value -. bat_min) /. (bat_max -. bat_min) in
    let f = max 0. (min 1. f) in
    strip.gauge#set_fraction f

end



(************ Google Maps handling *****************************************)
module GM = struct
  let active_http = fun x -> 
    Gm.no_http := not x

  let auto = ref false
  let active_auto = fun x -> 
    auto := x


  let fill_tiles = fun geomap -> 
    ignore (Thread.create MapGoogle.fill_window geomap)
      
  let update = fun geomap ->
    if !auto then fill_tiles geomap


(** Creates a calibrated map from the Google tiles (selected region) *)
  let map_from_tiles = fun (geomap:G.widget) () ->
    match geomap#region with
      None -> GToolbox.message_box "Error" "Select a region (shift-left drag)"
    | Some ((xw1,yw1), (xw2,yw2)) ->
	let geo1 = geomap#of_world (xw1,yw1)
	and geo2 = geomap#of_world (xw2,yw2) in
	let sw = { posn_lat = min geo1.posn_lat geo2.posn_lat;
		   posn_long = min geo1.posn_long geo2.posn_long }
	and ne = { posn_lat = max geo1.posn_lat geo2.posn_lat;
		   posn_long = max geo1.posn_long geo2.posn_long } in
	let pix = MapGoogle.pixbuf sw ne in
	let nw = { posn_lat = ne.posn_lat; posn_long = sw.posn_long }
	and se = { posn_lat = sw.posn_lat; posn_long = ne.posn_long } in
	save_map geomap ~projection:"Mercator" pix nw se
end (* GM module *)
    

(***************** Editing ONE (single) flight plan **************************)
module Edit = struct
  let current_fp = ref None

      (** Wrapper checking there is currently no flight plan loaded *)
  let if_none = fun f ->
    match !current_fp with
      Some _ ->
	GToolbox.message_box "Error" "Only one editable flight plan at a time"
    | None ->
	f ()


  let close_fp = fun () ->
    match !current_fp with
      None -> () (* Nothing to close *)
    | Some (fp, _filename) ->
	fp#destroy ();
	current_fp := None
	    
  let load_xml_fp = fun geomap accel_group ?(xml_file=path_fps) xml ->
    set_georef_if_none geomap (MapFP.georef_of_xml xml);
    let fp = new MapFP.flight_plan geomap "red" fp_dtd xml in
    fp#window#show ();
    fp#window#add_accel_group accel_group;
    ignore (fp#window#connect#destroy ~callback:close_fp);
    current_fp := Some (fp,xml_file);
    fp

  let labelled_entry = fun ?width_chars text value h ->
    let _ = GMisc.label ~text ~packing:h#add () in
    GEdit.entry ?width_chars ~text:value ~packing:h#add ()

  let new_fp = fun geomap accel_group () ->
    if_none (fun () ->
      let dialog = GWindow.window ~border_width:10 ~title:"New flight plan" () in
      let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
      let h = GPack.hbox ~packing:dvbx#pack () in
      let default_latlong =
	match geomap#georef with
	  None -> "WGS84 37.21098 -113.45678"
	| Some geo -> Latlong.string_of geo in
      let latlong  = labelled_entry ~width_chars:25 "Geographic Reference" default_latlong h in
      let alt0 = labelled_entry ~width_chars:4 "Ground Alt" "380" h in
      let h = GPack.hbox ~packing:dvbx#pack () in
      let alt = labelled_entry ~width_chars:4 "Default Alt" "430" h in
      let qfu = labelled_entry ~width_chars:4 "QFU" "270" h in
      let mdfh = labelled_entry ~width_chars:4 "Max distance from HOME" "500" h in
      
      let h = GPack.hbox ~packing:dvbx#pack () in
      let name  = labelled_entry "Name" "Test flight" h in
      
      let h = GPack.hbox ~packing:dvbx#pack () in
      let cancel = GButton.button ~stock:`CANCEL ~packing: h#add () in 
      ignore(cancel#connect#clicked ~callback:dialog#destroy);

      let createfp = GButton.button ~stock:`OK ~packing: h#add () in
      createfp#grab_default ();
      ignore(createfp#connect#clicked ~callback:
	       begin fun _ ->
		 let xml = Xml.parse_file fp_example in
		 let s = ExtXml.subst_attrib in
		 let wgs84 = Latlong.of_string latlong#text in
		 let xml = s "lat0" (deg_string_of_rad wgs84.posn_lat) xml in
		 let xml = s "lon0" (deg_string_of_rad wgs84.posn_long) xml in
		 let xml = s "ground_alt" alt0#text xml in
		 let xml = s "qfu" qfu#text xml in
		 let xml = s "alt" alt#text xml in
		 let xml = s "max_dist_from_home" mdfh#text xml in
		 let xml = s "name" name#text xml in
		 ignore (load_xml_fp geomap accel_group xml);
		 dialog#destroy ()
	       end);
      dialog#show ())

	
(** Loading a flight plan for edition *)
  let load_fp = fun geomap accel_group () ->
    if_none (fun () ->
      match GToolbox.select_file ~title:"Open flight plan" ~filename:(path_fps^"*.xml") () with
	None -> ()
      | Some xml_file ->
	  try
	    let xml = Xml.parse_file xml_file in
	    ignore (load_xml_fp geomap accel_group ~xml_file xml)
	  with
	    Dtd.Check_error(e) -> 
	      let m = sprintf "Error while loading %s:\n%s" xml_file (Dtd.check_error e) in
	      GToolbox.message_box "Error" m)

  let create_wp = fun geo ->
    match !current_fp with
      None -> 
	GToolbox.message_box "Error" "Load a flight plan first";
	failwith "create_wp"
    | Some (fp,_) ->
	fp#add_waypoint geo


  let save_fp = fun () ->
    match !current_fp with
      None -> () (* Nothing to save *)
    | Some (fp, filename) ->
	match GToolbox.select_file ~title:"Save Flight Plan" ~filename () with
	  None -> ()
	| Some file -> 
	    let f  = open_out file in
	    fprintf f "%s\n" (Xml.to_string_fmt fp#xml);
	    close_out f

  let ref_point_of_waypoint = fun xml ->
    Xml.Element("point", ["x",Xml.attrib xml "x";
			  "y",Xml.attrib xml "y";
			  "geo", Xml.attrib xml "name"],[])


(** Calibration of chosen image (requires a dummy flight plan) *)
  let calibrate_map = fun (geomap:G.widget) accel_group () ->
    match !current_fp with
    | Some (_fp,_) ->  GToolbox.message_box "Error" "Close current flight plan before calibration"
    | None ->
	match GToolbox.select_file ~filename:default_path_maps ~title:"Open Image" () with
	  None -> ()
	| Some image -> 
	    (** Displaying the image in the NW corner *)
	    let pixbuf = GdkPixbuf.from_file image in
	    let pix = GnoCanvas.pixbuf ~pixbuf ~props:[`ANCHOR `NW] geomap#canvas#root in
	    let (x0, y0) = geomap#canvas#get_scroll_offsets in
	    let (x,y) = geomap#canvas#window_to_world (float x0) (float y0) in
	    pix#move x y;

	    (** Open a dummy flight plan *)
	    let dummy_georef =
	      match geomap#georef with
		None -> {posn_lat = (Deg>>Rad)43.; posn_long = (Deg>>Rad)1. }
	      | Some geo -> geo in
	    let fp_xml = dummy_fp dummy_georef in
	    let fp = load_xml_fp geomap accel_group fp_xml in
	    
	    (** Dialog to finish calibration *)
	    let dialog = GWindow.window ~border_width:10 ~title:"Map calibration" () in
	    let v = GPack.vbox ~packing:dialog#add () in
	    let _ = GMisc.label ~text:"Choose 2 (or more) waypoints (Ctrl-Left)\nRename the waypoints with their geographic coordinates\nFor example: 'WGS84 43.123456 1.234567' or 'UTM 530134 3987652 12' or 'LBT2e 123456 543210'\nClick the button below to save the XML result file\n" ~packing:v#add () in
	    let h = GPack.hbox ~packing:v#pack () in
	    let cancel = GButton.button ~stock:`CLOSE ~packing:h#add () in
	    let cal = GButton.button ~stock:`OK ~packing:h#add () in
	    let destroy = fun () ->
	      dialog#destroy ();
	      close_fp ();
	      pix#destroy () in
	    ignore(cancel#connect#clicked ~callback:destroy);
	    ignore(cal#connect#clicked ~callback:(fun _ ->
	      let points = List.map XmlEdit.xml_of_node fp#waypoints in
	      let points = List.map ref_point_of_waypoint points in
	      let xml = Xml.Element ("map", 
				     ["file", Filename.basename image;
				      "projection", geomap#projection],
				     points) in
	      match GToolbox.select_file ~filename:(default_path_maps//".xml") ~title:"Save calibrated map" () with
		None -> ()
	      | Some xml_file ->
		  let f = open_out xml_file in
		  Printf.fprintf f "%s\n" (Xml.to_string_fmt xml);
		  close_out f));
	    cal#grab_default ();
	    dialog#show ()

  let radius = ref 30.
  let path = ref (ref [])
  let cur_arc = ref None
  and cur_seg = ref None
  and cur_f = ref { G2D.x2D = 0.; y2D = 0. }

  let set_segment = fun s p1 p2 ->
    let x1 = p1.G2D.x2D and y1 = p1.G2D.y2D
    and x2 = p2.G2D.x2D and y2 = p2.G2D.y2D in
    s#set [`POINTS [|x1;y1;x2;y2|]]

  let arc_from_points = fun (geomap:G.widget) c l f s ->
    let cl = G2D.vect_make c l
    and cf = G2D.vect_make c f in
    let pol_cl = G2D.cart2polar cl in
    let al = pol_cl.G2D.theta2D
    and af = (G2D.cart2polar cf).G2D.theta2D in
    let xc = c.G2D.x2D and yc = c.G2D.y2D in
    let (a1, a2) = if s > 0. then (al, af) else (af, al) in
    geomap#arc ~nb_points:10 ~fill_color:"blue" ~width:2 (xc,yc) pol_cl.G2D.r2D a1 a2

      (** Update path after waypoint move *)
  let update_path = fun (geomap:G.widget) path waypoint ->
    let n = waypoint#name in
    let rec loop = function
	[] -> failwith (sprintf "update_path: %s not found" n)
      | [p] -> [p]
      | (wp, Some arc, Some seg, _f, radius)::wps ->
	  let new_wps = if wp#name = waypoint#name then wps else loop wps in
	  begin
	    match new_wps with
	      [] -> failwith "unreachable"
	    | (wp1, _arc1, _seg1, f1, _r1)::new_wps' -> (* Previous *)
		let wp1_2D = geomap#pt2D_of wp1#pos
		and wp_2D = geomap#pt2D_of wp#pos in
		match new_wps' with
		  [] -> (* wp is the second point: simple segment *)
		    set_segment seg wp1_2D wp_2D;
		    (wp, Some arc, Some seg, wp1_2D, radius)::new_wps
		| _ -> (* At least 2 points before *)
		    let (c, f, s) = G2D.arc_segment f1 wp1_2D wp_2D radius in
		    set_segment seg f wp_2D;
		    arc#destroy ();
		    let new_arc = arc_from_points geomap c wp1_2D f s in
		    (wp, Some new_arc, Some seg, f, radius)::new_wps
	  end
      | _ -> failwith "update_path" in
    path := loop !path

	

  let path_button = fun (geomap:G.widget) (xw, yw) ->
    let geo = geomap#of_world (xw, yw) in
    let wp = create_wp geo in
    let cur_path = !path in
    wp#connect (fun () -> update_path geomap cur_path wp);
    if ! !path = [] then
      cur_f := {G2D.x2D=xw; G2D.y2D=yw};
    cur_path := (wp, !cur_arc, !cur_seg, !cur_f, !radius) :: ! cur_path;
    cur_arc := Some (GnoCanvas.line geomap#canvas#root);
    cur_seg := Some (geomap#segment ~fill_color:"blue" ~width:2 geo geo)

  let path_notify (geomap:G.widget) (xw, yw) =
    match ! !path with
      [] -> false (** Empty path: nothing to do *)
    | (wp1, _, _, ll,_) :: p -> (** Last is wp1 *)
	begin
	  match !cur_seg with
	    None -> failwith "path_notify"
	  | Some segment -> 
	      let l = geomap#pt2D_of wp1#pos
	      and cur_2D = {G2D.x2D = xw; y2D = yw } in
	      match p with
		[] -> (** Only 1 point in the current path:add a simple segment*)
		  set_segment segment l cur_2D;
	      | _::_ -> (** Already 2 points: add an arc and a segment *)
		  match !cur_arc with
		    None -> failwith "path_notify"
		  | Some arc ->
		      arc#destroy ();
		      let (c, f, s) = G2D.arc_segment ll l cur_2D !radius in
		      set_segment segment f cur_2D;
		      let arc = arc_from_points geomap c l f s in
		      cur_arc := Some arc;
		      cur_f := f
	end;	    
	true

	  
  let path_close = fun () ->
    let destroy = fun ref ->
      match !ref with
	None -> () 
      | Some s -> s#destroy (); ref := None in
    destroy cur_arc;
    destroy cur_seg;
    begin
      match !current_fp with
	None -> ()
      | Some (fp, _) ->
	  fp#insert_path (List.map (fun (wp,_,_,_,r) -> (wp, r)) (List.rev ! !path));
    end;
    path := ref []

  let path_change_radius = function
      `UP -> radius := !radius *. 1.1
    | `DOWN -> radius := !radius /. 1.1
    | _ -> ()
end (** Edit module *)
    

(******* Mouse motion handling **********************************************)
let motion_notify = fun (geomap:G.widget) ev ->
  let xc = GdkEvent.Motion.x ev 
  and yc = GdkEvent.Motion.y ev in
  let xwyw = geomap#window_to_world xc yc in
  Edit.path_notify geomap xwyw


(******* Mouse wheel handling ***********************************************)
let any_event = fun (geomap:G.widget) ev ->
  match GdkEvent.get_type ev with
    `SCROLL ->
      let state = GdkEvent.Scroll.state (GdkEvent.Scroll.cast ev) in
      if Gdk.Convert.test_modifier `CONTROL state && Gdk.Convert.test_modifier `SHIFT state then 
	let scroll_event = GdkEvent.Scroll.cast ev in
	Edit.path_change_radius (GdkEvent.Scroll.direction scroll_event);
	let xc = GdkEvent.Scroll.x scroll_event 
	and yc = GdkEvent.Scroll.y scroll_event in
	let xwyw = geomap#window_to_world xc yc in
	Edit.path_notify geomap xwyw
      else
	false
  | _ ->
      false



(******* Mouse buttons handling **********************************************)
let button_press = fun (geomap:G.widget) ev ->
  let state = GdkEvent.Button.state ev in
  if GdkEvent.Button.button ev = 3 && Gdk.Convert.test_modifier `CONTROL state && Gdk.Convert.test_modifier `SHIFT state then begin
    Edit.path_close ();
    true
  end else if GdkEvent.Button.button ev = 3 then begin
    (** Display a tile from Google Maps or IGN *)
    let xc = GdkEvent.Button.x ev 
    and yc = GdkEvent.Button.y ev in
    let (xw,yw) = geomap#window_to_world xc yc in

    let thread = fun f x -> ignore (Thread.create f x) in
    let wgs84 = geomap#of_world (xw,yw) in
    let display_ign = fun () ->	
      thread (MapIGN.display_tile geomap) wgs84
    and display_gm = fun () ->
      thread (fun () ->
	try ignore (MapGoogle.display_tile geomap wgs84) with
	  Gm.Not_available -> ())
	() in

    GToolbox.popup_menu ~entries:[`I ("Load Google tile", display_gm);
				  `I ("Load IGN tile", display_ign)]
      ~button:3 ~time:(Int32.of_int 00);
    true;
  end else if GdkEvent.Button.button ev = 1 && Gdk.Convert.test_modifier `CONTROL state then
    if Gdk.Convert.test_modifier `SHIFT state then begin
      let xc = GdkEvent.Button.x ev in
      let yc = GdkEvent.Button.y ev in
      let xyw = geomap#canvas#window_to_world xc yc in
      Edit.path_button geomap xyw;
      true
    end else begin
      let xc = GdkEvent.Button.x ev in
      let yc = GdkEvent.Button.y ev in
      let xyw = geomap#canvas#window_to_world xc yc in
      let geo = geomap#of_world xyw in
      ignore (Edit.create_wp geo);
      true
    end else 
    false


(******* Real time handling of flying A/Cs ***********************************) module Live = struct
  module Ground_Pprz = Pprz.Messages(struct let name = "ground" end)

  type color = string
  type aircraft = {
      ac_name : string;
      config : Pprz.values;
      track : MapTrack.track;
      color: color;
      fp_group : MapFP.flight_plan;
      fp : Xml.xml;
      dl_settings_window : GWindow.window;
      dl_settings_adjustments : float array;
      block_label : GMisc.label;
      apmode_label : GMisc.label;
      blocks : (int * string) list;
      mutable last_ap_mode : string
    }

  let live_aircrafts = Hashtbl.create 3

  let aircraft_pos_msg = fun track wgs84 heading altitude speed climb ->
    let h = 
      try
	Srtm.of_wgs84 wgs84
      with
	_ -> truncate altitude in
    track#move_icon wgs84 heading altitude (float_of_int h) speed climb

  let carrot_pos_msg = fun track wgs84 ->
    track#move_carrot wgs84

  let cam_pos_msg = fun track wgs84 target_wgs84 ->
    track#move_cam wgs84 target_wgs84

  let circle_status_msg = fun track wgs84 radius ->
    track#draw_circle wgs84 radius

  let segment_status_msg = fun track geo1 geo2 ->
    track#draw_segment geo1 geo2

  let ap_status_msg = fun track flight_time ->
    track#update_ap_status flight_time
      

  let show_mission = fun ac on_off ->
    let a = Hashtbl.find live_aircrafts ac in
    if on_off then
      a.fp_group#show ()
    else
      a.fp_group#hide ()

  let resize_track = fun ac track ->
    match
      GToolbox.input_string ~text:(soi track#size) ~title:ac "Track size"
    with
      None -> ()
    | Some s -> track#resize (int_of_string s)

  let commit_changes = fun ac ->
    let a = Hashtbl.find live_aircrafts ac in
    List.iter 
      (fun w ->
	let (i, w) = a.fp_group#index w in
	if w#moved then 
	  let wgs84 = w#pos in
	  let vs = ["ac_id", Pprz.String ac;
		    "wp_id", Pprz.Int i;
		    "lat", Pprz.Float ((Rad>>Deg)wgs84.posn_lat);
		    "long", Pprz.Float ((Rad>>Deg)wgs84.posn_long);
		    "alt", Pprz.Float w#alt
		  ] in
	  Ground_Pprz.message_send "map2d" "MOVE_WAYPOINT" vs)
      a.fp_group#waypoints

  let send_event = fun ac e ->
    Ground_Pprz.message_send "map2d" "SEND_EVENT" 
      ["ac_id", Pprz.String ac; "event_id", Pprz.Int e]

  let dl_settings = fun ac_id ac_name xml ->
    let window = GWindow.window ~title:(sprintf "Datalink settings %s" ac_name) () in
    let quit = fun () -> GMain.Main.quit (); exit 0 in
    ignore (window#connect#destroy ~callback:quit);
    let vbox = GPack.vbox ~packing:window#add () in

    begin
      try
	let settings = Xml.children (ExtXml.child xml "dl_settings") in
	let n = List.length settings in
	let current_values = Array.create n 42. in
	let i = ref 0 in
	List.iter
	  (fun s ->
	    let f = fun a -> float_of_string (ExtXml.attrib s a) in
	    let lower = f "min"
	    and upper = f "max"
	    and step_incr = f "step" in
	    let value = (lower +. upper) /. 2. in
	    let text = ExtXml.attrib s "var" in
	    let adj = GData.adjustment ~value ~lower ~upper:(upper+.10.) ~step_incr () in
	    let hbox = GPack.hbox ~width:500 ~packing:vbox#add () in
	    let _l = GMisc.label ~width:100 ~text ~packing:hbox#pack () in
	    let _v = GMisc.label ~width:50 ~text:"N/A" ~packing:hbox#pack () in
	    let _scale = GRange.scale `HORIZONTAL ~digits:2 ~adjustment:adj ~packing:hbox#add () in
	    let ii = !i in
(***	    let b = GButton.button ~label:"Update" ~stock:`REFRESH ~packing:hbox#pack () in
	    let update = fun () -> adj#set_value current_values.(ii) in
	    ignore (b#connect#clicked ~callback:update); ***)

	    let update_current_value = fun _ ->
	      let s = string_of_float current_values.(ii) in
	      if _v#text <> s then
		_v#set_text s;
	      true in

	    ignore (Glib.Timeout.add 500 update_current_value);

	    let callback = fun () -> 
	      let vs = ["ac_id", Pprz.String ac_id; "index", Pprz.Int ii;"value", Pprz.Float adj#value] in
	      Ground_Pprz.message_send "dl" "DL_SETTING" vs in
	    let b = GButton.button ~label:"Commit" ~stock:`APPLY ~packing:hbox#pack () in
	    ignore (b#connect#clicked ~callback);

	    incr i
	  )
	  settings;
	window, current_values
      with _ -> window, [||]
    end

  let center = fun geomap track () ->
    match track#last with
      None -> ()
    | Some geo ->
	geomap#center geo


  let active_dl_settings = fun ac_id x ->
    let ac = Hashtbl.find live_aircrafts ac_id in
    let w = ac.dl_settings_window in
    if x then w#show () else w#misc#hide ()

  let blocks_of_stages = fun stages ->
  let blocks = ref [] in
  List.iter (fun x ->
    let name = ExtXml.attrib x "block_name"
    and id = int_attr x "block" in
    if not (List.mem_assoc id !blocks) then
      blocks := (id, name) :: !blocks)
    (Xml.children stages);
  List.sort compare !blocks

  let menu_entry_of_block = fun ac (id, name) ->
    let send_msg = fun () ->
      Ground_Pprz.message_send "map2d" "JUMP_TO_BLOCK" 
	["ac_id", Pprz.String ac; "block_id", Pprz.Int id] in
    `I (name, send_msg)

  let reset_waypoints = fun fp () ->
    List.iter (fun w ->
      let (i, w) = fp#index w in
      w#reset_moved ())
      fp#waypoints


  let create_ac = fun (geomap:G.widget) ac_id config ->
    let color = Pprz.string_assoc "default_gui_color" config
    and name = Pprz.string_assoc "ac_name" config in

    (** Get the flight plan **)
    let fp_url = Pprz.string_assoc "flight_plan" config in
    let fp_file = Http.file_of_url fp_url in
    let fp_xml_dump = Xml.parse_file fp_file in
    let stages = ExtXml.child fp_xml_dump "stages" in
    let blocks = blocks_of_stages stages in

    let label = GPack.hbox ~spacing:3 () in
    let eb = GBin.event_box ~width:10 ~height:10 ~packing:label#pack () in
    eb#coerce#misc#modify_bg [`NORMAL, `NAME color];
    let ac_label = GMisc.label ~text:name ~packing:label#pack () in
    let apmode_label = GMisc.label ~packing:label#pack () in
    let block_label = GMisc.label ~packing:label#pack () in

    let ac_mi = GMenu.image_menu_item ~image:label ~packing:geomap#menubar#append () in
    let ac_menu = GMenu.menu () in
    ac_mi#set_submenu ac_menu;
    let ac_menu_fact = new GMenu.factory ac_menu in
    let fp = ac_menu_fact#add_check_item "Fligh Plan" ~active:false in
    ignore (fp#connect#toggled (fun () -> show_mission ac_id fp#active));
    
    let track = new MapTrack.track ~name ~color:color geomap in

    ignore (ac_menu_fact#add_item "Center A/C" ~callback:(center geomap track));
    
    ignore (ac_menu_fact#add_item "Clear Track" ~callback:(fun () -> track#clear_map2D));
    ignore (ac_menu_fact#add_item "Resize Track" ~callback:(fun () -> resize_track ac_id track));
    let reset_wp_menu = ac_menu_fact#add_item "Reset Waypoints" in

    let jump_block_entries = List.map (menu_entry_of_block ac_id) blocks in
    
    let sm = ac_menu_fact#add_submenu "Datalink" in
    let dl_menu = [
      `M ("Jump to block", jump_block_entries);
      `I ("Commit Moves", (fun () -> commit_changes ac_id));
      `I ("Event 1", (fun () -> send_event ac_id 1));
      `I ("Event 2", (fun () -> send_event ac_id 2));
      `C ("Settings", false, (active_dl_settings ac_id))] in

    GToolbox.build_menu sm ~entries:dl_menu;

    let cam = ac_menu_fact#add_check_item "Cam Display" ~active:false in
    ignore (cam#connect#toggled (fun () -> track#set_cam_state cam#active));
    let params = ac_menu_fact#add_check_item "flight param. display" ~active:false in
    ignore (params#connect#toggled (fun () -> track#set_params_state params#active));

    let fp_xml = ExtXml.child fp_xml_dump "flight_plan" in

    let ds_window, ds_adjs = dl_settings ac_id name fp_xml in

    let fp = load_mission color geomap fp_xml in
    fp#hide ();
    ignore (reset_wp_menu#connect#activate (reset_waypoints fp));
    Hashtbl.add live_aircrafts ac_id { track = track; color = color; 
				       fp_group = fp ; config = config ; 
				       fp = fp_xml; ac_name = name;
				       dl_settings_adjustments = ds_adjs;
				       dl_settings_window = ds_window;
				       block_label = block_label;
				       apmode_label = apmode_label;
				       blocks = blocks; last_ap_mode= ""
				     };
    ignore (Strip.add strip_table ac_id config color)

  (** Bind to message while catching all the esceptions of the callback *)
  let safe_bind = fun msg cb ->
    let safe_cb = fun sender vs ->
      try cb sender vs with _ -> () in
    ignore (Ground_Pprz.message_bind msg safe_cb)
      

  let ask_config = fun geomap ac ->
    let get_config = fun _sender values ->
      create_ac geomap ac values
    in
    Ground_Pprz.message_req "map2d" "CONFIG" ["ac_id", Pprz.String ac] get_config

      

  let one_new_ac = fun (geomap:G.widget) ac ->
    if not (Hashtbl.mem live_aircrafts ac) then begin
      ask_config geomap ac
    end

  let get_wind_msg = fun sender vs ->
    try
      let ac_strip = Strip.find (Pprz.string_assoc "ac_id" vs) in
      let set_label = fun label_name field_name ->
	Strip.set_label ac_strip label_name 
	  (Printf.sprintf "%.1f" (Pprz.float_assoc field_name vs))
      in
      set_label "wind" "wspeed";
      set_label "dir" "dir"
    with
      Not_found -> ()

  let get_fbw_msg = fun sender vs ->
    try
      let ac_strip = Strip.find (Pprz.string_assoc "ac_id" vs) in
      Strip.set_label ac_strip "RC" (Pprz.string_assoc "rc_status" vs)
    with
      Not_found -> ()
	  

  let get_engine_status_msg = fun sender vs ->
    try
      let ac_strip = Strip.find (Pprz.string_assoc "ac_id" vs) in
      Strip.set_label ac_strip "throttle" 
	(string_of_float (Pprz.float_assoc "throttle" vs));
      Strip.set_bat ac_strip (Pprz.float_assoc "bat" vs)
    with
      Not_found -> ()
	  
  let get_if_calib_msg = fun sender vs ->
    try
      let ac_strip = Strip.find (Pprz.string_assoc "ac_id" vs) in
      Strip.set_label ac_strip "settings" (Pprz.string_assoc "if_mode" vs)
    with
      Not_found -> ()

  let listen_wind_msg = fun () ->
    safe_bind "WIND" get_wind_msg

  let listen_fbw_msg = fun () ->
    safe_bind "FLY_BY_WIRE" get_fbw_msg

  let listen_engine_status_msg = fun () ->
    safe_bind "ENGINE_STATUS" get_engine_status_msg

  let listen_if_calib_msg = fun () ->
   safe_bind "INFLIGH_CALIB" get_if_calib_msg

  let list_separator = Str.regexp ","
      
  let aircrafts_msg = fun (geomap:G.widget) acs ->
    let acs = Pprz.string_assoc "ac_list" acs in
    let acs = Str.split list_separator acs in
    List.iter (one_new_ac geomap) acs


  let listen_dl_value = fun () ->
    let get_dl_value = fun _sender vs ->
      let ac_id = Pprz.string_assoc "ac_id" vs in
      try
	let ac = Hashtbl.find live_aircrafts ac_id in
	let adjs = ac.dl_settings_adjustments in
	let csv = Pprz.string_assoc "values" vs in
	let values = Array.of_list (Str.split list_separator csv) in
	for i = 0 to min (Array.length values) (Array.length adjs) - 1 do
	  adjs.(i) <- float_of_string values.(i)
	done
      with Not_found -> ()
    in
    safe_bind "DL_VALUES" get_dl_value


  let listen_flight_params = fun () ->
    let get_fp = fun _sender vs ->
      let ac_id = Pprz.string_assoc "ac_id" vs in
      try
	let ac_strip = Strip.find ac_id in
	let ac = Hashtbl.find live_aircrafts ac_id in
	let a = fun s -> Pprz.float_assoc s vs in
	let wgs84 = { posn_lat = (Deg>>Rad)(a "lat"); posn_long = (Deg>>Rad)(a "long") } in
	aircraft_pos_msg ac.track wgs84 (a "course") (a "alt")  (a "speed") (a "climb");
	let set_label lbl_name field_name =
	  let s = 
	    if (a field_name) < 0. 
	    then 
	      "- "^(Printf.sprintf "%.1f" (abs_float (a field_name)))
	    else
	      Printf.sprintf "%.1f" (a field_name)
	  in
	  Strip.set_label ac_strip lbl_name s
	in
	set_label "alt" "alt";
	set_label "speed" "speed";
	set_label "climb" "climb"
      with Not_found -> ()
    in
    safe_bind "FLIGHT_PARAM" get_fp;

    let get_ns = fun _sender vs ->
      let ac_id = Pprz.string_assoc "ac_id" vs in
      try
	let ac_strip = Strip.find ac_id in
	let ac = Hashtbl.find live_aircrafts ac_id in
	let a = fun s -> Pprz.float_assoc s vs in
	let wgs84 = { posn_lat = (Deg>>Rad)(a "target_lat"); posn_long = (Deg>>Rad)(a "target_long") } in
	carrot_pos_msg ac.track wgs84;
	let b = List.assoc (Pprz.int_assoc "cur_block" vs) ac.blocks in
	let b = String.sub b 0 (min 10 (String.length b)) in
	ac.block_label#set_label b;
	let set_label = fun l f ->
	  Strip.set_label ac_strip l (Printf.sprintf "%.1f" (Pprz.float_assoc f vs)) in
	set_label "->" "target_alt";
	set_label "/" "target_climb"
      with Not_found -> ()
    in
    safe_bind "NAV_STATUS" get_ns;

    let get_cam_status = fun _sender vs ->
      let ac_id = Pprz.string_assoc "ac_id" vs in
      try
	let ac = Hashtbl.find live_aircrafts ac_id in
	let a = fun s -> Pprz.float_assoc s vs in
	let wgs84 = { posn_lat = (Deg>>Rad)(a "cam_lat"); posn_long = (Deg>>Rad)(a "cam_long") }
	and target_wgs84 = { posn_lat = (Deg>>Rad)(a "cam_target_lat"); posn_long = (Deg>>Rad)(a "cam_target_long") } in
	
	cam_pos_msg ac.track wgs84 target_wgs84
      with Not_found -> () in
    safe_bind "CAM_STATUS" get_cam_status;

    let get_circle_status = fun _sender vs ->
      let ac_id = Pprz.string_assoc "ac_id" vs in
      try
	let ac = Hashtbl.find live_aircrafts ac_id in
	let a = fun s -> Pprz.float_assoc s vs in
	let wgs84 = { posn_lat = (Deg>>Rad)(a "circle_lat"); posn_long = (Deg>>Rad)(a "circle_long") } in
	circle_status_msg ac.track wgs84 (float_of_string (Pprz.string_assoc "radius" vs)) 
      with Not_found -> ()
    in
    safe_bind "CIRCLE_STATUS" get_circle_status;

    let get_segment_status = fun _sender vs ->
      let ac_id = Pprz.string_assoc "ac_id" vs in
      try
	let ac = Hashtbl.find live_aircrafts ac_id in
	let a = fun s -> Pprz.float_assoc s vs in
	let geo1 = { posn_lat = (Deg>>Rad)(a "segment1_lat"); posn_long = (Deg>>Rad)(a "segment1_long") }
	and geo2 = { posn_lat = (Deg>>Rad)(a "segment2_lat"); posn_long = (Deg>>Rad)(a "segment2_long") } in
	segment_status_msg ac.track geo1 geo2
      with Not_found -> ()
    in
    safe_bind "SEGMENT_STATUS" get_segment_status;


    let get_ap_status = fun _sender vs ->
      let ac_id = Pprz.string_assoc "ac_id" vs in
      try
	let ac_strip = Strip.find ac_id in
	let ac = Hashtbl.find live_aircrafts ac_id in
	ap_status_msg ac.track ( float_of_int (Pprz.int32_assoc "flight_time" vs ));
	let ap_mode = Pprz.string_assoc "ap_mode" vs in
	if ap_mode <> ac.last_ap_mode then begin
	  say (sprintf "%s, %s" ac.ac_name ap_mode);
	  ac.last_ap_mode <- ap_mode
	end;
	ac.apmode_label#set_label ap_mode;
	Strip.set_label ac_strip "AP" (Pprz.string_assoc "ap_mode" vs);
	Strip.set_label ac_strip "GPS" (Pprz.string_assoc "gps_mode" vs);
	let ft = 
	  let t = Int32.to_int (Int32.of_string (Pprz.string_assoc "flight_time" vs)) in
	  Printf.sprintf "%02d:%02d:%02d" (t / 3600) ((t mod 3600) / 60) ((t mod 3600) mod 60) in
	Strip.set_label ac_strip "flight_time" ft
      with 
	Not_found -> ()
    in
    safe_bind "AP_STATUS" get_ap_status;

    listen_dl_value ()

  let listen_waypoint_moved = fun () ->
    let get_values = fun _sender vs ->
      let ac_id = Pprz.string_assoc "ac_id" vs in
      let ac = Hashtbl.find live_aircrafts ac_id in
      (** Not_found catched by safe_bind *)
      let wp_id = Pprz.int_assoc "wp_id" vs in
      let a = fun s -> Pprz.float_assoc s vs in
      let geo = { posn_lat = (Deg>>Rad)(a "lat"); posn_long = (Deg>>Rad)(a "long") } in

      (** No indexed access to waypoints: iter and compare: *)
      List.iter (fun w ->
	let (i, w) = ac.fp_group#index w in
	if i = wp_id then begin
	  if not w#moved then w#set geo;
	  raise Exit (** catched by  safe_bind *)
	end)
	ac.fp_group#waypoints
    in
    safe_bind "WAYPOINT_MOVED" get_values
    
end (** module Live *)




(******** Help ***************************************************************)
let keys_help = fun () ->
  GToolbox.message_box ~title:"Keys" ~ok:"Close"
    "Zoom: Mouse Wheel, PgUp, PgDown\n\
    Pan: Shift Middle, Arrows\n\
    Load Map Tile: Right\n\
    Select Region: Shift-Left + Drag\n\
    Create Waypoint: Ctrl-Left\n\
    Move Waypoint: Middle\n\
    Edit Waypoint: Left\n\
    Path Point: Ctrl-Shift-Left\n\
    Path Radius: Ctrl-Shift-Wheel\n\
    Path End: Ctrl-Shift-Right"


(******** Sectors **********************************************************)

module Sector = struct
  let rec display = fun (geomap:G.widget) r ->
    
    match String.lowercase (Xml.tag r) with
      "disc" ->
	let rad = float_of_string (ExtXml.attrib r "radius")
	and geo = Latlong.of_string (ExtXml.attrib (ExtXml.child r "point") "pos") in
	ignore (geomap#circle ~width:5 ~color:"red" geo rad)
    | "union" ->
	List.iter (display geomap) (Xml.children r)
    | "polygon" ->
	let pts = List.map (fun x ->  Latlong.of_string (ExtXml.attrib x "pos")) (Xml.children r) in
	let pts = Array.of_list pts in
	let n = Array.length pts in
	for i = 0 to n - 1 do
	  ignore (geomap#segment ~width:5 ~fill_color:"red" pts.(i) pts.((i+1)mod n))
	done
    |x -> fprintf stderr "Sector.display: '%s' not yet\n%!" x	    
	  

  let display_sector = fun (geomap:G.widget) sector ->
    display geomap (ExtXml.child sector "0")

    
  let load = fun geomap () ->
    match GToolbox.select_file ~title:"Load sectors" ~filename:(path_fps^"*.xml") () with
      None -> ()
    | Some f ->
	try
	  let xml = Xml.parse_file f in
	  List.iter (display_sector geomap) (Xml.children xml)
	with
	  Dtd.Prove_error(e) -> 
	    let m = sprintf "Error while loading %s:\n%s" f (Dtd.prove_error e) in
	    GToolbox.message_box "Error" m
end
      


(***************** MAIN ******************************************************)
let _main =
  let ivy_bus = ref "127.255.255.255:2010"
  and geo_ref = ref ""
  and map_file = ref ""
  and center = ref ""
  and zoom = ref 1.
  and projection= ref G.UTM
  and display_strip = ref false in
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.25:2010";
      "-strip", Arg.Unit (fun () -> display_strip := true), "Display strip";
      "-ref", Arg.Set_string geo_ref, "Geographic ref (default '')";
      "-zoom", Arg.Set_float zoom, "Initial zoom";
      "-center", Arg.Set_string center, "Initial map center";
      "-mercator", Arg.Unit (fun () -> projection:=G.Mercator),"Switch to (Google Maps) Mercator projection";
      "-lambertIIe", Arg.Unit (fun () -> projection:=G.LambertIIe),"Switch to LambertIIe projection";
      "-ign", Arg.Set_string IGN.data_path, "IGN tiles path";
      "-speech", Arg.Set speech, "Speech";
      "-m", Arg.String (fun x -> map_file := x), "Map description file"] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: ";
  (*                                 *)
  Ivy.init "Paparazzi map 2D" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  Srtm.add_path default_path_srtm;
  Gm.cache_path := var_maps_path;
  IGN.cache_path := var_maps_path;
  
  (** window for map2d **)
  let window = GWindow.window ~title: "Map2d" ~border_width:1 ~width:400 () in
  let vbox= GPack.vbox ~packing: window#add () in

  (** window for vertical situation *)
  let vertical_situation = GWindow.window ~title: "Vertical" ~border_width:1 ~width:400 () in
  let _vertical_vbox= GPack.vbox ~packing: vertical_situation#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in

  ignore (window#connect#destroy ~callback:quit);
  ignore (vertical_situation#connect#destroy ~callback:quit);
  ignore (strip_panel#connect#destroy ~callback:quit);

  let geomap = new G.widget ~projection:!projection ~height:400 () in

  let menu_fact = new GMenu.factory geomap#file_menu in
  let accel_group = menu_fact#accel_group in

  ignore (geomap#canvas#event#connect#button_press (button_press geomap));
  ignore (geomap#canvas#event#connect#motion_notify (motion_notify geomap));
  ignore (geomap#canvas#event#connect#any (any_event geomap));

  (** widget displaying aircraft vertical position  *)

  let _active_vertical = fun x ->
    if x then vertical_situation#show () else vertical_situation#misc#hide () in
  ignore (menu_fact#add_item "Redraw" ~key:GdkKeysyms._L ~callback:geomap#canvas#update_now);
  ignore (menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);

  (* Maps handling *)
  let map_menu = geomap#factory#add_submenu "Maps" in
  let map_menu_fact = new GMenu.factory ~accel_group map_menu in
  ignore (map_menu_fact#add_item "Load" ~key:GdkKeysyms._M ~callback:(load_map geomap));
  ignore (map_menu_fact#add_item "Calibrate" ~key:GdkKeysyms._C ~callback:(Edit.calibrate_map geomap accel_group));
  ignore (map_menu_fact#add_item "GoogleMaps Fill" ~key:GdkKeysyms._G ~callback:(fun _ -> GM.fill_tiles geomap));
  ignore (map_menu_fact#add_check_item "GoogleMaps Http" ~key:GdkKeysyms._H ~active:true ~callback:GM.active_http);
  ignore (map_menu_fact#add_check_item "GoogleMaps Auto" ~active:false ~callback:GM.active_auto);
  ignore (map_menu_fact#add_item "Map of Region" ~key:GdkKeysyms._R ~callback:(map_from_region geomap));
  ignore (map_menu_fact#add_item "Map of Google Tiles" ~key:GdkKeysyms._T ~callback:(GM.map_from_tiles geomap));
  ignore (map_menu_fact#add_item "Load sector" ~key:GdkKeysyms._T ~callback:(Sector.load geomap));
  
  (** Connect Google Maps display to view change *)
  geomap#connect_view (fun () -> GM.update geomap);
  
  (** Flight plan editing *)
  let fp_menu = geomap#factory#add_submenu "Edit" in
  let fp_menu_fact = new GMenu.factory ~accel_group fp_menu in
  ignore (fp_menu_fact#add_item "New flight plan" ~key:GdkKeysyms._N ~callback:(Edit.new_fp geomap accel_group));
  ignore (fp_menu_fact#add_item "Open flight plan" ~key:GdkKeysyms._O ~callback:(Edit.load_fp geomap accel_group));
  ignore (fp_menu_fact#add_item "Save flight plan" ~key:GdkKeysyms._S ~callback:(Edit.save_fp));
  ignore (fp_menu_fact#add_item "Close flight plan" ~key:GdkKeysyms._W ~callback:(Edit.close_fp));

  (** Help pushed to the right *)
  let mi = GMenu.menu_item ~label:"Help" ~right_justified:true ~packing:geomap#menubar#append () in
  let help_menu = GMenu.menu () in
  GToolbox.build_menu help_menu ~entries:[`I ("Keys", keys_help)];
  mi#set_submenu help_menu;

  (** Separate from A/C menus *)
  ignore (geomap#factory#add_separator ());

  (** Pack the canvas in the window *)
  vbox#pack ~expand:true geomap#frame#coerce;

  (** Set the initial soom *)
  geomap#zoom !zoom;

  (** Periodically probe new A/Cs *)
  ignore (Glib.Timeout.add 2000 (fun () -> Live.Ground_Pprz.message_req "map2d" "AIRCRAFTS" [] (fun _sender vs -> Live.aircrafts_msg geomap vs); false));

  (** New aircraft message *)
  Live.safe_bind "NEW_AIRCRAFT" (fun _sender vs -> Live.one_new_ac geomap (Pprz.string_assoc "ac_id" vs));

  (** Listen for all messages on ivy *)
  Live.listen_flight_params ();
  Live.listen_wind_msg ();
  Live.listen_fbw_msg ();
  Live.listen_engine_status_msg ();
  Live.listen_if_calib_msg ();
  Live.listen_waypoint_moved ();

  (** Display the window *)
  window#add_accel_group accel_group;
  window#show ();

  if !display_strip then strip_panel#show ();

  (** Center the map as required *)
  if !center <> "" then begin
    try geomap#center (Latlong.of_string !center) with
      _ -> GToolbox.message_box "Error" (sprintf "Cannot center at '%s' (no ref ?)" !center)
  end;

  (** Loading an initial map *)
  if !geo_ref <> "" then
    set_georef_if_none geomap (Latlong.of_string !geo_ref);
  if !map_file <> "" then begin
    let xml_map_file = if !map_file.[0] <> '/' then Filename.concat default_path_maps !map_file else !map_file in
    display_map geomap xml_map_file
  end;

  say "Welcome to paparazzi";

  (** Threaded main loop (map tiles loaded concurently) *)
  GtkThread.main ()
