(*
 * $Id$
 *
 * Multi aircrafts map display
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

module Ground_Pprz = Pprz.Protocol(struct let name = "ground" end)

let float_attr = fun xml a -> float_of_string (ExtXml.attrib xml a)
let int_attr = fun xml a -> int_of_string (ExtXml.attrib xml a)
type color = string

let soi = string_of_int
let sof = string_of_float
let list_separator = Str.regexp ","

(*** parameters used for creating the vertical display window
let _max_graduations = 20
let _vertical_delta = 5.0
let _max_east = 2000.0
let _max_label = 4 
let _approx_ground_altitude = ref 0.0
***)

module G = MapCanvas

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

let labelled_entry = fun text value h ->
  let _ = GMisc.label ~text ~packing:h#add () in
  GEdit.entry ~text:value ~packing:h#add ()


(** Dummy flight plan *)
let dummy_fp = fun latlong ->
  Xml.Element("flight_plan",
	      ["lat0", sof ((Rad>>Deg)latlong.posn_lat);
	       "lon0", sof ((Rad>>Deg)latlong.posn_long);
	       "alt", "42.";
	       "MAX_DIST_FROM_HOME", "1000."
	     ],
	      [Xml.Element("waypoints", [],[])])
    
type aircraft = {
    config : Pprz.values;
    track : MapTrack.track;
    color: color;
    fp_group : MapFP.flight_plan;
    fp : Xml.xml;
    dl_settings : GWindow.window
  }

let live_aircrafts = Hashtbl.create 3

let set_georef_if_none = fun geomap wgs84 ->
  match geomap#georef with
    None -> geomap#set_georef wgs84
  | Some _ -> ()


let display_map = fun (geomap:G.widget) xml_map ->
  let dir = Filename.dirname xml_map in
  let xml_map = Xml.parse_file xml_map in
  let image = dir // ExtXml.attrib xml_map "file" in

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
      geomap#moveto geo1
  | _ -> failwith (sprintf "display_map: two ref points required")


let load_map = fun (geomap:G.widget) () ->
  match GToolbox.select_file ~title:"Open Map" ~filename:(default_path_maps^"*.xml") () with
    None -> ()
  | Some f -> display_map geomap f


(** FIXME : also in MapFP.ml *)
let georef_of_fp = fun xml ->
  let lat0 = float_attr xml "lat0"
  and lon0 = float_attr xml "lon0" in
  {posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 }

	    
let load_mission = fun color geomap xml ->
  set_georef_if_none geomap (georef_of_fp xml);
  new MapFP.flight_plan geomap color fp_dtd xml

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
    

let show_mission = fun geomap ac on_off ->
  let a = Hashtbl.find live_aircrafts ac in
  if on_off then
    a.fp_group#show ()
  else
    a.fp_group#hide ()


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
  


let resize_track = fun ac track ->
  match GToolbox.input_string ~text:(string_of_int track#size) ~title:ac "Track size" with
    None -> ()
  | Some s -> track#resize (int_of_string s)




(***
let canvas_color = fun gdk_color ->
  let r = Gdk.Color.red gdk_color
  and g = Gdk.Color.green gdk_color
  and b = Gdk.Color.blue gdk_color in
  Printf.sprintf "#%02x%02x%02x" r g b

let gdk_color = fun s ->
  Gdk.Color.alloc (Gdk.Color.get_system_colormap ()) (`NAME s)

let colorsel = 
  let dialog_ref = ref None in
  fun (track:MapTrack.track) (box:GObj.widget) ->
    let colordlg =
      (** Creates the dialog if it has not been done yet *)
      match !dialog_ref with
      | None ->
	  let dlg = GWindow.color_selection_dialog ~title:"Select track color" () in
	  dialog_ref := Some dlg;
	  let callback = fun response ->
	    begin
	      match response with
		`OK -> 
		  let c = dlg#colorsel#color in
		  box#coerce#misc#modify_bg [`NORMAL, `COLOR c];
		  track#set_color (canvas_color c)
	      | _ -> ()
	    end;
	    dlg#misc#hide ()
	  in
	  ignore (dlg#connect#response ~callback);
	  dlg
      | Some dlg -> dlg
    in
    let colorsel = colordlg#colorsel in

    colorsel#set_has_palette true;
    ignore (colordlg#run ())
***)



let dl_settings = fun ac_id xml ->
  let window = GWindow.window ~title:("Datalink settings ") () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);
  let vbox = GPack.vbox ~packing:window#add () in

  begin
    try
      let settings = Xml.children (ExtXml.child xml "dl_settings") in
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
	  let hbox = GPack.hbox ~width:400 ~packing:vbox#add () in
	  let _l = GMisc.label ~width:100 ~text ~packing:hbox#pack () in
	  let _scale = GRange.scale `HORIZONTAL ~digits:2 ~adjustment:adj ~packing:hbox#add () in
	  let ii = !i in
	  let callback = fun () -> 
	    let vs = ["ac_id", Pprz.String ac_id; "index", Pprz.Int ii;"value", Pprz.Float adj#value] in
	    Ground_Pprz.message_send "dl" "DL_SETTING" vs in
	  let b = GButton.button ~label:"Commit" ~stock:`OK ~packing:hbox#pack () in
	  ignore (b#connect#clicked ~callback);
	  incr i
	)
      settings
    with _ -> ()
  end;
  window

let center = fun geomap track () ->
  match track#last with
    None -> ()
  | Some geo ->
      geomap#center geo;
      geomap#canvas#update_now ()


let active_dl_settings = fun ac_id x ->
  let ac = Hashtbl.find live_aircrafts ac_id in
  let w = ac.dl_settings in
  if x then w#show () else w#misc#hide ();;


let create_ac = fun (geomap:MapCanvas.widget) ac_id config ->
  let color = Pprz.string_assoc "default_gui_color" config
  and name = Pprz.string_assoc "ac_name" config in
  let ac_menu = geomap#factory#add_submenu name in
  let ac_menu_fact = new GMenu.factory ac_menu in
  let fp = ac_menu_fact#add_check_item "Fligh Plan" ~active:false in
  ignore (fp#connect#toggled (fun () -> show_mission geomap ac_id fp#active));
  ignore (ac_menu_fact#add_check_item "Datalink Settings" ~callback:(active_dl_settings ac_id));
  
  let track = new MapTrack.track ~name ~color:color geomap in

  ignore (ac_menu_fact#add_item "Center A/C" ~callback:(center geomap track));
  
  let eb = GBin.event_box ~width:10 ~height:10 () in
  eb#coerce#misc#modify_bg [`NORMAL, `NAME color];
  let _col_menu = ac_menu_fact#add_image_item ~label:"Color" ~image:eb#coerce ~callback:(fun () -> () (*** TO FIX colorsel track eb#coerce***) ) () in
  ignore (ac_menu_fact#add_item "Clear Track" ~callback:(fun () -> track#clear_map2D));
  ignore (ac_menu_fact#add_item "Resize Track" ~callback:(fun () -> resize_track ac_id track));
    ignore (ac_menu_fact#add_item "Commit Moves" ~callback:(fun () -> commit_changes ac_id));
  ignore (ac_menu_fact#add_item "Event 1" ~callback:(fun () -> send_event ac_id 1));
  ignore (ac_menu_fact#add_item "Event 2" ~callback:(fun () -> send_event ac_id 2));
  let cam = ac_menu_fact#add_check_item "Cam Display" ~active:false in
  ignore (cam#connect#toggled (fun () -> track#set_cam_state cam#active));
  let params = ac_menu_fact#add_check_item "flight param. display" ~active:false in
  ignore (params#connect#toggled (fun () -> track#set_params_state params#active));
  let event_ac = fun e ->
    match e with
      `BUTTON_PRESS _ | `BUTTON_RELEASE _ -> 
	Ground_Pprz.message_send "ground" "SELECTED" ["aircraft_id", Pprz.String ac_id];
	true
    | _ -> false in
  ignore (track#aircraft#connect#event event_ac);
  let fp_url = Pprz.string_assoc "flight_plan" config in
  let fp_file = Http.file_of_url fp_url in
  let fp_xml_dump = Xml.parse_file fp_file in
  let fp_xml = ExtXml.child fp_xml_dump "flight_plan" in

  let ds = dl_settings ac_id fp_xml in

  let fp = load_mission color geomap fp_xml in
  fp#hide ();
  Hashtbl.add live_aircrafts ac_id { track = track; color = color; fp_group = fp ; config = config ; fp = fp_xml; dl_settings = ds}
  



let ask_config = fun geomap ac ->
  let get_config = fun _sender values ->
    create_ac geomap ac values
  in
  Ground_Pprz.message_req "map2d" "CONFIG" ["ac_id", Pprz.String ac] get_config

	 

let one_new_ac = fun (geomap:MapCanvas.widget) ac ->
  if not (Hashtbl.mem live_aircrafts ac) then begin
     ask_config geomap ac
  end
      
      
let live_aircrafts_msg = fun (geomap:MapCanvas.widget) acs ->
  let acs = Pprz.string_assoc "ac_list" acs in
  let acs = Str.split list_separator acs in
  List.iter (one_new_ac geomap) acs


let listen_flight_params = fun () ->
  let get_fp = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      let wgs84 = { posn_lat = (Deg>>Rad)(a "lat"); posn_long = (Deg>>Rad)(a "long") } in
      aircraft_pos_msg ac.track wgs84 (a "course") (a "alt")  (a "speed") (a "climb")
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" get_fp);

  let get_ns = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      let wgs84 = { posn_lat = (Deg>>Rad)(a "target_lat"); posn_long = (Deg>>Rad)(a "target_long") } in
      carrot_pos_msg ac.track wgs84
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "NAV_STATUS" get_ns);

  let get_cam_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      let wgs84 = { posn_lat = (Deg>>Rad)(a "cam_lat"); posn_long = (Deg>>Rad)(a "cam_long") }
      and target_wgs84 = { posn_lat = (Deg>>Rad)(a "cam_target_lat"); posn_long = (Deg>>Rad)(a "cam_target_long") } in
      
      cam_pos_msg ac.track wgs84 target_wgs84
    with Not_found -> ()
  in ignore (Ground_Pprz.message_bind "CAM_STATUS" get_cam_status);

  let get_circle_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      let wgs84 = { posn_lat = (Deg>>Rad)(a "circle_lat"); posn_long = (Deg>>Rad)(a "circle_long") } in
      circle_status_msg ac.track wgs84 (float_of_string (Pprz.string_assoc "radius" vs)) 
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "CIRCLE_STATUS" get_circle_status);

  let get_segment_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      let geo1 = { posn_lat = (Deg>>Rad)(a "segment1_lat"); posn_long = (Deg>>Rad)(a "segment2_long") }
      and geo2 = { posn_lat = (Deg>>Rad)(a "segment2_lat"); posn_long = (Deg>>Rad)(a "segment2_long") } in
      segment_status_msg ac.track geo1 geo2
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "SEGMENT_STATUS" get_segment_status);

 let get_ap_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      ap_status_msg ac.track ( float_of_int (Pprz.int32_assoc "flight_time" vs ))
    with 
      Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "AP_STATUS" get_ap_status);;

let active_gm_http = fun x -> 
  Gm.no_http := not x

let gm_auto = ref false
let active_gm_auto = fun x -> 
  gm_auto := x


let fill_gm_tiles = fun geomap -> 
  ignore (Thread.create MapGoogle.fill_window geomap)
  
let gm_update = fun geomap ->
  if !gm_auto then fill_gm_tiles geomap

let map_from_region = fun (geomap:MapCanvas.widget) () ->
  match geomap#region with
    None -> GToolbox.message_box "Error" "Select a region first (drag left button)"
  | Some ((xw1,yw1), (xw2,yw2)) ->
      let (xc1, yc1) = geomap#canvas#w2c xw1 yw1
      and (xc2, yc2) = geomap#canvas#w2c xw2 yw2 in
      let width = xc2-xc1 and height = yc2-yc1 in
      let p = GdkPixbuf.create width height () in
      let (x0, y0) = geomap#canvas#get_scroll_offsets in
      let xc1= xc1 - x0 and yc1 = yc1 - y0 in
      GdkPixbuf.get_from_drawable ~dest:p ~width ~height ~src_x:xc1 ~src_y:yc1 geomap#canvas#misc#window;
      match GToolbox.select_file ~filename:(default_path_maps//".xml") ~title:"Save region map" () with
	None -> ()
      | Some xml_file  ->
	  let jpg = Filename.chop_extension xml_file ^ ".png" in
	  GdkPixbuf.save jpg "png" p;
	  let point = fun (x,y) xyw ->
	    let wgs84 = geomap#of_world xyw in
	    Xml.Element ("point", ["x",soi x;"y",soi y;"geo", Latlong.string_of wgs84], []) in
	  let points = [point (0, 0) (xw1,yw1); point (width, height) (xw2,yw2)] in
	  let xml = Xml.Element ("map", 
				 ["file", Filename.basename jpg;
				  "projection", geomap#projection],
			       points) in
	  let f = open_out xml_file in
	  Printf.fprintf f "%s\n" (Xml.to_string_fmt xml);
	  close_out f


module Edit = struct
  (** Editing of ONE single flight plan *)
  let current_fp = ref None

  (** Wrapper checking there is currently no flight plan loaded *)
  let if_none = fun f ->
    match !current_fp with
      Some _ -> GToolbox.message_box "Error" "Only one editable flight plan at a time"
    | None -> f ()


  let load_xml_fp = fun geomap ?(xml_file=path_fps) xml ->
    set_georef_if_none geomap (georef_of_fp xml);
    let fp = new MapFP.flight_plan geomap "red" fp_dtd xml in
    fp#show_xml ();
    current_fp := Some (fp,xml_file);
    fp


  let new_fp = fun geomap () ->
    if_none (fun () ->
      let dialog = GWindow.window ~border_width:10 ~title:"New flight plan" () in
      let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
      let h = GPack.hbox ~packing:dvbx#pack () in
      let default_latlong =
	match geomap#georef with
	  None -> "WGS84 37.21098 -113.45678"
	| Some geo -> Latlong.string_of geo in
      let latlong  = labelled_entry "latlong" default_latlong h in
      let alt0 = labelled_entry "ground_alt" "380" h in
      let h = GPack.hbox ~packing:dvbx#pack () in
      let alt = labelled_entry "alt" "430" h in
      let qfu = labelled_entry "QFU" "270" h in
      let mdfh = labelled_entry "Max dist" "500" h in
      
      let h = GPack.hbox ~packing:dvbx#pack () in
      let name  = labelled_entry "Name" "Test flight" h in
      
      let h = GPack.hbox ~packing:dvbx#pack () in
      let createfp = GButton.button ~label:"Create FP" ~packing: h#add () in
      let cancel = GButton.button ~label:"Close" ~packing: h#add () in 
      ignore(cancel#connect#clicked ~callback:dialog#destroy);
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
		 ignore (load_xml_fp geomap xml);
		 dialog#destroy ()
	       end);
      dialog#show ())

	
(** Loading a flight plan for edition *)
  let load_fp = fun geomap () ->
    if_none (fun () ->
      match GToolbox.select_file ~title:"Open flight plan" ~filename:(path_fps^"*.xml") () with
	None -> ()
      | Some xml_file ->
	  let xml = Xml.parse_file xml_file in
	  ignore (load_xml_fp geomap ~xml_file xml))

  let create_wp = fun geomap geo ->
    match !current_fp with
      None -> GToolbox.message_box "Error" "Load a flight plan first"
    | Some (fp,_) ->
	ignore (fp#add_waypoint geo)

  let close_fp = fun geomap () ->
    match !current_fp with
      None -> () (* Nothing to close *)
    | Some (fp, filename) -> 
	fp#destroy ();
	current_fp := None

  let save_fp = fun geomap () ->
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

(** Calibration of chosen image *)
  let calibrate_map = fun (geomap:G.widget) () ->
    match !current_fp with
    | Some (fp,_) ->  GToolbox.message_box "Error" "Close current flight plan before calibration"
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
	    let fp = load_xml_fp geomap fp_xml in
	      
	    (** Dialog to finish calibration *)
	    let dialog = GWindow.window ~border_width:10 ~title:"Map calibration" () in
	    let v = GPack.vbox ~packing:dialog#add () in
	    let _ = GMisc.label ~text:"Choose 2 (or more) waypoints (CTRL Left Button)\nRename the waypoints with their geographic coordinates\nFor example: 'WGS84 43.123456 1.234567' or 'UTM 530134 3987652 12' or 'LBT2e 123456 543210'\nClick the button below to save the XML result file\n" ~packing:v#add () in
	    let h = GPack.hbox ~packing:v#pack () in
	    let cal = GButton.button ~label:"Calibrate" ~packing:h#add () in
	    let cancel = GButton.button ~label:"Close" ~packing:h#add () in
	    let destroy = fun () ->
	      dialog#destroy ();
	      close_fp geomap ();
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
	    dialog#show ()
end (** Edit module *)
    


let button_press = fun (geomap:MapCanvas.widget) ev ->
  let state = GdkEvent.Button.state ev in
  if GdkEvent.Button.button ev = 3 then begin
    (** Display a tile from Google Maps or IGN *)
    let xc = GdkEvent.Button.x ev 
    and yc = GdkEvent.Button.y ev in
    let (xw,yw) = geomap#window_to_world xc yc in
    
    let wgs84 = geomap#of_world (xw,yw) in
    let display = fun geo ->
      if Gdk.Convert.test_modifier `SHIFT (GdkEvent.Button.state ev) then
	MapIGN.display_tile geomap geo
      else
	try ignore (MapGoogle.display_tile geomap geo) with
	  Gm.Not_available -> () in

    ignore(Thread.create display wgs84);
    true;
  end else if GdkEvent.Button.button ev = 1 && Gdk.Convert.test_modifier `CONTROL state then begin
    let xc = GdkEvent.Button.x ev in
    let yc = GdkEvent.Button.y ev in
    let xyw = geomap#canvas#window_to_world xc yc in
    let geo = geomap#of_world xyw in
    Edit.create_wp geomap geo;
    true
  end else
    false




let _ =
  let ivy_bus = ref "127.255.255.255:2010"
  and geo_ref = ref ""
  and map_file = ref ""
  and projection= ref MapCanvas.UTM in
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.25:2010";
      "-ref", Arg.Set_string geo_ref, "Geographic ref (default '')";
      "-mercator", Arg.Unit (fun () -> projection:=MapCanvas.Mercator),"Switch to (Google Maps) Mercator projection";
      "-lambertIIe", Arg.Unit (fun () -> projection:=MapCanvas.LambertIIe),"Switch to LambertIIe projection";
      "-ign", Arg.Set_string IGN.data_path, "IGN tiles path";
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

  let window = GWindow.window ~title: "Map2d" ~border_width:1 ~width:400 () in
  let vbox= GPack.vbox ~packing: window#add () in

  let vertical_situation = GWindow.window ~title: "Vertical" ~border_width:1 ~width:400 () in
  let _vertical_vbox= GPack.vbox ~packing: vertical_situation#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  ignore (vertical_situation#connect#destroy ~callback:quit);

  let geomap = new MapCanvas.widget ~projection:!projection ~height:400 () in
  let accel_group = geomap#menu_fact#accel_group in

  ignore (geomap#canvas#event#connect#button_press (button_press geomap));

  (** widget displaying aircraft vertical position  *)

  let _active_vertical = fun x ->
    if x then vertical_situation#show () else vertical_situation#misc#hide () in
  ignore (geomap#menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);

  (* Maps handling *)
  let map_menu = geomap#factory#add_submenu "Maps" in
  let map_menu_fact = new GMenu.factory ~accel_group map_menu in
  ignore (map_menu_fact#add_item "Load" ~key:GdkKeysyms._M ~callback:(load_map geomap));
  ignore (map_menu_fact#add_item "Calibrate" ~key:GdkKeysyms._C ~callback:(Edit.calibrate_map geomap));
  ignore (map_menu_fact#add_item "GM Fill" ~key:GdkKeysyms._G ~callback:(fun _ -> fill_gm_tiles geomap));
  ignore (map_menu_fact#add_check_item "GM Http" ~key:GdkKeysyms._H ~active:true ~callback:active_gm_http);
  ignore (map_menu_fact#add_check_item "GM Auto" ~active:false ~callback:active_gm_auto);
  ignore (map_menu_fact#add_item "Map of region" ~key:GdkKeysyms._R ~callback:(map_from_region geomap));
 
  (** Connect Google Maps display to view change *)
  geomap#connect_view (fun () -> gm_update geomap);

  
  (** Flight plan editing *)
  let fp_menu = geomap#factory#add_submenu "Edit" in
  let fp_menu_fact = new GMenu.factory ~accel_group fp_menu in
  ignore (fp_menu_fact#add_item "New flight plan" ~key:GdkKeysyms._N ~callback:(Edit.new_fp geomap));
  ignore (fp_menu_fact#add_item "Open flight plan" ~key:GdkKeysyms._O ~callback:(Edit.load_fp geomap));
  ignore (fp_menu_fact#add_item "Save flight plan" ~key:GdkKeysyms._S ~callback:(Edit.save_fp geomap));
  ignore (fp_menu_fact#add_item "Close flight plan" ~callback:(Edit.close_fp geomap));

  (** Separate from A/C menus *)
  ignore (geomap#factory#add_separator ());

  vbox#pack ~expand:true geomap#frame#coerce;

  (* Loading an initial map *)
  if !geo_ref <> "" then
    set_georef_if_none geomap (Latlong.of_string !geo_ref);
  if !map_file <> "" then begin
    let xml_map_file = if !map_file.[0] <> '/' then Filename.concat default_path_maps !map_file else !map_file in
    display_map geomap xml_map_file
  end;

  ignore (Glib.Timeout.add 2000 (fun () -> Ground_Pprz.message_req "map2d" "AIRCRAFTS" [] (fun _sender vs -> live_aircrafts_msg geomap vs); false));

  ignore (Ground_Pprz.message_bind "NEW_AIRCRAFT" (fun _sender vs -> one_new_ac geomap (Pprz.string_assoc "ac_id" vs)));

  listen_flight_params ();

  window#add_accel_group accel_group;
  window#show ();
 
  GtkThread.main ()
