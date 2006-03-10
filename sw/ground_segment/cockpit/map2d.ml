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

let fos = float_of_string
let list_separator = Str.regexp ","

(** parameters used for creating the vertical display window *) 
let max_graduations = 20

let vertical_delta = 5.0

let max_east = 2000.0

let max_label = 4 

let approx_ground_altitude = ref 0.0

module G = MapCanvas

let home = Env.paparazzi_home
let (//) = Filename.concat
let default_path_srtm = home // "data" // "srtm"
let default_path_maps = home // "data" // "maps" // ""
let default_path_missions = home // "conf"
let gm_tiles_path = home // "var" // "maps"
let _ = 
  ignore (Sys.command (sprintf "mkdir -p %s" gm_tiles_path))



type aircraft = {
    config : Pprz.values;
    track : MapTrack.track;
    color: color;
    mutable fp_group : (MapWaypoints.group * (int * MapWaypoints.waypoint) list) option;
    fp : Xml.xml;
    dl_settings : GWindow.window
  }

let live_aircrafts = Hashtbl.create 3

let set_georef_if_none = fun geomap wgs84 ->
  match geomap#georef with
    None -> geomap#set_georef wgs84
  | Some _ -> ()
	


let load_map = fun (geomap:G.widget) xml_map ->
  let dir = Filename.dirname xml_map in
  let xml_map = Xml.parse_file xml_map in
  let image = dir // ExtXml.attrib xml_map "file"
  and scale = float_attr xml_map "scale"
  and utm_zone = int_attr xml_map "utm_zone" in
  assert (ExtXml.attrib xml_map "projection" = "UTM");

  match Xml.children xml_map with
    p1::p2::_ ->
      let x1 = truncate (float_attr p1 "x")
      and y1 = truncate (float_attr p1 "y")
      and x2 = truncate (float_attr p2 "x")
      and y2 = truncate (float_attr p2 "y")
      and utm_x1 = float_attr p1 "utm_x"
      and utm_y1 = float_attr p1 "utm_y" 
      and utm_x2 = float_attr p2 "utm_x"
      and utm_y2 = float_attr p2 "utm_y" in
      
      let utm1 = {utm_x = utm_x1;  utm_y = utm_y1; utm_zone = utm_zone }
      and utm2 = {utm_x = utm_x2;  utm_y = utm_y2; utm_zone = utm_zone } in
      let geo1 = Latlong.of_utm WGS84 utm1
      and geo2 = Latlong.of_utm WGS84 utm2 in
      
      (* Take this point as a reference for the display if none currently *)
      set_georef_if_none geomap geo1;
      
      ignore (geomap#display_pixbuf ((x1,y1),geo1) ((x2,y2),geo2) (GdkPixbuf.from_file image));
      geomap#moveto geo1
  | _ -> failwith (sprintf "load_map: two ref points required")




let load_mission = fun color geomap xml ->
  let lat0 = float_attr xml "lat0"
  and lon0 = float_attr xml "lon0"
  and alt0 = float_attr xml "alt" in
  let ref_wgs84 = {posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 } in
  let utm0 = utm_of WGS84 ref_wgs84 in
  let waypoints = ExtXml.child xml "waypoints" in
  let max_dist_from_home = float_attr xml "MAX_DIST_FROM_HOME" in
  
  set_georef_if_none geomap ref_wgs84;

  let wgs84_of_xy = fun x y ->
    Latlong.of_utm WGS84 (utm_add utm0 (x, y) ) in

  let fp = new MapWaypoints.group ~color ~editable:true geomap in
  let i = ref 0 in
  let wpts = List.map
      (fun wp ->
	let wgs84 = wgs84_of_xy (float_attr wp "x") (float_attr wp "y") in
	let alt = try float_attr wp "alt" with _ -> alt0 in
	let w = MapWaypoints.waypoint fp ~name:(ExtXml.attrib wp "name") ~alt wgs84 in
	if  ExtXml.attrib wp "name" = "HOME" then
	  ignore (geomap#circle ~color wgs84 max_dist_from_home);
	incr i;
	!i, w
      ) 
      (Xml.children waypoints) in
  fp, wpts


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
    

let display_fp = fun geomap ac ->
  try
    let ac = Hashtbl.find live_aircrafts ac in
    ac.fp_group <- Some (load_mission ac.color geomap ac.fp)
  with Failure x ->
    GToolbox.message_box ~title:"Error while loading flight plan" x


let show_mission = fun geomap ac on_off ->
  if on_off then
    display_fp geomap ac
  else
    let a = Hashtbl.find live_aircrafts ac in
    match a.fp_group with
      None -> ()
    | Some (g, _wpts) -> 
	a.fp_group <- None;
	g#group#destroy ()

let commit_changes = fun ac ->
  let a = Hashtbl.find live_aircrafts ac in
  match a.fp_group with
    Some (g, wpts) -> 
      List.iter 
	(fun (i, w) -> 
	  if w#moved then 
	    let wgs84 = w#pos in
	    let vs = ["ac_id", Pprz.String ac;
		      "wp_id", Pprz.Int i;
		      "lat", Pprz.Float ((Rad>>Deg)wgs84.posn_lat);
		      "long", Pprz.Float ((Rad>>Deg)wgs84.posn_long);
		      "alt", Pprz.Float w#alt
		    ] in
	    Ground_Pprz.message_send "map2d" "MOVE_WAYPOINT" vs)
	wpts
  | _ -> ()

let send_event = fun ac e ->
  Ground_Pprz.message_send "map2d" "SEND_EVENT" 
    ["ac_id", Pprz.String ac; "event_id", Pprz.Int e]
  


let resize_track = fun ac track ->
  match GToolbox.input_string ~text:(string_of_int track#size) ~title:ac "Track size" with
    None -> ()
  | Some s -> track#resize (int_of_string s)



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
  let col_menu = ac_menu_fact#add_image_item ~label:"Color" ~image:eb#coerce ~callback:(fun () -> () (*** TO FIX colorsel track eb#coerce***) ) () in
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

  Hashtbl.add live_aircrafts ac_id { track = track; color = color; fp_group = None ; config = config ; fp = fp_xml; dl_settings = ds}
  



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
      let a = fun s -> Pprz.string_assoc s vs in
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

let button_press = fun (geomap:MapCanvas.widget) ev ->
  if GdkEvent.Button.button ev = 3 then begin
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

    ignore(Thread.create display wgs84)
  end;
  false


let fill_gm_tiles = fun geomap -> ignore (Thread.create MapGoogle.fill_window geomap)
  

let gm_update = fun geomap ->
  if !gm_auto then fill_gm_tiles geomap

let _ =
  let ivy_bus = ref "127.255.255.255:2010"
  and geo_ref = ref ""
  and map_file = ref ""
  and mission_file = ref ""
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
  Gm.cache_path := gm_tiles_path;

  let window = GWindow.window ~title: "Map2d" ~border_width:1 ~width:400 () in
  let vbox= GPack.vbox ~packing: window#add () in

  let vertical_situation = GWindow.window ~title: "Vertical" ~border_width:1 ~width:400 () in
  let vertical_vbox= GPack.vbox ~packing: vertical_situation#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  ignore (vertical_situation#connect#destroy ~callback:quit);

  let geomap = new MapCanvas.widget ~projection:!projection ~height:400 () in
  let accel_group = geomap#menu_fact#accel_group in

  ignore (geomap#canvas#event#connect#button_press (button_press geomap));

  (** widget displaying aircraft vertical position  *)

  let active_vertical = fun x ->
    if x then vertical_situation#show () else vertical_situation#misc#hide () in
 
 
  ignore (geomap#menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);
  ignore (geomap#menu_fact#add_check_item "Vertical View" ~key:GdkKeysyms._V ~callback:active_vertical);
  ignore (geomap#menu_fact#add_item "GM Fill" ~key:GdkKeysyms._G ~callback:(fun _ -> fill_gm_tiles geomap));
  ignore (geomap#menu_fact#add_check_item "GM Http" ~key:GdkKeysyms._H ~active:true ~callback:active_gm_http);
  ignore (geomap#menu_fact#add_check_item "GM Auto" ~key:GdkKeysyms._A ~active:false ~callback:active_gm_auto);
 
  geomap#connect_view (fun () -> gm_update geomap);

  vbox#pack ~expand:true geomap#frame#coerce;

  (* Loading an initial map *)
  if !geo_ref <> "" then
    set_georef_if_none geomap (Latlong.of_string !geo_ref)
  else if !map_file <> "" then begin
    let xml_map_file = if !map_file.[0] <> '/' then Filename.concat default_path_maps !map_file else !map_file in
    load_map geomap xml_map_file
  end;

  ignore (Glib.Timeout.add 2000 (fun () -> Ground_Pprz.message_req "map2d" "AIRCRAFTS" [] (fun _sender vs -> live_aircrafts_msg geomap vs); false));

  ignore (Ground_Pprz.message_bind "NEW_AIRCRAFT" (fun _sender vs -> one_new_ac geomap (Pprz.string_assoc "ac_id" vs)));

  listen_flight_params ();

  window#add_accel_group accel_group;
  window#show ();
 
  GtkThread.main ()
