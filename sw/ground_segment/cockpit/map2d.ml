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

let google_maps_url = fun s -> sprintf "http://kh1.google.com/kh?n=404&v=3&t=%s" s



type aircraft = {
    config : Pprz.values;
    track : MapTrack.track;
    color: color;
    mutable fp_group : (MapWaypoints.group * (int * MapWaypoints.waypoint) list) option;
    fp : Xml.xml;
    dl_settings : GWindow.window
  }

let live_aircrafts = Hashtbl.create 3

let map_ref = ref None

let float_attr = fun xml a -> float_of_string (ExtXml.attrib xml a)

let load_map = fun (geomap:G.widget) (vertical_display:MapCanvas.basic_widget) xml_map ->
  let dir = Filename.dirname xml_map in
  let xml_map = Xml.parse_file xml_map in
  let image = dir // ExtXml.attrib xml_map "file"
  and scale = float_attr xml_map "scale"
  and utm_zone =
    try int_of_string (Xml.attrib xml_map "utm_zone") with
      _ -> 31 in
  geomap#set_world_unit scale;
  approx_ground_altitude := ( try (float_attr xml_map "approx_ground_altitude") 
      with _ -> 0.0);
  vertical_display#set_world_unit scale;
  let one_ref = ExtXml.child xml_map "point" in
  let x = float_attr one_ref "x" and y = float_attr one_ref "y"
  and utm_x = float_attr one_ref "utm_x" and utm_y = float_attr one_ref "utm_y" in
  let utm_x0 = utm_x -. x *. scale
  and utm_y0 = utm_y +. y *. scale in

  let utm_ref =
    match !map_ref with
      None ->
	let utm0 = {utm_x = utm_x0;  utm_y = utm_y0; utm_zone = utm_zone } in
	map_ref := Some utm0;
	utm0
    | Some utm ->
	assert (utm_zone = utm.utm_zone);
	utm in

  let wgs84_of_en = fun en ->
    of_utm WGS84 {utm_x = utm_ref.utm_x +. en.G.east; utm_y = utm_ref.utm_y +. en.G.north; utm_zone = utm_zone} in

  geomap#set_wgs84_of_en wgs84_of_en;
  let en0 = {G.east=utm_x0 -. utm_ref.utm_x; north=utm_y0 -. utm_ref.utm_y} in
  ignore (geomap#display_map en0 (GdkPixbuf.from_file image));
  geomap#moveto en0


let set_geo_ref = fun geomap wgs84 ->
  let utm_ref = utm_of WGS84 wgs84 in
  let wgs84_of_en = fun en ->
    of_utm WGS84 {utm_x = utm_ref.utm_x +. en.G.east; utm_y = utm_ref.utm_y +. en.G.north; utm_zone = utm_ref.utm_zone} in

  geomap#set_wgs84_of_en wgs84_of_en;
  geomap#set_world_unit 1.;
  assert (!map_ref = None);
  map_ref := Some utm_ref


exception Wget_failure of string

let file_of_url = fun ?(extension=".xml") ?dest url ->
  if String.sub url 0 7 = "file://" then
    String.sub url 7 (String.length url - 7)
  else
    let tmp_file =
      match dest with 
	Some s -> s
      | None -> Filename.temp_file "fp" extension in
    let c = sprintf "wget --cache=off -O %s '%s'" tmp_file url in
    if Sys.command c = 0 then
      tmp_file
    else begin
      Sys.remove tmp_file;
      raise (Wget_failure url)
    end

let load_mission = fun color geomap xml ->
  let lat0 = float_attr xml "lat0"
  and lon0 = float_attr xml "lon0"
  and alt0 = float_attr xml "alt" in
  let utm0 = utm_of WGS84 {posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 } in
  let waypoints = ExtXml.child xml "waypoints" in
  let max_dist_from_home = float_attr xml "MAX_DIST_FROM_HOME" in
  
  let utm_ref =
    match !map_ref with
      None ->
	map_ref := Some utm0;
	utm0
    | Some utm ->
	assert (utm0.utm_zone = utm.utm_zone);
	utm in
  let en_of_xy = fun x y ->
    {G.east = x +. utm0.utm_x -. utm_ref.utm_x;
     G.north = y +. utm0.utm_y -. utm_ref.utm_y } in

  let fp = new MapWaypoints.group ~color ~editable:true geomap in
  let i = ref 0 in
  let wpts = List.map
      (fun wp ->
	let en = en_of_xy (float_attr wp "x") (float_attr wp "y") in
	let alt = try float_attr wp "alt" with _ -> alt0 in
	let w = MapWaypoints.waypoint fp ~name:(ExtXml.attrib wp "name") ~alt en in
	if  ExtXml.attrib wp "name" = "HOME" then
	  ignore (geomap#circle ~color en max_dist_from_home);
	incr i;
	!i, w
      ) 
      (Xml.children waypoints) in
  fp, wpts


let aircraft_pos_msg = fun track utm_x_ utm_y_ heading altitude speed climb ->
  match !map_ref with
    None -> ()
  | Some utm0 ->
      let en =  {G.east = utm_x_ -. utm0.utm_x; north = utm_y_ -. utm0.utm_y } in
      let h = 
	try
	  Srtm.of_utm { utm_zone = utm0.utm_zone; utm_x = utm_x_; utm_y = utm_y_}
	with
	  _ -> truncate altitude
      in
      track#move_icon en heading altitude (float_of_int h) speed climb

let carrot_pos_msg = fun track utm_x utm_y ->
  match !map_ref with
    None -> ()
  | Some utm0 ->
      let en =  {G.east = utm_x -. utm0.utm_x; north = utm_y -. utm0.utm_y } in
      track#move_carrot en

let cam_pos_msg = fun track utm_x utm_y target_utm_x target_utm_y ->
  match !map_ref with
    None -> ()
  | Some utm0 ->
      let en =  {G.east = utm_x -. utm0.utm_x; north = utm_y -. utm0.utm_y } in
      let target_en =  {G.east = target_utm_x -. utm0.utm_x; north = target_utm_y -. utm0.utm_y } in  
      track#move_cam en target_en

let circle_status_msg = fun track utm_x utm_y radius ->
  match !map_ref with
    None -> ()
  | Some utm0 ->
      let en =  {G.east = utm_x -. utm0.utm_x; north = utm_y -. utm0.utm_y } in  
      track#draw_circle en radius

let segment_status_msg = fun track utm_x utm_y utm2_x utm2_y ->
  match !map_ref with
    None -> ()
  | Some utm0 ->
      let en =  {G.east = utm_x -. utm0.utm_x; north = utm_y -. utm0.utm_y } in  
      let en2 =  {G.east = utm2_x -. utm0.utm_x; north = utm2_y -. utm0.utm_y } in
      track#draw_segment en en2

let circle_status_msg = fun track utm_x utm_y radius ->
  match !map_ref with
    None -> ()
  | Some utm0 ->
      let en =  {G.east = utm_x -. utm0.utm_x; north = utm_y -. utm0.utm_y } in  
      track#draw_circle en radius

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
  match a.fp_group, !map_ref with
    Some (g, wpts), Some utm0 -> 
      List.iter 
	(fun (i, w) -> 
	  if w#moved then 
	    let {MapCanvas.east=e; MapCanvas.north=n} =w#en in
	    let vs = ["ac_id", Pprz.String ac;
		      "wp_id", Pprz.Int i;
		      "utm_east", Pprz.Float (utm0.utm_x+.e);
		      "utm_north", Pprz.Float (utm0.utm_y+.n);
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


let active_dl_settings = fun ac_id x ->
  let ac = Hashtbl.find live_aircrafts ac_id in
  let w = ac.dl_settings in
  if x then w#show () else w#misc#hide ();;


let create_ac = fun (geomap:MapCanvas.widget) (vertical_display:MapCanvas.basic_widget) ac_id config ->
  let color = Pprz.string_assoc "default_gui_color" config
  and name = Pprz.string_assoc "ac_name" config in
  let ac_menu = geomap#factory#add_submenu name in
  let ac_menu_fact = new GMenu.factory ac_menu in
  let fp = ac_menu_fact#add_check_item "Fligh Plan" ~active:false in
  ignore (fp#connect#toggled (fun () -> show_mission geomap ac_id fp#active));
  ignore (ac_menu_fact#add_check_item "Datalink Settings" ~callback:(active_dl_settings ac_id));
  
  let track = new MapTrack.track ~name ~color:color geomap vertical_display in
  
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
  let ac_menu_vertical = vertical_display#factory#add_submenu ac_id in
  let ac_menu_fact_vertical = new GMenu.factory ac_menu_vertical in
  let params = ac_menu_fact#add_check_item "flight param. display" ~active:false in
  ignore (params#connect#toggled (fun () -> track#set_params_state params#active));
  let v_params = ac_menu_fact_vertical#add_check_item "flight param. display" ~active:false in
  ignore (v_params#connect#toggled (fun () -> track#set_v_params_state v_params#active));
  let event_ac = fun e ->
    match e with
      `BUTTON_PRESS _ | `BUTTON_RELEASE _ -> 
	Ground_Pprz.message_send "ground" "SELECTED" ["aircraft_id", Pprz.String ac_id];
	true
    | _ -> false in
  ignore (track#aircraft#connect#event event_ac);
  let fp_url = Pprz.string_assoc "flight_plan" config in
  let fp_file = file_of_url fp_url in
  let fp_xml_dump = Xml.parse_file fp_file in
  let fp_xml = ExtXml.child fp_xml_dump "flight_plan" in

  let ds = dl_settings ac_id fp_xml in

  Hashtbl.add live_aircrafts ac_id { track = track; color = color; fp_group = None ; config = config ; fp = fp_xml; dl_settings = ds}
  



let ask_config = fun geomap vd ac ->
  let get_config = fun _sender values ->
    create_ac geomap vd ac values
  in
  Ground_Pprz.message_req "map2d" "CONFIG" ["ac_id", Pprz.String ac] get_config

	 

let one_new_ac = fun (geomap:MapCanvas.widget)(vertical_display:MapCanvas.basic_widget) ac ->
  if not (Hashtbl.mem live_aircrafts ac) then begin
     ask_config geomap vertical_display ac
  end
      
      
let live_aircrafts_msg = fun (geomap:MapCanvas.widget) (vertical_display:MapCanvas.basic_widget) acs ->
  let acs = Pprz.string_assoc "ac_list" acs in
  let acs = Str.split list_separator acs in
  List.iter (one_new_ac geomap vertical_display) acs


let listen_flight_params = fun () ->
  let get_fp = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      aircraft_pos_msg ac.track (a "east") (a "north") (a "course") (a "alt")  (a "speed") (a "climb")
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" get_fp);

  let get_ns = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
	carrot_pos_msg ac.track (a "target_east") (a "target_north") 
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "NAV_STATUS" get_ns);

  let get_cam_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      cam_pos_msg ac.track (a "cam_east") (a "cam_north") (a "target_east") (a "target_north")
    with Not_found -> ()
  in ignore (Ground_Pprz.message_bind "CAM_STATUS" get_cam_status);

  let get_circle_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      circle_status_msg ac.track (a "circle_east") (a "circle_north") (float_of_string (Pprz.string_assoc "radius" vs)) 
    with Not_found -> ()
  in
  ignore (Ground_Pprz.message_bind "CIRCLE_STATUS" get_circle_status);

  let get_segment_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    try
      let ac = Hashtbl.find live_aircrafts ac_id in
      let a = fun s -> Pprz.float_assoc s vs in
      segment_status_msg ac.track (a "segment1_east") (a "segment1_north") (a "segment2_east") (a "segment2_north") 
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

let en_of_wgs84 = fun geomap wgs84 ->
  let ref = geomap#wgs84_of_en {G.east=0.; north = 0.} in
  let utm_ref = utm_of WGS84 ref in

  let utm = utm_of WGS84 wgs84 in
  {G.east=utm.utm_x -. utm_ref.utm_x; north=utm.utm_y -. utm_ref.utm_y}


let is_prefix = fun a b ->
  String.length b >= String.length a &&
  a = String.sub b 0 (String.length a)

let get_gm_from_cache = fun f ->
  let files = Sys.readdir gm_tiles_path in
  let rec loop = fun i ->
    if i < Array.length files then
      let fi = files.(i) in
      if is_prefix (Filename.chop_extension fi) f then
	let (sw, s) = Latlong.gm_lat_long_of_tile f in
	(gm_tiles_path // fi, sw, s)
      else
	loop (i+1)
    else
      raise Not_found in
  loop 0


let rec get_gm_tile = fun wgs84 zoom ->
  if zoom < 10 then
    try
      let (gm_string, sw, scale) = Latlong.gm_tile_string wgs84 zoom in
      try get_gm_from_cache gm_string with
	Not_found ->
	  let url = google_maps_url gm_string in
	  let jpg_file = gm_tiles_path // (gm_string ^ ".jpg") in
	  ignore (file_of_url ~extension:".jpg" ~dest:jpg_file url);
	  (jpg_file, sw, scale)
    with (** Error, let's try a lower zoom *)
      Wget_failure __ -> get_gm_tile wgs84 (zoom+1)
  else
    failwith "download_gm_tile"




let button_press = fun (geomap:MapCanvas.widget) ev ->
  let xc = GdkEvent.Button.x ev 
  and yc = GdkEvent.Button.y ev in
  let (xw, yw) = geomap#window_to_world xc yc in
  let en = geomap#en_of_world xw yw in
  let wgs84 = geomap#wgs84_of_en en in

  let (jpg_file, sw, scale) = get_gm_tile wgs84 1 in

  let en_sw = en_of_wgs84 geomap sw in
  let en_nw = { en_sw with G.north = en_sw.G.north +. 256. *. scale } in
  let scale = scale /. geomap#get_world_unit () in
  let map = geomap#display_map ~scale en_nw (GdkPixbuf.from_file jpg_file) in
  map#raise 1;
  true
  


let _ =
  let ivy_bus = ref "127.255.255.255:2010"
  and geo_ref = ref ""
  and map_file = ref ""
  and mission_file = ref "" in
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.25:2010";
      "-ref", Arg.Set_string geo_ref, "Geographic ref (default '')";
      "-m", Arg.String (fun x -> map_file := x), "Map description file"] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: ";
  (*                                 *)
  Ivy.init "Paparazzi map 2D" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  Srtm.add_path default_path_srtm;

  let window = GWindow.window ~title: "Map2d" ~border_width:1 ~width:400 () in
  let vbox= GPack.vbox ~packing: window#add () in
  let vertical_situation = GWindow.window ~title: "Vertical" ~border_width:1 ~width:400 () in
  let vertical_vbox= GPack.vbox ~packing: vertical_situation#add () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);
  ignore (vertical_situation#connect#destroy ~callback:quit);

  let geomap = new MapCanvas.widget ~height:400 () in
  let accel_group = geomap#menu_fact#accel_group in

  ignore (geomap#canvas#event#connect#button_press (button_press geomap));

  (** widget displaying aircraft vertical position  *)
  let vertical_display = new MapCanvas.basic_widget ~height:400 () in
  let ac_vertical_fact = new GMenu.factory vertical_display#file_menu in
  let time_axis = ac_vertical_fact#add_check_item "x_axis : Time" ~active:false in
    ignore (time_axis#connect#toggled (fun () ->
      let set_one_track = (fun a b -> 
	(b.track)#set_vertical_time_axis time_axis#active) in 
      Hashtbl.iter (set_one_track) live_aircrafts));
  let vertical_graduations = GnoCanvas.group vertical_display#canvas#root in
  vertical_display#set_vertical_factor 10.0;

  let active_vertical = fun x ->
    if x then vertical_situation#show () else vertical_situation#misc#hide () in
 
 
  ignore (geomap#menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);
  ignore (geomap#menu_fact#add_check_item "Vertical View" ~key:GdkKeysyms._V ~callback:active_vertical);
 

  vbox#pack ~expand:true geomap#frame#coerce;
  vertical_vbox#pack ~expand:true vertical_display#frame#coerce;

  (* Loading an initial map *)
  if !geo_ref <> "" then
    set_geo_ref geomap (Latlong.of_string !geo_ref)
  else if !map_file <> "" then begin
    let xml_map_file = if !map_file.[0] <> '/' then Filename.concat default_path_maps !map_file else !map_file in
    load_map geomap vertical_display xml_map_file
  end;

  let max_level = (float_of_int max_graduations) *. vertical_delta +. !approx_ground_altitude in 
   vertical_display#set_vertical_max_level max_level;
  for i = 0 to max_graduations do
    let level = (float_of_int i) *. vertical_delta in    
    ignore ( vertical_display#segment ~group:vertical_graduations ~fill_color:"blue" {G.east = 0.0 ; G.north = level *. (-. vertical_display#get_vertical_factor) } {G.east = max_east ; G.north =  level *. (-. vertical_display#get_vertical_factor) } ) ;
    for j = 0 to max_label do
    ignore( vertical_display#text ~group:vertical_graduations ~fill_color:"red" ~x_offset:30.0 ~y_offset:(-.0.5) {G.east = (float_of_int j) *. max_east /. (float_of_int max_label) ; G.north = level *. (-. vertical_display#get_vertical_factor) } ((string_of_float ( max_level -. level) )^" m") )
    done;
  done;   

  ignore (Glib.Timeout.add 2000 (fun () -> Ground_Pprz.message_req "map2d" "AIRCRAFTS" [] (fun _sender vs -> live_aircrafts_msg geomap vertical_display vs); false));

  ignore (Ground_Pprz.message_bind "NEW_AIRCRAFT" (fun _sender vs -> one_new_ac geomap vertical_display (Pprz.string_assoc "ac_id" vs)));

  listen_flight_params ();

  window#add_accel_group accel_group;
  window#show ();
 
  GMain.Main.main ()
