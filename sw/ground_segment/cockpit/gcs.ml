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

module G = MapCanvas
open Printf
open Latlong

let soi = string_of_int

let home = Env.paparazzi_home
let (//) = Filename.concat
let default_path_srtm = home // "data" // "srtm"
let default_path_maps = home // "data" // "maps"
let layout_path = home // "conf" // "gcs"
let var_maps_path = home // "var" // "maps"
let _ = 
  ignore (Sys.command (sprintf "mkdir -p %s" var_maps_path))

let ign = ref false
let get_bdortho = ref ""
let auto_center_new_ac = ref false
let no_alarm = ref false


(** Display a calibrated (XML) map *)
let display_map = fun (geomap:G.widget) xml_map ->
  try
    let dir = Filename.dirname xml_map in
    let xml_map = ExtXml.parse_file xml_map in
    let image = dir // ExtXml.attrib xml_map "file" in
    let map_projection = Xml.attrib xml_map "projection" in
    let opacity = try Some (int_of_string (Xml.attrib xml_map "opacity")) with _ -> None in
    let current_projection = geomap#projection in
    if map_projection <> current_projection then
      GToolbox.message_box "Warning" (sprintf "You are loading a map in %s projection while the display use %s" map_projection current_projection);

    let pix_ref = fun p ->
      truncate (ExtXml.float_attrib p "x"), truncate (ExtXml.float_attrib p "y") in
    let geo_ref = fun p ->
      try Latlong.of_string (Xml.attrib p "geo") with
	_ -> (* Compatibility with the old UTM format *)
	  let utm_x = ExtXml.float_attrib p "utm_x"
	  and utm_y = ExtXml.float_attrib p "utm_y" in
	  let utm_zone = ExtXml.int_attrib xml_map "utm_zone" in
	  let utm = {utm_x = utm_x;  utm_y = utm_y; utm_zone = utm_zone } in
	  Latlong.of_utm WGS84 utm in
    
    match Xml.children xml_map with
      p1::p2::_ ->
	let x1y1 = pix_ref p1
	and x2y2 = pix_ref p2
	and geo1 = geo_ref p1
	and geo2 = geo_ref p2 in
	
	(* Take this point as a reference for the display if none currently *)
	Map2d.set_georef_if_none geomap geo1;
	
	ignore (geomap#display_pixbuf ?opacity ((x1y1),geo1) ((x2y2),geo2) (GdkPixbuf.from_file image));
	geomap#center geo1
    | _ -> failwith (sprintf "display_map: two ref points required")
  with
    Xml.File_not_found f ->
      GToolbox.message_box "Error" (sprintf "File does not exist: %s" f)
  | ExtXml.Error s ->
      GToolbox.message_box "Error" (sprintf "Error in XML file: %s" s)
	


let load_map = fun (geomap:G.widget) () ->
  match GToolbox.select_file ~title:"Open Map" ~filename:(default_path_maps // "*.xml") () with
    None -> ()
  | Some f -> display_map geomap f



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


(************ Google Maps handling *****************************************)
module GM = struct
  let active_http = fun x -> 
    Gm.no_http := not x

  (** Fill the visible background with Google tiles *)
  let fill_tiles = fun geomap -> 
    ignore (Thread.create MapGoogle.fill_window geomap)

  let auto = ref false
  let update = fun geomap ->
    if !auto then fill_tiles geomap
  let active_auto = fun geomap x -> 
    auto := x;
    update geomap      

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

let bdortho_size = 400
let bdortho_store = Hashtbl.create 97
let display_bdortho = fun  (geomap:G.widget) wgs84 () ->
  let r = bdortho_size / 2 in
  let { lbt_x = lx; lbt_y = ly} = lambertIIe_of wgs84 in
  let lx = lx + r and ly = ly + bdortho_size/2 in
  let lx = lx - (lx mod bdortho_size)
  and ly = ly - (ly mod bdortho_size) in
  let f = sprintf "ortho_%d_%d_%d.jpg" lx ly r in
  let f = var_maps_path // f in
  if not (Hashtbl.mem bdortho_store f) then begin
    Hashtbl.add bdortho_store f true;
    let display = fun _ ->
      let nw = of_lambertIIe {lbt_x = lx - r; lbt_y = ly + r}
      and se = of_lambertIIe {lbt_x = lx + r; lbt_y = ly - r} in
      ignore (geomap#display_pixbuf ((0,0), nw) ((bdortho_size, bdortho_size), se)  (GdkPixbuf.from_file f));
      
    in
    if Sys.file_exists f then
      display f
    else
      ignore (Thread.create
		(fun f ->
		  let c = sprintf "%s %d %d %d %s" !get_bdortho lx ly r f in
		  ignore (Sys.command c);
		  display f)
		f)
  end


let fill_ortho = fun (geomap:G.widget) ->
  (** First estimate the coverage of the window *)
  let width_c, height_c = Gdk.Drawable.get_size geomap#canvas#misc#window
  and (xc0, yc0) = geomap#canvas#get_scroll_offsets in
  let (xw0, yw0) = geomap#window_to_world (float xc0) (float (yc0+height_c))
  and (xw1, yw1) = geomap#window_to_world (float (xc0+width_c)) (float yc0) in
  let sw = geomap#of_world (xw0, yw0)
  and ne = geomap#of_world (xw1, yw1) in
  let lbt2e_sw = lambertIIe_of sw
  and lbt2e_ne = lambertIIe_of ne in
  let w = lbt2e_ne.lbt_x - lbt2e_sw.lbt_x
  and h = lbt2e_ne.lbt_y - lbt2e_sw.lbt_y in
  for i = 0 to w / bdortho_size + 1 do
    let lbt_x = lbt2e_sw.lbt_x + bdortho_size * i in
    for j = 0 to h / bdortho_size + 1 do
      let lbt_y = lbt2e_sw.lbt_y + bdortho_size * j in
      let geo = of_lambertIIe {lbt_x = lbt_x; lbt_y = lbt_y } in
      display_bdortho geomap geo ()
    done
  done
    

    

(******* Mouse motion handling **********************************************)
let motion_notify = fun (geomap:G.widget) ev ->
  let xc = GdkEvent.Motion.x ev 
  and yc = GdkEvent.Motion.y ev in
  let xwyw = geomap#window_to_world xc yc in
  EditFP.path_notify geomap xwyw


(******* Mouse wheel handling ***********************************************)
let any_event = fun (geomap:G.widget) ev ->
  match GdkEvent.get_type ev with
    `SCROLL ->
      let state = GdkEvent.Scroll.state (GdkEvent.Scroll.cast ev) in
      if Gdk.Convert.test_modifier `CONTROL state && Gdk.Convert.test_modifier `SHIFT state then 
	let scroll_event = GdkEvent.Scroll.cast ev in
	EditFP.path_change_radius (GdkEvent.Scroll.direction scroll_event);
	let xc = GdkEvent.Scroll.x scroll_event 
	and yc = GdkEvent.Scroll.y scroll_event in
	let xwyw = geomap#window_to_world xc yc in
	EditFP.path_notify geomap xwyw
      else
	false
  | _ ->
      false



(******* Mouse buttons handling **********************************************)
let button_press = fun (geomap:G.widget) ev ->
  let state = GdkEvent.Button.state ev in
  if GdkEvent.Button.button ev = 3 && Gdk.Convert.test_modifier `CONTROL state && Gdk.Convert.test_modifier `SHIFT state then begin
    EditFP.path_close ();
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

    let m = if !ign then [`I ("Load IGN tile", display_ign)] else [] in
    let m =
      if !get_bdortho <> "" then
	(`I ("Load BDORTHO", display_bdortho geomap wgs84))::m
      else 
	m in

    GToolbox.popup_menu ~entries:([`I ("Load Google tile", display_gm)]@m)
      ~button:3 ~time:(Int32.of_int 00);
    true;
  end else if GdkEvent.Button.button ev = 1 && Gdk.Convert.test_modifier `CONTROL state then
    if Gdk.Convert.test_modifier `SHIFT state then begin
      let xc = GdkEvent.Button.x ev in
      let yc = GdkEvent.Button.y ev in
      let xyw = geomap#canvas#window_to_world xc yc in
      EditFP.path_button geomap xyw;
      true
    end else begin
      let xc = GdkEvent.Button.x ev in
      let yc = GdkEvent.Button.y ev in
      let xyw = geomap#canvas#window_to_world xc yc in
      let geo = geomap#of_world xyw in
      ignore (EditFP.create_wp geomap geo);
      true
    end else 
    false






(******** Help ***************************************************************)
let keys_help = fun () ->
  GToolbox.message_box ~title:"Keys" ~ok:"Close"
    "Zoom: Mouse Wheel, PgUp, PgDown\n\
    Pan: Map & keyboard arrows\n\
    Fit to window: f\n\
    Load Map Tile: Right\n\
    Select Region: Left + Drag\n\
    Create Waypoint: Ctrl-Left\n\
    Create Path Point: Ctrl-Shift-Left\n\
    Change Path Radius: Wheel\n\
    Move Waypoint: Left drag\n\
    Edit Waypoint: Left click\n"



(***************** MAIN ******************************************************)
let ivy_bus = ref "127.255.255.255:2010"
and geo_ref = ref ""
and map_files = ref []
and center = ref ""
and zoom = ref 1.
and maximize = ref false
and fullscreen = ref false
and projection = ref G.Mercator
and auto_ortho = ref false
and mplayer = ref ""
and plugin_window = ref ""
and layout_file = ref "horizontal.xml"
and edit = ref false
and display_particules = ref false

let options =
  [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.25:2010";
    "-maximize", Arg.Set maximize, "Maximize window";
    "-fullscreen", Arg.Set fullscreen, "Fullscreen window";
    "-ref", Arg.Set_string geo_ref, "Geographic ref (default '')";
    "-zoom", Arg.Set_float zoom, "Initial zoom";
    "-center", Arg.Set_string center, "Initial map center";
    "-center_ac", Arg.Set auto_center_new_ac, "Centers the map on any new A/C";
    "-track_size", Arg.Set_int Live.track_size, (sprintf "Default track length (%d)" !Live.track_size);
    "-plugin", Arg.Set_string  plugin_window, "External X application (launched with the id of the plugin window as argument)";
    "-mplayer", Arg.Set_string mplayer, "Launch mplayer with the given argument as X plugin";
    "-utm", Arg.Unit (fun () -> projection:=G.Mercator),"Switch to UTM local projection";
    "-mercator", Arg.Unit (fun () -> projection:=G.Mercator),"Switch to (Google Maps) Mercator projection, default";
    "-lambertIIe", Arg.Unit (fun () -> projection:=G.LambertIIe),"Switch to LambertIIe projection";
    "-ign", Arg.String (fun s -> ign:=true; IGN.data_path := s), "IGN tiles path";
    "-ortho", Arg.Set_string get_bdortho, "IGN tiles path";
    "-edit", Arg.Unit (fun () -> edit := true; layout_file := "editor.xml"), "Flight plan editor";
    "-layout", Arg.Set_string layout_file, "XML layout specification";
    "-no_alarm", Arg.Set no_alarm, "Disables alarm page";
    "-auto_ortho", Arg.Set auto_ortho, "IGN tiles path";
    "-google_fill", Arg.Set GM.auto, "Google maps auto fill";
    "-speech", Arg.Set Speech.active, "Active vocal messages";
    "-particules", Arg.Set display_particules, "Display particules";
    "-m", Arg.String (fun x -> map_files := x :: !map_files), "Map description file"]


let quit = fun () ->
  match GToolbox.question_box ~title:"Leaving GCS" ~buttons:["Quit"; "Cancel"] "Do you want to quit ?" with
    1 ->
      GMain.Main.quit (); 
      exit 0
  | _ -> ()

let create_geomap = fun window editor_frame ->
  let geomap = new G.widget ~height:500 ~projection:!projection () in

  let menu_fact = new GMenu.factory geomap#file_menu in
  let accel_group = menu_fact#accel_group in

  ignore (geomap#canvas#event#connect#button_press (button_press geomap));
  ignore (geomap#canvas#event#connect#motion_notify (motion_notify geomap));
  ignore (geomap#canvas#event#connect#any (any_event geomap));

  ignore (menu_fact#add_item "Redraw" ~key:GdkKeysyms._L ~callback:(fun _ -> geomap#canvas#misc#draw None));
  let switch_fullscreen = fun x ->
    if x then
      window#fullscreen ()
    else
      window#unfullscreen () in
  ignore (menu_fact#add_check_item "Fullscreen" ~key:GdkKeysyms._F ~active: !fullscreen ~callback:switch_fullscreen);
  ignore (menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);

  (* Maps handling *)
  let map_menu = geomap#factory#add_submenu "Maps" in
  let map_menu_fact = new GMenu.factory ~accel_group map_menu in
  ignore (map_menu_fact#add_item "Load" ~key:GdkKeysyms._M ~callback:(load_map geomap));
  if !edit then
    ignore (map_menu_fact#add_item "Calibrate" ~key:GdkKeysyms._C ~callback:(EditFP.calibrate_map geomap editor_frame accel_group));
  ignore (map_menu_fact#add_item "GoogleMaps Fill" ~key:GdkKeysyms._G ~callback:(fun _ -> GM.fill_tiles geomap));
  ignore (map_menu_fact#add_check_item "GoogleMaps Http" ~key:GdkKeysyms._H ~active:true ~callback:GM.active_http);
  ignore (map_menu_fact#add_check_item "GoogleMaps Auto" ~active:!GM.auto ~callback:(GM.active_auto geomap));
  ignore (map_menu_fact#add_item "Map of Region" ~key:GdkKeysyms._R ~callback:(map_from_region geomap));
  ignore (map_menu_fact#add_item "Map of Google Tiles" ~key:GdkKeysyms._T ~callback:(GM.map_from_tiles geomap));
  ignore (map_menu_fact#add_item "Load sector" ~callback:(Sectors.load geomap));
  
  (** Connect Google Maps display to view change *)
  geomap#connect_view (fun () -> GM.update geomap);
  if !auto_ortho then
    geomap#connect_view (fun () -> fill_ortho geomap);
  
  (** Flight plan editing *)
  if !edit then begin
    let fp_menu = geomap#factory#add_submenu "Edit" in
    let fp_menu_fact = new GMenu.factory ~accel_group fp_menu in
    ignore (fp_menu_fact#add_item "New flight plan" ~key:GdkKeysyms._N ~callback:(EditFP.new_fp geomap editor_frame accel_group));
    ignore (fp_menu_fact#add_item "Open flight plan" ~key:GdkKeysyms._O ~callback:(EditFP.load_fp geomap editor_frame accel_group));
    ignore (fp_menu_fact#add_item "Save flight plan" ~key:GdkKeysyms._S ~callback:(EditFP.save_fp));
    ignore (fp_menu_fact#add_item "Close flight plan" ~key:GdkKeysyms._W ~callback:(EditFP.close_fp))
  end;

  (** Help pushed to the right *)
  let mi = GMenu.menu_item ~label:"Help" ~right_justified:true ~packing:geomap#menubar#append () in
  let help_menu = GMenu.menu () in
  GToolbox.build_menu help_menu ~entries:[`I ("Keys", keys_help)];
  mi#set_submenu help_menu;

  (** Separate from A/C menus *)
  ignore (geomap#factory#add_separator ());

  (** Set the initial soom *)
  geomap#zoom !zoom;
  geomap, menu_fact



let resize = fun (widget:GObj.widget) orientation size ->
  match size with
    Some size ->
      if orientation = `HORIZONTAL then
	widget#misc#set_size_request ~width:size ()
      else
	widget#misc#set_size_request ~height:size ()
  | None -> ()


let rec pack_widgets = fun orientation xml widgets packing ->
  let size = try Some (ExtXml.int_attrib xml "size") with _ -> None in
  match String.lowercase (Xml.tag xml) with
    "widget" ->
      let name = ExtXml.attrib xml "name" in
      let widget =
	try List.assoc name widgets with
	  Not_found -> failwith (sprintf "Unknown widget: '%s'" name)
      in
      resize widget orientation size;
      packing widget
  | "rows" ->
      let resize = match size with None -> fun _ -> () | Some width -> fun (x:GObj.widget) -> x#misc#set_size_request ~width () in
      pack_list resize `VERTICAL (Xml.children xml) widgets packing
  | "columns" -> 
      let resize = match size with None -> fun _ -> () | Some height -> fun (x:GObj.widget) -> x#misc#set_size_request ~height () in
      pack_list resize `HORIZONTAL (Xml.children xml) widgets packing
  | x -> failwith (sprintf "pack_widgets: %s" x)
and pack_list = fun resize orientation xmls widgets packing ->
  match xmls with
    [] -> ()
  | x::xs ->
      let paned = GPack.paned orientation ~show:true ~packing () in
      resize paned#coerce;
      pack_widgets orientation x widgets paned#add1;
      pack_list resize orientation xs widgets paned#add2

      
      
    

let _main =
  Arg.parse options
    (fun x -> Printf.fprintf stderr "Warning: Don't do anything with '%s'\n" x)
    "Usage: ";
  (*                                 *)
  Ivy.init "Paparazzi map 2D" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  Srtm.add_path default_path_srtm;
  Gm.cache_path := var_maps_path;
  IGN.cache_path := var_maps_path;

  let layout_file = layout_path // !layout_file in
  let layout = ExtXml.parse_file layout_file in
  let width = ExtXml.int_attrib layout "width"
  and height = ExtXml.int_attrib layout "height" in
  
  (** The whole window map2d **)
  let window = GWindow.window ~title:"Paparazzi GCS" ~border_width:1 ~width ~height () in
  if !maximize then
    window#maximize ();
  if !fullscreen then
    window#fullscreen ();
  
  ignore (window#connect#destroy ~callback:quit);

  (* Editor frame *)
  let editor_frame = GBin.frame () in

  let geomap, menu_fact = create_geomap window editor_frame in

  let map_frame = GPack.vbox () in
  (** Put the canvas in a frame *)
  map_frame#add geomap#frame#coerce;

  (** Aircraft notebook *)
  let ac_notebook = GPack.notebook ~tab_border:0 () in

  (** Alerts text frame *)
  let alert_page = GBin.frame () in
  let my_alert = new Pages.alert alert_page in

  (** plugin frame *)
  let plugin_width = 400 and plugin_height = 300 in
  let plugin_frame = GPack.vbox ~width:plugin_width () in

  let widgets = ["map2d", map_frame#coerce; 
		 "strips", Strip.scrolled#coerce;
		 "aircraft", ac_notebook#coerce;
		 "editor", editor_frame#coerce;
		 "alarms", alert_page#coerce;
	         "plugin", plugin_frame#coerce] in


  pack_widgets `HORIZONTAL (ExtXml.child layout "0") widgets window#add;

  if !mplayer <> "" then
    plugin_window := sprintf "mplayer -nomouseinput '%s' -wid " !mplayer;
  if !plugin_window <> "" then begin  
    let frame = GBin.event_box ~packing:plugin_frame#add ~width:plugin_width ~height:plugin_height () in
    let s = GWindow.socket ~packing:frame#add () in
    let com = sprintf "%s 0x%lx -geometry %dx%d" !plugin_window s#xwindow plugin_width plugin_height in

    let pid = ref None in
    let restart = fun () ->
      begin match !pid with
	None -> ()
      | Some p -> try Unix.kill p 9 with _ -> () 
      end;
      pid := Some (Unix.create_process "/bin/sh" [|"/bin/sh"; "-c"; com|] Unix.stdin Unix.stdout Unix.stderr) in

    restart ();

    ignore (menu_fact#add_item "Restart plugin" ~key:GdkKeysyms._P ~callback:restart);
    
    Plugin.frame := Some frame;
    
    let swap = fun _ ->
      (** Keep the center of the geo canvas *)
      let c = geomap#get_center () in

      let child1 = List.hd map_frame#children in
      let child2 = List.hd plugin_frame#children in
      child2#misc#reparent map_frame#coerce;
      child1#misc#reparent plugin_frame#coerce;

      (* Strange: the centering does not work if done inside this callback.
	 It is postponed to be called by the mainloop(). *)
      ignore (GMain.Idle.add (fun () -> geomap#center c; false));
    in

    let callback = fun ev ->
      Printf.printf "%d\n%!" (GdkEvent.Button.button ev);
      match GdkEvent.Button.button ev with
	1 -> swap (); true
      | 3 -> restart (); true
      | _ -> false in
      
    ignore (frame#event#connect#button_press ~callback)
  end;

  (** Wait for A/Cs and subsequent messages *)
  Live.listen_acs_and_msgs geomap ac_notebook my_alert !auto_center_new_ac;

  (** Display the window *)
  let accel_group = menu_fact#accel_group in
  window#add_accel_group accel_group;
  window#show ();

  (** Loading an initial map *)
  if !geo_ref <> "" then
    Map2d.set_georef_if_none geomap (Latlong.of_string !geo_ref);
  List.iter (fun map_file ->
    let xml_map_file = if map_file.[0] <> '/' then default_path_maps // map_file else map_file in
    display_map geomap xml_map_file)
    !map_files;

  (** Center the map as required *)
  if !center <> "" then begin
    Map2d.set_georef_if_none geomap (Latlong.of_string !center);
    geomap#center (Latlong.of_string !center)
  end;

  Speech.say "Welcome to papa ratsi";

  if !display_particules then
    Particules.listen geomap ;

  (** Threaded main loop (map tiles loaded concurently) *)
  GtkThread.main ()
