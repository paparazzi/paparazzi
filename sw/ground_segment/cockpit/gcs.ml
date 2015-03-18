(*
 * Multi aircrafts map display and flight plan editor
 *
 * Copyright (C) 2004-2009 ENAC, Pascal Brisset, Antoine Drouin
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

let locale = GtkMain.Main.init ~setlocale:false ()

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


(** This module could be inserted into Ocaml_toosl; but it requires threads.cma *)
module TodoList = struct
  (** A list of functions to call *)
  let queue = (Queue.create () : (unit -> unit) Queue.t)

  (** The id of a running thread executing the queue *)
  let doer = ref None

  (** A mutex to handle concurrent accesses *)
  let mutex = Mutex.create ()

  let rec exec_todo_list = fun todo_list ->
    Mutex.lock mutex;
    if Queue.is_empty todo_list then begin
      (** Nothing mode to do: exiting the thread *)
      doer := None;
      Mutex.unlock mutex
    end else
      (** Pick a function from the list, call it and continue *)
      let f = Queue.take queue in
      Mutex.unlock mutex;
      f ();
      exec_todo_list todo_list

  let add = fun f ->
    Mutex.lock mutex;
    (** Add the function to the queue *)
    Queue.add f queue;
    if !doer = None then
      (** Nobody is currently running the queue: start a thread *)
      doer := Some (Thread.create exec_todo_list queue);
    Mutex.unlock mutex
end


(************ Maps handling (Google, OSM, MS, etc.) ***********************************)
module GM = struct
  (** Fill the visible background with map tiles *)
  let zoomlevel = ref 20
  let fill_tiles = fun geomap ->
    match geomap#georef with
        None -> ()
      | Some _ -> TodoList.add (fun () -> MapGoogle.fill_window geomap !zoomlevel)

  let auto = ref false
  let update = fun geomap ->
    if !auto then fill_tiles geomap
  let active_auto = fun geomap x ->
    auto := x;
    update geomap

  (** Creates a calibrated map from the map tiles (selected region) *)
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
        let pix = MapGoogle.pixbuf sw ne !zoomlevel in
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
      TodoList.add
        (fun () ->
          let c = sprintf "%s %d %d %d %s" !get_bdortho lx ly r f in
          ignore (Sys.command c);
          display f)
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
let motion_notify = fun (_geomap:G.widget) _ev -> false

(******* Mouse wheel handling ***********************************************)
let any_event = fun (_geomap:G.widget) _ev -> false

(******* Mouse buttons handling **********************************************)
let button_press = fun (geomap:G.widget) ev ->
  let state = GdkEvent.Button.state ev in
  if GdkEvent.Button.button ev = 3 then begin
    (** Display a map tile from map provider (Google, OSC, ..) or IGN *)
    let xc = GdkEvent.Button.x ev
    and yc = GdkEvent.Button.y ev in
    let (xw,yw) = geomap#window_to_world xc yc in

    let wgs84 = geomap#of_world (xw,yw) in
    let display_ign = fun () ->
      TodoList.add (fun () -> MapIGN.display_tile geomap wgs84)
    and display_gm = fun () ->
      TodoList.add
        (fun () ->
          try ignore (MapGoogle.display_tile geomap wgs84 !GM.zoomlevel) with
              Gm.Not_available -> ()) in

    let m = if !ign then [`I ("Load IGN tile", display_ign)] else [] in
    let m =
      if !get_bdortho <> "" then
        (`I ("Load BDORTHO", display_bdortho geomap wgs84))::m
      else
        m in
    GToolbox.popup_menu ~entries:([`I ("Load background tile", display_gm)]@m)
      ~button:3 ~time:(Int32.of_int 0);
    true
  end else if GdkEvent.Button.button ev = 1 && Gdk.Convert.test_modifier `CONTROL state then (* create new wp on Ctrl-click *)
      let xc = GdkEvent.Button.x ev in
      let yc = GdkEvent.Button.y ev in
      let xyw = geomap#canvas#window_to_world xc yc in
      let geo = geomap#of_world xyw in
      ignore (EditFP.create_wp geomap geo);
      true
    else
      false






(******** Help ***************************************************************)
let keys_help = fun () ->
  GToolbox.message_box ~title:"Keys" ~ok:"Close"
    "Zoom: Mouse Wheel, PgUp, PgDown\n\
    Pan: Map & keyboard arrows\n\
    Fit to window: f\n\
    Center active A/C: c or C\n\
    Fullscreen: F11\n\
    Load Map Tile: Right\n\
    Create Waypoint: Ctrl-Left\n\
    Move Waypoint: Left drag\n\
    Edit Waypoint: Left click\n"



(***************** MAIN ******************************************************)
let ivy_bus = ref Defivybus.default_ivy_bus
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
and wid = ref None
and srtm = ref false
and hide_fp = ref false
and timestamp = ref false

let options =
  [
    "-auto_ortho", Arg.Set auto_ortho, "IGN tiles path";
    "-b", Arg.String (fun x -> ivy_bus := x),(sprintf "<ivy bus> Default is %s" !ivy_bus);
    "-center", Arg.Set_string center, "Initial map center (e.g. 'WGS84 43.605 1.443')";
    "-center_ac", Arg.Set auto_center_new_ac, "Centers the map on any new A/C";
    "-edit", Arg.Unit (fun () -> edit := true; layout_file := "editor.xml"), "Flight plan editor";
    "-fullscreen", Arg.Set fullscreen, "Fullscreen window";
    "-maps_fill", Arg.Set GM.auto, "Automatically start loading background maps";
    "-maps_zoom", Arg.Set_int GM.zoomlevel, "Background maps zoomlevel (default: 20, min: 18, max: 22)";
    "-ign", Arg.String (fun s -> ign:=true; IGN.data_path := s), "IGN tiles path";
    "-lambertIIe", Arg.Unit (fun () -> projection:=G.LambertIIe),"Switch to LambertIIe projection";
    "-layout", Arg.Set_string layout_file, (sprintf "<XML layout specification> GUI layout. Default: %s" !layout_file);
    "-m", Arg.String (fun x -> map_files := x :: !map_files), "Map XML description file";
    "-maximize", Arg.Set maximize, "Maximize window";
    "-mercator", Arg.Unit (fun () -> projection:=G.Mercator),"Switch to Mercator projection, default";
    "-mplayer", Arg.Set_string mplayer, "Launch mplayer with the given argument as X plugin";
    "-no_alarm", Arg.Set no_alarm, "Disables alarm page";
    "-maps_no_http", Arg.Unit (fun () -> Gm.set_policy Gm.NoHttp), "Switch off downloading of maps, always use cached maps";
    "-ortho", Arg.Set_string get_bdortho, "IGN tiles path";
    "-osm", Arg.Unit (fun () -> Gm.set_maps_source Gm.OSM), "Use OpenStreetMap database (default is Google)";
    "-ms", Arg.Unit (fun () -> Gm.set_maps_source Gm.MS), "Use Microsoft maps database (default is Google)";
    "-particules", Arg.Set display_particules, "Display particules";
    "-plugin", Arg.Set_string  plugin_window, "External X application (launched with the id of the plugin window as argument)";
    "-ref", Arg.Set_string geo_ref, "Geographic ref (e.g. 'WGS84 43.605 1.443')";
    "-speech", Arg.Set Speech.active, "Enable vocal messages";
    "-srtm", Arg.Set srtm, "Enable SRTM elevation display";
    "-track_size", Arg.Set_int Live.track_size, (sprintf "Default track length (%d)" !Live.track_size);
    "-utm", Arg.Unit (fun () -> projection:=G.UTM),"Switch to UTM local projection";
    "-wid", Arg.String (fun s -> wid := Some (Int32.of_string s)), "<window id> Id of an existing window to be attached to";
    "-zoom", Arg.Set_float zoom, "Initial zoom";
    "-auto_hide_fp", Arg.Unit (fun () -> Live.auto_hide_fp true; hide_fp := true), "Automatically hide flight plans of unselected aircraft";
    "-timestamp", Arg.Set timestamp, "Bind on timestampped telemetry messages";
  ]


let quit = fun () ->
  match GToolbox.question_box ~title:"Leaving GCS" ~buttons:["Quit"; "Cancel"] "Do you want to quit ?" with
      1 ->
        GMain.Main.quit ();
        exit 0
    | _ -> ()

let create_geomap = fun switch_fullscreen editor_frame ->
  let geomap = new G.widget ~srtm:!srtm ~height:500 ~projection:!projection () in

  let menu_fact = new GMenu.factory geomap#file_menu in
  let accel_group = menu_fact#accel_group in

  ignore (geomap#canvas#event#connect#button_press (button_press geomap));
  ignore (geomap#canvas#event#connect#motion_notify (motion_notify geomap));
  ignore (geomap#canvas#event#connect#any (any_event geomap));

  ignore (menu_fact#add_check_item "Auto hide FP" ~callback:(fun hide -> Live.auto_hide_fp hide) ~active:!hide_fp);
  ignore (menu_fact#add_item "Redraw" ~key:GdkKeysyms._L ~callback:(fun _ -> geomap#canvas#misc#draw None));
  let fullscreen = menu_fact#add_image_item ~stock:(`STOCK "gtk-fullscreen") ~callback:switch_fullscreen () in
  fullscreen#add_accelerator accel_group GdkKeysyms._F11;
  ignore (menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);

  (* Maps handling *)
  let map_menu = geomap#factory#add_submenu "Maps" in
  let map_menu_fact = new GMenu.factory ~accel_group map_menu in
  ignore (map_menu_fact#add_item "Load User Map" ~key:GdkKeysyms._M ~callback:(load_map geomap));
  if !edit then
    ignore (map_menu_fact#add_item "Calibrate" ~key:GdkKeysyms._C ~callback:(EditFP.calibrate_map geomap editor_frame accel_group));

  (* Choose the map source *)
  let maps_source_menu = map_menu_fact#add_submenu "Maps Source" in
  let maps_source_fact = new GMenu.factory maps_source_menu in
  let group = ref None in
  (* Determine a decent default selected item *)
  let active_maps_source = Gm.get_maps_source () in
  List.iter
    (fun maps_source ->
      let callback = fun b -> if b then Gm.set_maps_source maps_source in
      let active = (maps_source = active_maps_source) in
      let menu_item = maps_source_fact#add_radio_item ~group: !group ~active ~callback (Gm.string_of_maps_source maps_source) in
      group := menu_item#group)
    Gm.maps_sources;

  (* Choose the map policy *)
  let maps_policy_menu = map_menu_fact#add_submenu "Maps Policy" in
  let maps_policy_fact = new GMenu.factory maps_policy_menu in
  let group = ref None in
  (* Determine a decent default selected item *)
  let active_policy = if Gm.get_policy () = Gm.NoHttp then Gm.NoHttp
    else Gm.CacheOrHttp in
  List.iter
    (fun policy ->
      let callback = fun b -> if b then Gm.set_policy policy in
      let active = (policy = active_policy) in
      let menu_item = maps_policy_fact#add_radio_item ~group: !group ~active ~callback (Gm.string_of_policy policy) in
      group := menu_item#group)
    Gm.policies;

  (* Map tiles fill menu entry and toolbar button *)
  let callback = fun _ -> GM.fill_tiles geomap in
  ignore (map_menu_fact#add_item "Maps Fill" ~key:GdkKeysyms._G ~callback);
  let b = GButton.button ~packing:geomap#toolbar#add () in
  ignore (b#connect#clicked callback);
  let pixbuf = GdkPixbuf.from_file (Env.gcs_icons_path // "googleearth.png") in
  ignore (GMisc.image ~pixbuf ~packing:b#add ());
  let tooltips = GData.tooltips () in
  tooltips#set_tip b#coerce ~text:"Fill current view with background map tiles";

  ignore (map_menu_fact#add_check_item "Maps Auto" ~active:!GM.auto ~callback:(GM.active_auto geomap));
  ignore (map_menu_fact#add_item "Map of Region" ~key:GdkKeysyms._R ~callback:(map_from_region geomap));
  ignore (map_menu_fact#add_item "Dump map of Tiles" ~key:GdkKeysyms._T ~callback:(GM.map_from_tiles geomap));
  ignore (map_menu_fact#add_item "Load sector" ~callback:(Sectors.load geomap));

  (** Connect Maps display to view change *)
  geomap#connect_view (fun () -> GM.update geomap);
  if !auto_ortho then
    geomap#connect_view (fun () -> fill_ortho geomap);

  (** Flight plan editing *)
  if !edit then begin
    let fp_menu = geomap#factory#add_submenu "Edit" in
    let fp_menu_fact = new GMenu.factory ~accel_group fp_menu in
    ignore (fp_menu_fact#add_item "New flight plan" ~key:GdkKeysyms._N ~callback:(EditFP.new_fp geomap editor_frame accel_group));
    ignore (fp_menu_fact#add_item "Open flight plan" ~key:GdkKeysyms._O ~callback:(EditFP.load_fp geomap editor_frame accel_group));
    ignore (fp_menu_fact#add_item "Save flight plan" ~key:GdkKeysyms._S ~callback:(fun () -> EditFP.save_fp geomap));
    ignore (fp_menu_fact#add_item "Close flight plan" ~key:GdkKeysyms._W ~callback:(fun () -> EditFP.close_fp geomap))
  end;

  (** Help pushed to the right *)
  let mi = GMenu.menu_item ~label:"Help" ~right_justified:true ~packing:geomap#menubar#append () in
  let help_menu = GMenu.menu () in
  GToolbox.build_menu help_menu ~entries:[`I ("Keys", keys_help)];
  mi#set_submenu help_menu;

  (** Separate from A/C menus *)
  ignore (geomap#factory#add_separator ());

  (** Set the initial zoom *)
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

let rec find_widget_children = fun name xml ->
  let xmls = Xml.children xml in
  match String.lowercase (Xml.tag xml) with
      "widget" when ExtXml.attrib xml "name" = name -> xmls
    | "rows" | "columns" ->
      let rec loop = function
      [] -> raise Not_found
        | x::xs ->
          try find_widget_children name x with
              Not_found -> loop xs in
      loop xmls
    | _ -> raise Not_found


let rec replace_widget_children = fun name children xml ->
  let xmls = Xml.children xml
  and tag = String.lowercase (Xml.tag xml) in
  match tag with
      "widget" ->
        Xml.Element("widget",
                    Xml.attribs xml,
                    if ExtXml.attrib xml "name" = name then children else xmls)
    | "rows" | "columns" ->
      let rec loop = function
      [] -> []
        | x::xs ->
          replace_widget_children name children x :: loop xs in
      Xml.Element(tag,
                  Xml.attribs xml,
                  loop xmls)
    | _ -> xml

let rec update_widget_size = fun orientation widgets xml ->
  let get_size = fun (widget:GObj.widget) orientation ->
    let rect = widget#misc#allocation in
    if orientation = `HORIZONTAL then rect.Gtk.width else rect.Gtk.height
  in
  let xmls = Xml.children xml
  and tag = String.lowercase (Xml.tag xml) in
  match tag with
      "widget" ->
        let name = ExtXml.attrib xml "name" in
        let widget =
          try List.assoc name widgets with
              Not_found -> failwith (sprintf "Unknown widget: '%s'" name)
        in
        let size = get_size widget orientation in
        let xml = ExtXml.subst_attrib "size" (string_of_int size) xml in
        Xml.Element("widget", Xml.attribs xml, xmls)
    | "rows" ->
        Xml.Element("rows", Xml.attribs xml, List.map (update_widget_size `VERTICAL widgets) xmls)
    | "columns" ->
        Xml.Element("columns", Xml.attribs xml, List.map (update_widget_size `HORIZONTAL widgets) xmls)
    | x -> failwith (sprintf "update_widget_size: %s" x)


(* get DTD head line for layout *)
let get_layout_dtd = fun filename ->
  let gcs_regexp = Str.regexp (Filename.concat Env.paparazzi_home "conf/gcs") in
  let local_dir = Str.replace_first gcs_regexp "" (Filename.dirname filename) in
  let split = Str.split (Str.regexp Filename.dir_sep) local_dir in
  let layout = List.fold_left (fun s _ -> "../" ^ s ) "layout.dtd" split in
  sprintf "<!DOCTYPE layout SYSTEM \"%s\">" layout


let save_layout = fun filename contents ->
  let dir = Filename.dirname filename in
  let dialog = GWindow.file_chooser_dialog ~action:`SAVE ~title:"Save Layout" () in
  ignore (dialog#set_current_folder dir);
  dialog#add_filter (GFile.filter ~name:"xml" ~patterns:["*.xml"] ());
  dialog#add_button_stock `CANCEL `CANCEL ;
  dialog#add_select_button_stock `SAVE `SAVE ;
  let _ = dialog#set_current_name (Filename.basename filename) in
  begin match dialog#run (), dialog#filename with
      `SAVE, Some name ->
        dialog#destroy ();
        let f = open_out name in
        fprintf f "%s\n\n" (get_layout_dtd name);
        fprintf f "%s\n" contents;
        close_out f
    | _ -> dialog#destroy ()
  end

let listen_dropped_papgets = fun (geomap:G.widget) ->
  let dnd_targets = [ { Gtk.target = "STRING"; flags = []; info = 0 } ] in
  geomap#canvas#drag#dest_set dnd_targets ~actions:[`COPY];
  ignore (geomap#canvas#drag#connect#data_received ~callback:(Papgets.dnd_data_received geomap#still))



(************************** MAIN ********************************************)
let () =
  let file_to_edit = ref "" in
  Arg.parse options
    (fun x -> if !edit then file_to_edit := x else Printf.fprintf stderr "Warning: Don't do anything with '%s'\n%!" x)
    "Usage: ";
  (*                                 *)
  if not !edit then begin
    Ivy.init "Paparazzi GCS" "READY" (fun _ _ -> ());
    Ivy.start !ivy_bus
  end;

  Srtm.add_path default_path_srtm;
  Gm.cache_path := var_maps_path;
  IGN.cache_path := var_maps_path;

  let layout_file = layout_path // !layout_file in
  let layout = ExtXml.parse_file layout_file in
  let width = ExtXml.int_attrib layout "width"
  and height = ExtXml.int_attrib layout "height" in

  let pid_plugin = ref None in
  let kill_plugin = fun () ->
    match !pid_plugin with
        None -> ()
      | Some pid ->
        try
          Unix.kill pid (-9);
          ignore (Unix.waitpid [] pid)
        with _ -> () in
  let destroy = fun _ ->
    kill_plugin ();
    exit 0 in

  (** The whole window map2d **)
  let window, switch_fullscreen =
    match !wid with
        None ->
          let icon = GdkPixbuf.from_file Env.icon_gcs_file in
          let window = GWindow.window ~icon ~title:"GCS" ~border_width:1 ~width ~height ~allow_shrink:true () in
          if !maximize then
            window#maximize ();
          if !fullscreen then
            window#fullscreen ();
          ignore (window#connect#destroy ~callback:destroy);
          let switch_fullscreen = fun () ->
            fullscreen := not !fullscreen;
            if !fullscreen then
              window#fullscreen ()
            else
              window#unfullscreen () in
          (window:>GWindow.window_skel),switch_fullscreen

      | Some xid ->
        let window =
          IFDEF GDK_NATIVE_WINDOW THEN
            Gdk.Window.native_of_xid xid
          ELSE
            xid
          END
        in
        (GWindow.plug ~window ~width ~height ():>GWindow.window_skel), fun _ -> () in

  (* Editor frame *)
  let editor_frame = GBin.frame () in

  let geomap, menu_fact = create_geomap switch_fullscreen editor_frame in

  let map_frame = GPack.vbox () in
  (** Put the canvas in a frame *)
  map_frame#add geomap#frame#coerce;

  (** window for the strip panel *)
  let scrolled = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
  let strips_table = GPack.vbox ~spacing:5 ~packing:scrolled#add_with_viewport () in

  (** Aircraft notebook *)
  let ac_notebook = GPack.notebook ~tab_border:0 () in

  (** Alerts text frame *)
  let alert_page = GBin.frame () in
  let my_alert = new Pages.alert alert_page in

  (** Altitude graph frame *)
  let alt_graph = new Gtk_tools.pixmap_in_drawin_area () in

  (** plugin frame *)
  let plugin_width = 400 and plugin_height = 300 in
  let plugin_frame = GPack.vbox ~width:plugin_width () in

  let widgets = ["map2d", map_frame#coerce;
                 "strips", scrolled#coerce;
                 "aircraft", ac_notebook#coerce;
                 "editor", editor_frame#coerce;
                 "alarms", alert_page#coerce;
                 "altgraph", alt_graph#drawing_area#coerce;
                 "plugin", plugin_frame#coerce] in

  let the_layout = ExtXml.child layout "0" in
  pack_widgets `HORIZONTAL the_layout widgets window#add;

  (** packing papgets *)
  let papgets = try find_widget_children "map2d" the_layout with Not_found -> [] in
  List.iter (Papgets.create geomap#still) papgets;
  listen_dropped_papgets geomap;

  let save_layout = fun () ->
    (* Ask if ac_id parameters from papgets should be saved *)
    let save_acid =
      if Papgets.has_papgets () then
        match GToolbox.question_box ~title:"Save Layout" ~buttons:["Yes"; "no"] ~default:1 "Do you want to save A/C id of Papgets if available\nYes: the saved layout will only work with A/C that have the same id (default)\nno: the saved layout will work with any A/C (but will mix data while using multiple A/C)" with
        | 2 -> false
        | _ -> true
      else true
    in
    let the_new_layout = replace_widget_children "map2d" (Papgets.dump_store save_acid) the_layout in
    let width, height = Gdk.Drawable.get_size window#misc#window in
    let the_new_layout = update_widget_size `HORIZONTAL widgets the_new_layout in
    let new_layout = Xml.Element ("layout", ["width", soi width; "height", soi height], [the_new_layout]) in
    save_layout layout_file (Xml.to_string_fmt new_layout)
  in
  ignore (menu_fact#add_item "Save layout" ~key:GdkKeysyms._S ~callback:save_layout);


  if !mplayer <> "" then
    plugin_window := sprintf "mplayer -really-quiet -nomouseinput %s -wid " !mplayer;
  if !plugin_window <> "" then begin
    if plugin_frame#misc#parent = None then
      failwith "Error: \"plugin\" widget required in layout description";
    let frame = GBin.event_box ~packing:plugin_frame#add ~width:plugin_width ~height:plugin_height () in
    let s = GWindow.socket ~packing:frame#add () in
    let com = sprintf "%s0x%lx" !plugin_window s#xwindow in

    let restart = fun () ->
      begin match !pid_plugin with
          None -> ()
        | Some p -> try Unix.kill p Sys.sigkill with _ -> ()
      end;
      let com = sprintf "exec %s" com in
      let dev_null = Unix.descr_of_out_channel (open_out "/dev/null") in
      pid_plugin := Some (Unix.create_process "/bin/sh" [|"/bin/sh"; "-c"; com|] dev_null dev_null dev_null) in

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
      match GdkEvent.Button.button ev with
          1 -> swap (); true
        | 3 -> restart (); true
        | _ -> false in

    ignore (frame#event#connect#button_press ~callback);
    ignore (menu_fact#add_item "Swap plugin/map" ~callback:(fun _ -> swap ()));
  end;

  (** Wait for A/Cs and subsequent messages *)
  if not !edit then
    begin
      my_alert#add "Waiting for telemetry...";
      Speech.say "Waiting for telemetry...";
      Live.listen_acs_and_msgs geomap ac_notebook strips_table my_alert !auto_center_new_ac alt_graph !timestamp
    end;

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

  if !file_to_edit <> "" then
    if Sys.file_exists !file_to_edit then
      EditFP.load_xml_file geomap editor_frame accel_group !file_to_edit
    else
      GToolbox.message_box "Error" (sprintf "Error: '%s' file does not exist\n%!" !file_to_edit);

  if !edit then
    EditFP.set_window_title geomap;

  (** Threaded main loop (map tiles loaded concurently) *)
  GtkThread.main ()
