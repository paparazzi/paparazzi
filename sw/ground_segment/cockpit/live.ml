(******* Real time handling of flying A/Cs ***********************************)

module G = MapCanvas
open Latlong
open Printf

module Tele_Pprz = Pprz.Messages(struct let name = "telemetry" end)
module Ground_Pprz = Pprz.Messages(struct let name = "ground" end)
module Alert_Pprz = Pprz.Messages(struct let name = "alert" end)

let rotate = fun a (x, y) ->
  let cosa = cos a and sina = sin a in
  (cosa *.x +. sina *.y, -. sina*.x +. cosa *. y)

let rec list_casso x = function
    [] -> raise Not_found
  | (a,b)::abs -> if x = b then a else list_casso x abs

let rec list_iter3 = fun f l1 l2 l3 ->
  match l1, l2, l3 with
    [], [], [] -> ()
  | x1::x1s, x2::x2s, x3::x3s ->
      f x1 x2 x3;
      list_iter3 f x1s x2s x3s
  | _ -> invalid_arg "list_iter3"


type color = string
type aircraft = {
    ac_name : string;
    config : Pprz.values;
    track : MapTrack.track;
    color: color;
    fp_group : MapFP.flight_plan;
    fp : Xml.xml;
    blocks : (int * string) list;
    mutable last_ap_mode : string;
    mutable last_stage : int * int;
    ir_page : Pages.infrared;
    gps_page : Pages.gps;
    pfd_page : Pages.pfd;
    misc_page : Pages.misc;
    settings_page : Pages.settings option;
    strip : Strip.t;
    mutable first_pos : bool
  }

let live_aircrafts = Hashtbl.create 3
let get_ac = fun vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs in
  Hashtbl.find live_aircrafts ac_id

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

let survey_status_msg = fun track geo1 geo2 ->
  track#draw_zone geo1 geo2

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
    GToolbox.input_string ~text:(string_of_int track#size) ~title:ac "Track size"
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

let center = fun geomap track () ->
  match track#last with
    None -> ()
  | Some geo ->
      geomap#center geo


let blocks_of_stages = fun stages ->
  let blocks = ref [] in
  List.iter (fun x ->
    let name = ExtXml.attrib x "block_name"
    and id = ExtXml.int_attrib x "block" in
    if not (List.mem_assoc id !blocks) then
      blocks := (id, name) :: !blocks)
    (Xml.children stages);
  List.sort compare !blocks

let jump_to_block = fun ac_id id ->
  Ground_Pprz.message_send "map2d" "JUMP_TO_BLOCK" 
    ["ac_id", Pprz.String ac_id; "block_id", Pprz.Int id]

let menu_entry_of_block = fun ac_id (id, name) ->
  let send_msg = fun () -> jump_to_block ac_id id in
  `I (name, send_msg)

let reset_waypoints = fun fp () ->
  List.iter (fun w ->
    let (_i, w) = fp#index w in
    w#reset_moved ())
    fp#waypoints

let icon = ref None
let show_snapshot = fun (geomap:G.widget) geo_FL geo_BR point pixbuf name ev ->
  match ev with
  | `BUTTON_PRESS _ev ->
      let image = GMisc.image ~pixbuf () in
      let icon = image#coerce in
      begin
	match GToolbox.question_box ~title:name ~buttons:["Delete"; "Close"] ~icon "" with
	  1  -> 
	    point#destroy ()
	| _ -> ()
      end;
      true
  | `LEAVE_NOTIFY _ev ->
      begin
	match !icon with
	  None -> ()
	| Some i -> i#destroy ()
      end;
      false
  | `ENTER_NOTIFY _ev ->
      let w = GdkPixbuf.get_width pixbuf
      and h = GdkPixbuf.get_height pixbuf in
      icon := Some (geomap#display_pixbuf ((0,0), geo_FL) ((w,h), geo_BR) pixbuf);
      false

  | _ -> false
	

let mark = fun (geomap:G.widget) ac_id track plugin_frame ->
  let i = ref 1 in fun () ->
    match track#last with
      Some geo ->
	begin
	  let group = geomap#background in
	  let point = geomap#circle ~group ~fill_color:"blue" geo 5. in
	  point#raise_to_top ();
	  let lat = (Rad>>Deg)geo.posn_lat
	  and long = (Rad>>Deg)geo.posn_long in
	  Tele_Pprz.message_send ac_id "MARK" 
	    ["ac_id", Pprz.String ac_id;
	     "lat", Pprz.Float lat;
	     "long", Pprz.Float long];
	  match plugin_frame with
	    None -> () 
	  | Some frame ->
	      let width, height = Gdk.Drawable.get_size frame#misc#window in
	      let dest = GdkPixbuf.create width height() in
	      GdkPixbuf.get_from_drawable ~dest ~width ~height frame#misc#window;
	      let name = sprintf "Snapshot-%s-%d_%f_%f_%f.png" ac_id !i lat long (track#last_heading) in
	      let png = sprintf "%s/var/logs/%s" Env.paparazzi_home name in
	      GdkPixbuf.save png "png" dest;
	      incr i;

	      (* Computing the footprint: front_left and back_right *)
	      let cam_aperture = 2.4/.1.9 in (* width over distance FIXME *)
	      let alt = track#last_altitude -. float (Srtm.of_wgs84 geo) in
	      let width = cam_aperture *. alt in
	      let height = width *. 3. /. 4. in
	      let utm = utm_of WGS84 geo in
	      let a = (Deg>>Rad)track#last_heading in
	      let (xfl,yfl) = rotate a (-.width/.2., height/.2.)
	      and (xbr,ybr) = rotate a (width/.2., -.height/.2.) in
	      let geo_FL = of_utm WGS84 (utm_add utm (xfl,yfl))
	      and geo_BR = of_utm WGS84 (utm_add utm (xbr,ybr)) in
	      ignore (point#connect#event (show_snapshot geomap geo_FL geo_BR point dest name))
	end
    | None -> ()


(** Load a mission. Returns the XML window *)
let load_mission = fun ?edit color geomap xml ->
  Map2d.set_georef_if_none geomap (MapFP.georef_of_xml xml);
  new MapFP.flight_plan ?edit ~show_moved:true geomap color Env.flight_plan_dtd xml



let create_ac = fun (geomap:G.widget) (acs_notebook:GPack.notebook) ac_id config ->
  let color = Pprz.string_assoc "default_gui_color" config
  and name = Pprz.string_assoc "ac_name" config in

  (** Get the flight plan **)
  let fp_url = Pprz.string_assoc "flight_plan" config in
  let fp_file = Http.file_of_url fp_url in
  let fp_xml_dump = Xml.parse_file fp_file in
  let stages = ExtXml.child fp_xml_dump "stages" in
  let blocks = blocks_of_stages stages in

  let label_box = GBin.event_box () in
  let label = GPack.hbox ~packing:label_box#add ~spacing:3 () in
  let eb = GBin.event_box ~width:10 ~height:10 ~packing:label#pack () in
  eb#coerce#misc#modify_bg [`NORMAL, `NAME color];
  let _ac_label = GMisc.label ~text:name ~packing:label#pack () in

  let ac_mi = GMenu.image_menu_item ~image:label_box ~packing:geomap#menubar#append () in
  let ac_menu = GMenu.menu () in
  ac_mi#set_submenu ac_menu;
  let ac_menu_fact = new GMenu.factory ac_menu in
  let fp = ac_menu_fact#add_check_item "Fligh Plan" ~active:true in
  ignore (fp#connect#toggled (fun () -> show_mission ac_id fp#active));
  
  let track = new MapTrack.track ~name ~color:color geomap in

  let center_ac = center geomap track in
  ignore (ac_menu_fact#add_item "Center A/C" ~callback:center_ac);
  
  ignore (ac_menu_fact#add_item "Clear Track" ~callback:(fun () -> track#clear_map2D));
  ignore (ac_menu_fact#add_item "Resize Track" ~callback:(fun () -> resize_track ac_id track));
  let reset_wp_menu = ac_menu_fact#add_item "Reset Waypoints" in

  let jump_block_entries = List.map (menu_entry_of_block ac_id) blocks in
  
  let commit_moves = fun () ->
    commit_changes ac_id in
  let sm = ac_menu_fact#add_submenu "Datalink" in
  let dl_menu = [
    `M ("Jump to block", jump_block_entries);
    `I ("Commit Moves", commit_moves)] in

  GToolbox.build_menu sm ~entries:dl_menu;

  let cam = ac_menu_fact#add_check_item "Cam Display" ~active:false in
  ignore (cam#connect#toggled (fun () -> track#set_cam_state cam#active));
  let params = ac_menu_fact#add_check_item "flight param. display" ~active:false in
  ignore (params#connect#toggled (fun () -> track#set_params_state params#active));

  (** Build the XML flight plan, connect then "jump_to_block" *)
  let fp_xml = ExtXml.child fp_xml_dump "flight_plan" in
  let fp = load_mission ~edit:false color geomap fp_xml in
  fp#connect_activated (fun node ->
    if XmlEdit.tag node = "block" then
      let block = XmlEdit.attrib node "name" in
      let id = list_casso block blocks in
      jump_to_block ac_id id);
  ignore (reset_wp_menu#connect#activate (reset_waypoints fp));


  (** Add a new tab in the A/Cs notebook, with a colored label *)
  let eb = GBin.event_box () in
  let _label = GMisc.label ~text:name ~packing:eb#add () in
  eb#coerce#misc#modify_bg [`NORMAL, `NAME color;`ACTIVE, `NAME color];

  (** Put a notebook for this A/C *)
  let ac_frame = GBin.frame ~packing:(acs_notebook#append_page ~tab_label:eb#coerce) () in
  let ac_notebook = GPack.notebook ~packing: ac_frame#add () in
  let visible = fun w ->
    ac_notebook#page_num w#coerce = ac_notebook#current_page in      

  (** Insert the flight plan tab *)
  let fp_label = GMisc.label ~text: "Flight Plan" () in
  (ac_notebook:GPack.notebook)#append_page ~tab_label:fp_label#coerce fp#window#coerce;
  
  let infrared_label = GMisc.label ~text: "Infrared" () in
  let infrared_frame = GBin.frame ~shadow_type: `NONE
      ~packing: (ac_notebook#append_page ~tab_label: infrared_label#coerce) () in
  let ir_page = new Pages.infrared infrared_frame in
  
  let gps_label = GMisc.label ~text: "GPS" () in
  let gps_frame = GBin.frame ~shadow_type: `NONE
      ~packing: (ac_notebook#append_page ~tab_label: gps_label#coerce) () in
  let gps_page = new Pages.gps ~visible gps_frame in

  let pfd_label = GMisc.label ~text: "PFD" () in
  let pfd_frame = GBin.frame ~shadow_type: `NONE
      ~packing: (ac_notebook#append_page ~tab_label: pfd_label#coerce) () in
  let pfd_page = new Pages.pfd pfd_frame
  and _pfd_page_num = ac_notebook#page_num pfd_frame#coerce in

  let misc_label = GMisc.label ~text: "Misc" () in
  let misc_frame = GBin.frame ~shadow_type: `NONE
      ~packing: (ac_notebook#append_page ~tab_label:misc_label#coerce) () in
  let misc_page = new Pages.misc ~packing:misc_frame#add misc_frame in

  let settings_page =
    let settings_url = Pprz.string_assoc "settings" config in
    let settings_file = Http.file_of_url settings_url in
    let settings_xml = Xml.parse_file settings_file in
    let xml_settings = Xml.children (ExtXml.child settings_xml "dl_settings") in
    let callback = fun idx value -> 
      let vs = ["ac_id", Pprz.String ac_id; "index", Pprz.Int idx;"value", Pprz.Float value] in
      Ground_Pprz.message_send "dl" "DL_SETTING" vs in
    let settings_tab = new Pages.settings ~visible xml_settings callback in
    let tab_label = (GMisc.label ~text:"Settings" ())#coerce in
    ac_notebook#append_page ~tab_label settings_tab#widget;
    Some settings_tab in
  
  (** Add a strip and connect it to the A/C notebook *)
  let select_this_tab =
    let n = acs_notebook#page_num ac_frame#coerce in
    fun () -> acs_notebook#goto_page n in
  let strip = Strip.add config color select_this_tab center_ac commit_moves (mark geomap ac_id track !Plugin.frame) in


  Hashtbl.add live_aircrafts ac_id { track = track; color = color; 
				     fp_group = fp ; config = config ; 
				     fp = fp_xml; ac_name = name;
				     blocks = blocks; last_ap_mode= "";
				     last_stage = (-1,-1);
				     ir_page = ir_page;
				     gps_page = gps_page;
				     pfd_page = pfd_page;
				     misc_page = misc_page;
				     settings_page = settings_page;
				     strip = strip; first_pos = true
				   }

    (** Bind to message while catching all the esceptions of the callback *)
let safe_bind = fun msg cb ->
  let safe_cb = fun sender vs ->
    try cb sender vs with _ -> () in
  ignore (Ground_Pprz.message_bind msg safe_cb)

let alert_bind = fun msg cb ->
  let safe_cb = fun sender vs ->
    try cb sender vs with _ -> () in
  ignore (Alert_Pprz.message_bind msg safe_cb)

let ask_config = fun geomap fp_notebook ac ->
  let get_config = fun _sender values ->
    create_ac geomap fp_notebook ac values
  in
  Ground_Pprz.message_req "map2d" "CONFIG" ["ac_id", Pprz.String ac] get_config

    

let one_new_ac = fun (geomap:G.widget) fp_notebook ac ->
  if not (Hashtbl.mem live_aircrafts ac) then begin
    ask_config geomap fp_notebook ac
  end

let get_wind_msg = fun _sender vs ->
  let ac = get_ac vs in
  let value = fun field_name -> sprintf "%.1f" (Pprz.float_assoc field_name vs) in
  ac.misc_page#set_wind_speed (value "wspeed");
  ac.misc_page#set_wind_dir (value "dir")

let get_fbw_msg = fun _sender vs ->
  let ac = get_ac vs in
  let status = Pprz.string_assoc "rc_status" vs in
  Strip.set_label ac.strip "RC" status;
  Strip.set_color ac.strip "RC"
    (match status with
      "LOST" -> "orange"
    | "REALLY_LOST" -> "red" 
    | _ -> "white")

    

let get_engine_status_msg = fun _sender vs ->
  let ac = get_ac vs in
  Strip.set_label ac.strip "throttle" 
    (string_of_float (Pprz.float_assoc "throttle" vs));
  Strip.set_bat ac.strip (Pprz.float_assoc "bat" vs)
    
let get_if_calib_msg = fun _sender _vs ->
  (** FIXME No longer in the strip *)
  ()

let listen_wind_msg = fun () ->
  safe_bind "WIND" get_wind_msg

let listen_fbw_msg = fun () ->
  safe_bind "FLY_BY_WIRE" get_fbw_msg

let listen_engine_status_msg = fun () ->
  safe_bind "ENGINE_STATUS" get_engine_status_msg

let listen_if_calib_msg = fun () ->
  safe_bind "INFLIGH_CALIB" get_if_calib_msg

let list_separator = Str.regexp ","
    
let aircrafts_msg = fun (geomap:G.widget) fp_notebook acs ->
  let acs = Pprz.string_assoc "ac_list" acs in
  let acs = Str.split list_separator acs in
  List.iter (one_new_ac geomap fp_notebook) acs


let listen_dl_value = fun () ->
  let get_dl_value = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    let ac = Hashtbl.find live_aircrafts ac_id in
    match ac.settings_page with
      Some settings ->
	let csv = Pprz.string_assoc "values" vs in
	let values = Array.of_list (Str.split list_separator csv) in
	for i = 0 to min (Array.length values) settings#length - 1 do
	  settings#set i (float_of_string values.(i))
	done
    | None -> () in
  safe_bind "DL_VALUES" get_dl_value


let highlight_fp = fun ac b s ->
  if (b, s) <> ac.last_stage then begin
    ac.last_stage <- (b, s);
    ac.fp_group#highlight_stage b s
  end


let listen_flight_params = fun geomap auto_center_new_ac ->
  let get_fp = fun _sender vs ->
    let ac = get_ac vs in
    let pfd_page = ac.pfd_page in

    pfd_page#set_attitude (Pprz.float_assoc "roll" vs) (Pprz.float_assoc "pitch" vs);
    pfd_page#set_alt (Pprz.float_assoc "alt" vs);
    pfd_page#set_climb (Pprz.float_assoc "climb" vs);
    pfd_page#set_speed (Pprz.float_assoc "speed" vs);

    let a = fun s -> Pprz.float_assoc s vs in
    let wgs84 = { posn_lat = (Deg>>Rad)(a "lat"); posn_long = (Deg>>Rad)(a "long") } in
    aircraft_pos_msg ac.track wgs84 (a "course") (a "alt")  (a "speed") (a "climb");

    if auto_center_new_ac && ac.first_pos then begin
      center geomap ac.track ();
      ac.first_pos <- false
    end;

    let set_label lbl_name field_name =
      let s = 
	if (a field_name) < 0. 
	then 
	  "- "^(sprintf "%.1f" (abs_float (a field_name)))
	else
	  sprintf "%.1f" (a field_name)
      in
      Strip.set_label ac.strip lbl_name s
    in
    set_label "alt" "alt";
    set_label "speed" "speed";
    set_label "climb" "climb"
  in
  safe_bind "FLIGHT_PARAM" get_fp;

  let get_ns = fun _sender vs ->
    let ac = get_ac vs in
    let a = fun s -> Pprz.float_assoc s vs in
    let wgs84 = { posn_lat = (Deg>>Rad)(a "target_lat"); posn_long = (Deg>>Rad)(a "target_long") } in
    carrot_pos_msg ac.track wgs84;
    let cur_block = Pprz.int_assoc "cur_block" vs
    and cur_stage = Pprz.int_assoc "cur_stage" vs in
    let b = List.assoc cur_block ac.blocks in
    let b = String.sub b 0 (min 10 (String.length b)) in
    highlight_fp ac cur_block cur_stage;
    let set_label = fun l f ->
      Strip.set_label ac.strip l (sprintf "%.1f" (Pprz.float_assoc f vs)) in
    set_label "->" "target_alt";
    set_label "/" "target_climb";
    Strip.set_label ac.strip "block_name" b
  in
  safe_bind "NAV_STATUS" get_ns;

  let get_cam_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    let ac = Hashtbl.find live_aircrafts ac_id in
    let a = fun s -> Pprz.float_assoc s vs in
    let wgs84 = { posn_lat = (Deg>>Rad)(a "cam_lat"); posn_long = (Deg>>Rad)(a "cam_long") }
    and target_wgs84 = { posn_lat = (Deg>>Rad)(a "cam_target_lat"); posn_long = (Deg>>Rad)(a "cam_target_long") } in
    
    cam_pos_msg ac.track wgs84 target_wgs84
  in
  safe_bind "CAM_STATUS" get_cam_status;

  let get_circle_status = fun _sender vs ->
    let ac = get_ac vs in
    let a = fun s -> Pprz.float_assoc s vs in
    let wgs84 = { posn_lat = (Deg>>Rad)(a "circle_lat"); posn_long = (Deg>>Rad)(a "circle_long") } in
    circle_status_msg ac.track wgs84 (float_of_string (Pprz.string_assoc "radius" vs)) 
  in
  safe_bind "CIRCLE_STATUS" get_circle_status;

  let get_segment_status = fun _sender vs ->
    let ac_id = Pprz.string_assoc "ac_id" vs in
    let ac = Hashtbl.find live_aircrafts ac_id in
    let a = fun s -> Pprz.float_assoc s vs in
    let geo1 = { posn_lat = (Deg>>Rad)(a "segment1_lat"); posn_long = (Deg>>Rad)(a "segment1_long") }
    and geo2 = { posn_lat = (Deg>>Rad)(a "segment2_lat"); posn_long = (Deg>>Rad)(a "segment2_long") } in
    segment_status_msg ac.track geo1 geo2
  in
  safe_bind "SEGMENT_STATUS" get_segment_status;


  let get_survey_status = fun _sender vs ->
    let ac = get_ac vs in
    let a = fun s -> Pprz.float_assoc s vs in
    let geo1 = { posn_lat = (Deg>>Rad)(a "south_lat"); posn_long = (Deg>>Rad)(a "west_long") }
    and geo2 = { posn_lat = (Deg>>Rad)(a "north_lat"); posn_long = (Deg>>Rad)(a "east_long") } in
    survey_status_msg ac.track geo1 geo2
  in
  safe_bind "SURVEY_STATUS" get_survey_status;


  let get_ap_status = fun _sender vs ->
    let ac = get_ac vs in
    ap_status_msg ac.track ( float_of_int (Pprz.int32_assoc "flight_time" vs ));
    let ap_mode = Pprz.string_assoc "ap_mode" vs in
    if ap_mode <> ac.last_ap_mode then begin
      Speech.say (sprintf "%s, %s" ac.ac_name ap_mode);
      ac.last_ap_mode <- ap_mode
    end;
    Strip.set_label ac.strip "AP" (Pprz.string_assoc "ap_mode" vs);
    Strip.set_color ac.strip "AP" (if ap_mode="HOME" then "red" else "white");
    let gps_mode = Pprz.string_assoc "gps_mode" vs in
    Strip.set_label ac.strip "GPS" gps_mode;
    Strip.set_color ac.strip "GPS" (if gps_mode<>"3D" then "red" else "white");
    let ft = 
      let t = Int32.to_int (Int32.of_string (Pprz.string_assoc "flight_time" vs)) in
      sprintf "%02d:%02d:%02d" (t / 3600) ((t mod 3600) / 60) ((t mod 3600) mod 60) in
    Strip.set_label ac.strip "flight_time" ft
  in
  safe_bind "AP_STATUS" get_ap_status;

  listen_dl_value ()

let listen_waypoint_moved = fun () ->
  let get_values = fun _sender vs ->
    let ac = get_ac vs in
    let wp_id = Pprz.int_assoc "wp_id" vs in
    let a = fun s -> Pprz.float_assoc s vs in
    let geo = { posn_lat = (Deg>>Rad)(a "lat"); posn_long = (Deg>>Rad)(a "long") }
    and altitude = a "alt" in

    (** FIXME: No indexed access to waypoints: iter and compare: *)
    List.iter (fun w ->
      let (i, w) = ac.fp_group#index w in
      if i = wp_id then begin
	w#set ~if_not_moved:true ~altitude ~update:true geo;
	raise Exit (** catched by safe_bind *)
      end)
      ac.fp_group#waypoints
  in
  safe_bind "WAYPOINT_MOVED" get_values
    
let get_alert_bat_low = fun a _sender vs -> 
  let ac_id = Pprz.string_assoc "ac_id" vs in
  let ac_name = ref "" in
  let level = Pprz.string_assoc "level" vs in
  let get_config = fun _sender config ->
    ac_name := Pprz.string_assoc "ac_name" config;
    a#add (!ac_name^" "^"BAT_LOW"^" "^level)
  in
  Ground_Pprz.message_req "map2d" "CONFIG" ["ac_id", Pprz.String ac_id] get_config
    

let listen_alert = fun a -> 
  alert_bind "BAT_LOW" (get_alert_bat_low a)
    

let get_infrared = fun _sender vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs in
  let ac = Hashtbl.find live_aircrafts ac_id in
  let ir_page = ac.ir_page in
  let gps_hybrid_mode = Pprz.string_assoc "gps_hybrid_mode" vs in
  let gps_hybrid_factor = Pprz.float_assoc "gps_hybrid_factor" vs in
  let contrast_status = Pprz.string_assoc "contrast_status" vs in
  let contrast_value = Pprz.int_assoc "contrast_value" vs in

  ir_page#set_gps_hybrid_mode gps_hybrid_mode;
  ir_page#set_gps_hybrid_factor gps_hybrid_factor;
  ir_page#set_contrast_status contrast_status;
  ir_page#set_contrast_value contrast_value
    
let listen_infrared = fun () -> safe_bind "INFRARED" get_infrared

let get_svsinfo = fun _sender vs ->
  let ac_id = Pprz.string_assoc "ac_id" vs in
  let ac = Hashtbl.find live_aircrafts ac_id in
  let gps_page = ac.gps_page in
  let svid = Str.split list_separator (Pprz.string_assoc "svid" vs)
  and cn0 = Str.split list_separator (Pprz.string_assoc "cno" vs)
  and flags = Str.split list_separator (Pprz.string_assoc "flags" vs) in 
  
  list_iter3
    (fun id cno flags ->
      if id <> "0" then gps_page#svsinfo id cno (int_of_string flags))
    svid cn0 flags
    
let listen_svsinfo = fun () -> safe_bind "SVSINFO" get_svsinfo

let message_request = Ground_Pprz.message_req
