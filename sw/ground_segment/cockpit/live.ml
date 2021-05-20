(*
 * Real time handling of flying A/Cs
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
open Latlong
module LL = Latlong
open Printf


module Tele_Pprz = PprzLink.Messages(struct let name = "telemetry" end)
module Ground_Pprz = PprzLink.Messages(struct let name = "ground" end)
module Alert_Pprz = PprzLink.Messages(struct let name = "alert" end)


let (//) = Filename.concat

let gcs_id = "GCS"

let approaching_alert_time = 3.
let approaching_alert_dmin = 0.5
let approaching_alert_slmin = 20.

let track_size = ref 500

let _auto_hide_fp = ref false

(* request AIRCRAFTS list until got first answer from server *)
let _req_aircrafts = ref true

let min_height = 200
let lines_height = 30

let is_int = fun x ->
  try let _ = int_of_string x in true with _ -> false

let ok_modes = ["MANUAL"; "AUTO1"; "AUTO2"]

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
type gps_acc_level = GPS_ACC_HIGH | GPS_ACC_LOW | GPS_ACC_VERY_LOW | GPS_NO_ACC

type aircraft = {
  ac_name : string;
  ac_speech_name : string;
  config : PprzLink.values;
  track : MapTrack.track;
  color: color;
  fp_group : MapFP.flight_plan;
  fp_show : GMenu.check_menu_item;
  wp_HOME : MapWaypoints.waypoint option;
  fp : Xml.xml;
  blocks : (int * string) list;
  mutable last_ap_mode : string;
  mutable last_stage : int * int;
  ir_page : Pages.infrared;
  gps_page : Pages.gps;
  pfd_page : Horizon.pfd;
  link_page : Pages.link;
  misc_page : Pages.misc;
  dl_settings_page : Page_settings.settings option;
  rc_settings_page : Pages.rc_settings option;
  pages : GObj.widget;
  notebook_label : GMisc.label;
  strip : Strip.t;
  rc_max_rate: float;
  mutable first_pos : bool;
  mutable last_block_name : string;
  mutable in_kill_mode : bool;
  mutable speed : float;
  mutable alt : float;
  mutable target_alt : float;
  mutable flight_time : int;
  mutable wind_speed : float;
  mutable wind_dir : float; (* Rad, clockwise from North *)
  mutable ground_prox : bool;
  mutable got_track_status_timer : int;
  mutable last_dist_to_wp : float;
  mutable dl_values : string option array;
  mutable last_unix_time : float;
  mutable airspeed : float;
  mutable version : string;
  mutable last_gps_acc : gps_acc_level;
  mutable last_bat_warn_time : float
}

let list_separator = Str.regexp ","
let filter_acs = ref []

let aircrafts = Hashtbl.create 3
exception AC_not_found
let find_ac = fun ac_id ->
  try
    Hashtbl.find aircrafts ac_id
  with
      Not_found -> raise AC_not_found

let active_ac = ref ""
let get_ac = fun vs ->
  let ac_id = PprzLink.string_assoc "ac_id" vs in
    find_ac ac_id

let show_fp = fun ac ->
  ac.fp_group#show ();
  ac.fp_show#set_active true

let hide_fp = fun ac ->
  ac.fp_group#hide ();
  ac.fp_show#set_active false

(* callback for FP check button in menu *)
let show_mission = fun ac on_off ->
  let a = find_ac ac in
  if on_off then
    a.fp_group#show ()
  else
    a.fp_group#hide ()

let auto_hide_fp = fun hide ->
  let _hide_fp = fun () ->
    Hashtbl.iter (fun _ a -> hide_fp a) aircrafts;
    if !active_ac <> "" then begin
      let a = find_ac !active_ac in
      show_fp a
    end;
  in
  _auto_hide_fp := hide;
  if hide then _hide_fp () else Hashtbl.iter (fun _ a -> show_fp a) aircrafts

let select_ac = fun acs_notebook ac_id ->
  if !active_ac <> ac_id then
    let ac = Hashtbl.find aircrafts ac_id in

    (* Show the buttons in the active strip and hide the previous active one *)
    ac.strip#show_buttons ();
    if !active_ac <> "" then begin
      let ac' = find_ac !active_ac in
      ac'.strip#hide_buttons ();
      ac'.notebook_label#set_width_chars (String.length ac'.notebook_label#text);
      if !_auto_hide_fp then hide_fp ac'
    end;

    (* Set the new active *)
    active_ac := ac_id;
    if !_auto_hide_fp then show_fp ac;

    (* Select and enlarge the label of the A/C notebook *)
    let n = acs_notebook#page_num ac.pages in
    acs_notebook#goto_page n;
    ac.notebook_label#set_width_chars 20

let filter_ac_ids = fun acs ->
  let acs = Str.split list_separator acs in
  filter_acs := acs;

module M = Map.Make (struct type t = string let compare = compare end)
let log =
  let last = ref M.empty in
  fun ?(say = false) (a:Pages.alert) ac_id s ->
    if not (M.mem ac_id !last) || M.find ac_id !last <> s then begin
      last := M.add ac_id s (M.remove ac_id !last);
      if say then Speech.say s;
      a#add s
    end

let log_and_say = fun a ac_id s -> log ~say:true a ac_id s

let resize_track = fun ac track ->
  match
    GToolbox.input_string ~text:(string_of_int track#size) ~title:ac "Track size"
  with
      None -> ()
    | Some s -> track#resize (int_of_string s)


let send_move_waypoint_msg = fun ac i w ->
  let wgs84 = w#pos in
  let vs = ["ac_id", PprzLink.String ac;
            "wp_id", PprzLink.Int i;
            "lat", PprzLink.Float ((Rad>>Deg)wgs84.posn_lat);
            "long", PprzLink.Float ((Rad>>Deg)wgs84.posn_long);
            "alt", PprzLink.Float w#alt
           ] in
  Ground_Pprz.message_send "gcs" "MOVE_WAYPOINT" vs

let commit_changes = fun ac ->
  let a = find_ac ac in
  List.iter
    (fun w ->
      let (i, w) = a.fp_group#index w in
      if w#moved then
        send_move_waypoint_msg ac i w)
    a.fp_group#waypoints

let center = fun geomap track () ->
  match track#last with
      None -> ()
    | Some geo ->
      geomap#center geo;
      geomap#canvas#misc#draw None


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
  Ground_Pprz.message_send "gcs" "JUMP_TO_BLOCK"
    ["ac_id", PprzLink.String ac_id; "block_id", PprzLink.Int id]

let dl_setting = fun ac_id idx value ->
  let vs = ["ac_id", PprzLink.String ac_id; "index", PprzLink.Int idx;"value", PprzLink.Float value] in
  Ground_Pprz.message_send "dl" "DL_SETTING" vs

let dl_emergency_cmd = fun ac_id cmd ->
  let vs = ["ac_id", PprzLink.String ac_id; "cmd", PprzLink.Int cmd] in
  Ground_Pprz.message_send "dl" "DL_EMERGENCY_CMD" vs

let get_dl_setting = fun ac_id idx ->
  let vs = ["ac_id", PprzLink.String ac_id; "index", PprzLink.Int idx] in
  Ground_Pprz.message_send "dl" "GET_DL_SETTING" vs

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
      point#raise_to_top ();
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
              ["ac_id", PprzLink.String ac_id;
               "lat", PprzLink.Float lat;
               "long", PprzLink.Float long];
            let frame =
              match plugin_frame with
                  None -> geomap#canvas#coerce
                | Some pf -> pf#coerce in
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


(** Light display of attributes in the flight plan. *)
let attributes_pretty_printer = fun attribs ->
  (* Remove the optional attributes *)
  let valid = fun a ->
    let a = Compat.lowercase_ascii a in
    a <> "no" && a <> "strip_icon" && a <> "strip_button" && a <> "pre_call"
    && a <> "post_call" && a <> "key" && a <> "group" in

  let sprint_opt = fun b s ->
    if String.length b > 0 then
      sprintf " %s%s%s" s b s
    else
      ""
  in
  let elt = Xml.Element("", attribs, []) in
  let pre_call = ExtXml.attrib_or_default elt "pre_call" ""
  and post_call = ExtXml.attrib_or_default elt "post_call" "" in

  let attribs = List.filter (fun (a, _) -> valid a) attribs in

  (* Don't print the name of the attribute if there is only one *)
  match attribs with
      [(_, v)] -> v ^ sprint_opt pre_call "]" ^ sprint_opt post_call "["
    | _        -> XmlEdit.string_of_attribs attribs


(** Load a mission. Returns the XML window *)
let load_mission = fun ?editable color geomap xml ->
  Map2d.set_georef_if_none geomap (MapFP.georef_of_xml xml);
  new MapFP.flight_plan ~format_attribs:attributes_pretty_printer ?editable ~show_moved:true geomap color Env.flight_plan_dtd xml

let get_rc_max_rate = fun af_xml ->
  let default_max_rate = 50. in
  try
    let gcs_section = ExtXml.child af_xml ~select:(fun x -> Xml.attrib x "name" = "MISC") "section" in
    let fvalue = fun name default->
      try ExtXml.float_attrib (ExtXml.child gcs_section ~select:(fun x -> ExtXml.attrib x "name" = name) "define") "value" with _ -> default in
    (fvalue "RC_MAX_RATE" 50.)
  with _ -> default_max_rate

let get_bat_levels = fun af_xml ->
  let default_catastrophic_level = 9.
  and default_max_level = 12.5
  and default_nb_cell = None in
  try
    let bat_section = ExtXml.child af_xml ~select:(fun x -> Xml.attrib x "name" = "BAT") "section" in
    let fvalue = fun name default ->
      try ExtXml.float_attrib (ExtXml.child bat_section ~select:(fun x -> ExtXml.attrib x "name" = name) "define") "value" with _ -> default in
    let fvalue_opt = fun name default ->
      try Some (ExtXml.float_attrib (ExtXml.child bat_section ~select:(fun x -> ExtXml.attrib x "name" = name) "define") "value") with _ -> default in
    fvalue "CATASTROPHIC_BAT_LEVEL" default_catastrophic_level, fvalue "MAX_BAT_LEVEL" default_max_level, fvalue_opt "BAT_NB_CELLS" default_nb_cell
  with _ -> (default_catastrophic_level, default_max_level, default_nb_cell)

let get_alt_shift = fun af_xml ->
  let default_plus_plus = 30.
  and default_plus = 5.
  and default_minus = -5. in
  try
    let gcs_section = ExtXml.child af_xml ~select:(fun x -> Xml.attrib x "name" = "GCS") "section" in
    let fvalue = fun name default ->
      try ExtXml.float_attrib (ExtXml.child gcs_section ~select:(fun x -> ExtXml.attrib x "name" = name) "define") "value" with _ -> default in
    fvalue "ALT_SHIFT_PLUS_PLUS" default_plus_plus,
    fvalue "ALT_SHIFT_PLUS" default_plus,
    fvalue "ALT_SHIFT_MINUS" default_minus
  with _ -> (default_plus_plus, default_plus, default_minus)

let get_speech_name = fun af_xml def_name ->
  let default_speech_name = def_name in
  try
    let gcs_section = ExtXml.child af_xml ~select:(fun x -> Xml.attrib x "name" = "GCS") "section" in
    let fvalue = fun name default ->
      try ExtXml.attrib (ExtXml.child gcs_section ~select:(fun x -> ExtXml.attrib x "name" = name) "define") "value" with _ -> default in
    fvalue "SPEECH_NAME" default_speech_name
  with _ -> default_speech_name

let get_icon_and_track_size = fun af_xml ->
  (* firmware name as default if fixedwing or rotorcraft *)
  let firmware = ExtXml.child af_xml "firmware" in
  let firmware_name = ExtXml.attrib firmware "name" in
  try
    (* search AC_ICON in GCS section *)
    let gcs_section = ExtXml.child af_xml ~select:(fun x -> Xml.attrib x "name" = "GCS") "section" in
    let ac_icon = ExtXml.child gcs_section ~select:(fun x -> ExtXml.attrib x "name" = "AC_ICON") "define" in
    match ExtXml.attrib ac_icon "value" with
    | "home" -> ("home", 1) (* no track for home icon *)
    | x -> (x, !track_size)
  with _ -> (firmware_name, !track_size)

let get_icons_theme = fun af_xml ->
  try
    let gcs_section = ExtXml.child af_xml ~select:(fun x -> Xml.attrib x "name" = "GCS") "section" in
    let fvalue = fun name default ->
      try ExtXml.attrib (ExtXml.child gcs_section ~select:(fun x -> ExtXml.attrib x "name" = name) "define") "value" with _ -> default in
    fvalue "ICONS_THEME" Env.gcs_default_icons_theme
  with _ -> Env.gcs_default_icons_theme

let key_press_event = fun keys do_action ev ->
  try
    let (modifiers, action) = List.assoc (GdkEvent.Key.keyval ev) keys in
    let ev_modifiers = GdkEvent.Key.state ev in
    if List.for_all (fun m -> List.mem m ev_modifiers) modifiers then begin
      do_action action;
      true
    end else
      false
  with
    | _ -> false




(*****************************************************************************)
let create_ac = fun ?(confirm_kill=true) alert (geomap:G.widget) (acs_notebook:GPack.notebook) (strips:GPack.box) (ac_id:string) config ->
  let color = PprzLink.string_assoc "default_gui_color" config
  and name = PprzLink.string_assoc "ac_name" config in

  (** Get the flight plan **)
  let fp_url = PprzLink.string_assoc "flight_plan" config in
  let fp_file = Http.file_of_url fp_url in
  let fp_xml_dump = ExtXml.parse_file ~noprovedtd:true fp_file in
  let stages = ExtXml.child fp_xml_dump "stages" in
  let blocks = blocks_of_stages stages in

  (** Get the airframe file *)
  let af_url = PprzLink.string_assoc "airframe" config in
  let af_file =  Http.file_of_url af_url in
  (* do not check dtd if it is a http url *)
  let via_http = Str.string_match (Str.regexp "http") af_url 0 in
  let af_xml = ExtXml.parse_file ~noprovedtd:via_http af_file in
  let af_xml = try Airframe.expand_includes ac_id af_xml with _ -> af_xml in

  (** Get an alternate speech name if available *)
  let speech_name = get_speech_name af_xml name in


  (* Aicraft menu decorated with a colored box *)
  let image = GBin.event_box ~width:10 ~height:10 () in
  image#coerce#misc#modify_bg [`NORMAL, `NAME color];
  let ac_mi = GMenu.image_menu_item ~label:name ~image ~packing:geomap#menubar#append () in

  let ac_menu = GMenu.menu () in
  ac_mi#set_submenu ac_menu;
  let ac_menu_fact = new GMenu.factory ac_menu in
  let fp_show = ac_menu_fact#add_check_item "Fligh Plan" ~active:true in
  ignore (fp_show#connect#toggled (fun () -> show_mission ac_id fp_show#active));

  let (icon, size) = get_icon_and_track_size af_xml in
  let track = new MapTrack.track ~size ~icon ~name ~color:color ac_id geomap in
  track#set_event_cb (select_ac acs_notebook);
  geomap#register_to_fit (track:>MapCanvas.geographic);

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

  let cam = ac_menu_fact#add_check_item "Cam footprint" ~active:false in
  ignore (cam#connect#toggled (fun () -> track#set_cam_state cam#active));
  let params = ac_menu_fact#add_check_item "A/C label" ~active:false in
  ignore (params#connect#toggled (fun () -> track#set_params_state params#active));

  (** Add a new tab in the A/Cs notebook, with a colored label *)
  let eb = GBin.event_box () in
  let _label = GMisc.label ~text:name ~packing:eb#add () in
  eb#coerce#misc#modify_bg [`NORMAL, `NAME color;`ACTIVE, `NAME color];

  (** Put a notebook for this A/C *)
  let ac_frame = GBin.frame () in
  ignore (acs_notebook#append_page ~tab_label:eb#coerce ac_frame#coerce);
  let ac_notebook = GPack.notebook ~packing: ac_frame#add () in
  let visible = fun w ->
    ac_notebook#page_num w#coerce = ac_notebook#current_page in

  (** Add a strip *)
  let min_bat, max_bat, nb_cell_bat = get_bat_levels af_xml in
  let alt_shift_plus_plus, alt_shift_plus, alt_shift_minus = get_alt_shift af_xml in
  let icons_theme = get_icons_theme af_xml in
  let param = { Strip.color = color;
                min_bat = min_bat; max_bat = max_bat; nb_cell_bat = nb_cell_bat;
                alt_shift_plus_plus = alt_shift_plus_plus;
                alt_shift_plus = alt_shift_plus;
                alt_shift_minus = alt_shift_minus;
                icons_theme = icons_theme; } in
  (*let strip = Strip.add config color min_bat max_bat in*)
  let strip = Strip.add config param strips in
  strip#connect (fun () -> select_ac acs_notebook ac_id);
  strip#connect_mark (mark geomap ac_id track !Plugin.frame);

  (** Build the XML flight plan, connect then "jump_to_block" *)
  let fp_xml = ExtXml.child fp_xml_dump "flight_plan" in
  let fp = load_mission ~editable:false color geomap fp_xml in
  fp#connect_activated (fun node ->
    if XmlEdit.tag node = "block" then
      let block = XmlEdit.attrib node "name" in
      let id = list_casso block blocks in
      jump_to_block ac_id id);
  ignore (reset_wp_menu#connect#activate (reset_waypoints fp));

   (** Monitor waypoints changes *)
  List.iter
    (fun w ->
      let (i, w) = fp#index w in
      w#set_commit_callback (fun () -> send_move_waypoint_msg ac_id i w))
    fp#waypoints;

   (** Add waypoints as geo references *)
  List.iter
    (fun w ->
      let (_i, w) = fp#index w in
      geomap#add_info_georef (sprintf "%s.%s" name w#name) (w :> < pos : Latlong.geographic >))
    fp#waypoints;

  (** Add the short cut buttons in the strip *)
  let tooltips = GData.tooltips () in
  let keys = ref [] in (* Associations between keys and block ids *)
  List.iter (fun block ->
    let id = ExtXml.int_attrib block "no" in
    begin (* Is it a key short cut ? *)
      try
        let key, modifiers = GtkData.AccelGroup.parse (Env.key_modifiers_of_string (Xml.attrib block "key")) in
        keys := (key, (modifiers, id)) :: !keys
      with
          _ -> ()
    end;
    try (* Is it a strip button ? *)
      let label = ExtXml.attrib block "strip_button"
      and block_name = ExtXml.attrib block "name"
      and group = ExtXml.attrib_or_default block "group" "" in
      let b =
        try (* Is it an icon ? *)
          let icon = Xml.attrib block "strip_icon" in
          let b = GButton.button () in
          let pixbuf = GdkPixbuf.from_file (Env.get_gcs_icon_path icons_theme icon) in
          ignore (GMisc.image ~pixbuf ~packing:b#add ());

      (* Drag for Drop *)
          let papget = Papget_common.xml "goto_block" "button"
            [ "block_name", block_name;
              "ac_id", ac_id;
              "icon", icons_theme // icon] in
          Papget_common.dnd_source b#coerce papget;

      (* Associates the label as a tooltip *)
          tooltips#set_tip b#coerce ~text:label;
          b
        with
            Xml.No_attribute _ -> (* It's not an icon *)
              GButton.button ~label ()
          | exc ->
            fprintf stderr "Error: '%s' Using a standard button" (Printexc.to_string exc);
            GButton.button ~label ()
      in
      strip#add_widget b#coerce ~group;
      ignore (b#connect#clicked (fun _ -> jump_to_block ac_id id))
    with
        _ -> ())
    (Xml.children (ExtXml.child (ExtXml.child fp_xml_dump "flight_plan") "blocks"));


  (** Handle key shortcuts for block selection *)
  let key_press = key_press_event !keys (fun block_id -> jump_to_block ac_id block_id) in
  ignore (geomap#canvas#event#connect#after#key_press key_press);


  (** Insert the flight plan tab *)
  let fp_label = GMisc.label ~text: "Flight Plan" () in
  ignore ((ac_notebook:GPack.notebook)#append_page ~tab_label:fp_label#coerce fp#window#coerce);

  let infrared_label = GMisc.label ~text: "Infrared" () in
  let infrared_frame = GBin.frame ~show:false ~shadow_type:`NONE () in
  ignore (ac_notebook#append_page ~tab_label: infrared_label#coerce infrared_frame#coerce);
  let ir_page = new Pages.infrared infrared_frame in

  let gps_label = GMisc.label ~text: "GPS" () in
  let gps_frame = GBin.frame ~shadow_type: `NONE () in
  ignore (ac_notebook#append_page ~tab_label: gps_label#coerce gps_frame#coerce);
  let gps_page = new Pages.gps ~visible gps_frame in

  let pfd_label = GMisc.label ~text: "PFD" () in
  let pfd_frame = GBin.frame ~shadow_type: `NONE () in
  ignore (ac_notebook#append_page ~tab_label: pfd_label#coerce pfd_frame#coerce);
  let pfd_page = new Horizon.pfd pfd_frame
  and _pfd_page_num = ac_notebook#page_num pfd_frame#coerce in

  let link_label = GMisc.label ~text: "Link" () in
  let link_frame = GBin.frame ~shadow_type: `NONE () in
  ignore (ac_notebook#append_page ~tab_label: link_label#coerce link_frame#coerce);
  let link_page = new Pages.link ~visible link_frame in

  let misc_label = GMisc.label ~text: "Misc" () in
  let misc_frame = GBin.frame ~shadow_type: `NONE () in
  ignore (ac_notebook#append_page ~tab_label:misc_label#coerce misc_frame#coerce);
  let misc_page = new Pages.misc ~packing:misc_frame#add misc_frame in

  let settings_url = PprzLink.string_assoc "settings" config in
  let settings_file = Http.file_of_url settings_url in
  let settings_xml =
    try
      if String.compare "replay" settings_file <> 0 then
        ExtXml.parse_file ~noprovedtd:true settings_file
      else
        Xml.Element("empty", [], [])
    with exc ->
      prerr_endline (Printexc.to_string exc);
      Xml.Element("empty", [], [])
  in
  let dl_setting_callback = fun idx value ->
    if classify_float value = FP_normal ||  classify_float value = FP_zero then
      dl_setting ac_id idx value
    else
      get_dl_setting ac_id idx
  in
  let dl_settings_page =
    try
      let xml_settings = Xml.children (ExtXml.child settings_xml "dl_settings") in
      let settings_tab = new Page_settings.settings ~visible xml_settings dl_setting_callback ac_id icons_theme (fun group x -> strip#add_widget ~group x) in

      (** Connect key shortcuts *)
      let key_press = fun ev ->
        key_press_event settings_tab#keys (fun commit -> commit ()) ev in
      ignore (geomap#canvas#event#connect#after#key_press key_press);

      let tab_label = GPack.hbox () in
      let _label = (GMisc.label ~text:"Settings" ~packing:tab_label#pack ()) in
      let button_save_settings = GButton.button ~packing:tab_label#pack () in
      ignore (GMisc.image ~stock:`SAVE ~packing:button_save_settings#add ());
      button_save_settings#set_border_width 0;
      ignore (button_save_settings#connect#clicked (fun () -> settings_tab#save af_file));
      ignore (ac_notebook#append_page ~tab_label:tab_label#coerce settings_tab#widget);
      Some settings_tab
    with exc ->
      log alert ac_id (Printexc.to_string exc);
      None in

  let rc_settings_page =
    try
      let xml_settings = Xml.children (ExtXml.child settings_xml "rc_settings") in
      if xml_settings = [] then
        raise Exit
      else
        let settings_tab = new Pages.rc_settings ~visible xml_settings in
        let tab_label = (GMisc.label ~text:"RC Settings" ())#coerce in
        ignore (ac_notebook#append_page ~tab_label settings_tab#widget);
        Some settings_tab
    with _ -> None in

  let rc_max_rate = get_rc_max_rate af_xml in

  let wp_HOME =
    let rec loop = function
    [] -> None
      | w::ws ->
        let (_i, w) = fp#index w in
        if w#name = "HOME" then Some w else loop ws in
    loop fp#waypoints in

  let ac = { track = track; color = color; last_dist_to_wp = 0.;
             fp_group = fp; fp_show = fp_show ; config = config ;
             wp_HOME = wp_HOME; fp = fp_xml;
             ac_name = name; ac_speech_name = speech_name;
             blocks = blocks; last_ap_mode= "";
             last_stage = (-1,-1);
             ir_page = ir_page; flight_time = 0;
             gps_page = gps_page;
             pfd_page = pfd_page;
             link_page = link_page;
             misc_page = misc_page;
             dl_settings_page = dl_settings_page;
             rc_settings_page = rc_settings_page;
             strip = strip; first_pos = true;
             rc_max_rate = rc_max_rate;
             last_block_name = ""; alt = 0.; target_alt = 0.;
             in_kill_mode = false; speed = 0.;
             wind_dir = 42.; ground_prox = true;
             wind_speed = 0.;
             pages = ac_frame#coerce;
             notebook_label = _label;
             got_track_status_timer = 1000;
             dl_values = [||]; last_unix_time = 0.;
             airspeed = 0.;
             version = "";
             last_gps_acc = GPS_NO_ACC;
             last_bat_warn_time = 0.
           } in
  Hashtbl.add aircrafts ac_id ac;
  select_ac acs_notebook ac_id;

  (** Periodically send the wind estimation through
      a WIND_INFO message packed into a RAW_DATALINK *)
  let send_wind = fun () ->
    if misc_page#periodic_send then begin
      (* FIXME: Disabling the timer would be preferable *)
      try
        let a = (pi/.2. -. ac.wind_dir)
        and w =  ac.wind_speed in

        let wind_east = sprintf "%.1f" (-. cos a *. w)
        and wind_north = sprintf "%.1f" (-. sin a *. w)
        and airspeed = sprintf "%.1f" ac.airspeed in

        (* only horizontal wind and airspeed are updated, so bitmask is 0b0000101 = 5 *)
        let msg_items = ["WIND_INFO"; ac_id; "5"; wind_east; wind_north; "0.0"; airspeed] in
        let value = String.concat ";" msg_items in
        let vs = ["ac_id", PprzLink.String ac_id; "message", PprzLink.String value] in
        Ground_Pprz.message_send "dl" "RAW_DATALINK" vs;
      with
          exc -> log alert ac_id (sprintf "send_wind (%s): %s" ac_id (Printexc.to_string exc))
    end;
    true
  in

  if is_int ac_id then
    ignore (Glib.Timeout.add 10000 send_wind);

  begin
    match dl_settings_page with
        Some settings_tab ->
    (** Connect the strip buttons *)
          let firmware = ExtXml.child af_xml "firmware" in
          let firmware_name = ExtXml.attrib firmware "name" in
          let connect = fun ?(warning=true) setting_name strip_connect ->
            try
              let id = settings_tab#assoc setting_name in
              if setting_name = "kill_throttle" then
                strip_connect (fun x -> (dl_setting_callback id x; dl_emergency_cmd ac_id 0))
              else
                strip_connect (fun x -> dl_setting_callback id x)
            with Not_found ->
              if setting_name = "kill_throttle" then
                strip_connect (fun x -> (if x = 1. then dl_emergency_cmd ac_id 0));
              if warning then
                fprintf stderr "Warning: %s not setable from GCS strip (i.e. not listed in the xml settings file)\n" setting_name in
          connect "flight_altitude" (fun f -> ac.strip#connect_shift_alt (fun x -> f (ac.target_alt+.x)));
          connect "autopilot.launch" ~warning:false ac.strip#connect_launch;
          connect "autopilot.kill_throttle" (ac.strip#connect_kill confirm_kill);
          (* try to connect either pprz_mode (fixedwing) or autopilot_mode (rotorcraft) *)
          begin match firmware_name with
          | "fixedwing" -> connect "autopilot.mode" ~warning:false (ac.strip#connect_mode 2.)
          | "rotorcraft" -> connect "autopilot.mode" ~warning:false (ac.strip#connect_mode 13.)
          | _ -> ()
          end;
          connect "nav_shift" ~warning:false  ac.strip#connect_shift_lateral;
          connect "autopilot.flight_time" ac.strip#connect_flight_time;
          let get_ac_unix_time = fun () -> ac.last_unix_time in
          connect ~warning:false "snav_desired_tow" (ac.strip#connect_apt get_ac_unix_time);
          begin (* Periodically update the appointment *)
            try
              let id = settings_tab#assoc "snav_desired_tow" in
              let set_appointment = fun _ ->
                begin try
                        let v = match ac.dl_values.(id) with None -> raise Not_found | Some x -> int_of_string x in
                        let t = Unix.gmtime (Latlong.unix_time_of_tow v) in
                        ac.strip#set_label "apt" (sprintf "%d:%02d:%02d" t.Unix.tm_hour t.Unix.tm_min t.Unix.tm_sec)
                  with _ -> () end;
                true
              in
              ignore (Glib.Timeout.add 1000 set_appointment)
            with Not_found -> ()
          end;

    (** Connect the GPS reset button *)
          begin
            try
              let gps_reset_id = settings_tab#assoc "gps.reset" in
              gps_page#connect_reset
                (fun x -> dl_setting_callback gps_reset_id (float x))
            with Not_found -> ()
          end
      | None -> ()
  end;

  (* Monitor track status *)
  let monitor_track_status = fun () ->
    ac.got_track_status_timer <- ac.got_track_status_timer + 1;
    if ac.got_track_status_timer > 5 then
      ac.track#delete_desired_track ();
    true in
  ignore (Glib.Timeout.add 1000 monitor_track_status);;


(* since tcl8.6 "green" refers to "darkgreen" and the former "green" is now "lime", but that is not available in older versions, so hardcode the color to #00ff00*)
let ok_color = "#00ff00"
let warning_color = "orange"
let alert_color = "red"

(** Bind to message while catching all the esceptions of the callback *)
let safe_bind = fun msg cb ->
  let safe_cb = fun sender vs ->
    try cb sender vs with
        AC_not_found -> () (* A/C not yet registed; silently ignore *)
      | x -> fprintf stderr "%s: safe_bind (%s:%a): %s\n%!" Sys.argv.(0) msg (fun c vs -> List.iter (fun (_,v) -> fprintf c "%s " (PprzLink.string_of_value v)) vs) vs (Printexc.to_string x) in
  ignore (Ground_Pprz.message_bind msg safe_cb)

let alert_bind = fun msg cb ->
  let safe_cb = fun sender vs ->
    try cb sender vs with _ -> () in
  ignore (Alert_Pprz.message_bind msg safe_cb)

let tele_bind = fun msg cb timestamp ->
  let safe_cb = fun sender vs ->
    try cb sender vs with
        AC_not_found -> () (* A/C not yet registed; silently ignore *)
      | x -> fprintf stderr "tele_bind (%s): %s\n%!" msg (Printexc.to_string x) in
  ignore (Tele_Pprz.message_bind ~timestamp msg safe_cb)

let ask_config = fun confirm_kill alert geomap fp_notebook strips ac ->
  let get_config = fun _sender values ->
    if not (Hashtbl.mem aircrafts ac) then
      create_ac ~confirm_kill alert geomap fp_notebook strips ac values
  in
  ignore(Ground_Pprz.message_req "gcs" "CONFIG" ["ac_id", PprzLink.String ac] get_config)



let one_new_ac = fun confirm_kill alert (geomap:G.widget) fp_notebook strips ac ->
  if (List.length !filter_acs = 0) || (List.mem ac !filter_acs) && not (Hashtbl.mem aircrafts ac) then
    ask_config confirm_kill alert geomap fp_notebook strips ac


let get_wind_msg = fun (geomap:G.widget) _sender vs ->
  let ac = get_ac vs in
  let value = fun field_name -> PprzLink.float_assoc field_name vs in
  let airspeed = value "mean_aspeed" in
  ac.airspeed <- airspeed;
  ac.strip#set_airspeed airspeed;
  ac.misc_page#set_value "Mean airspeed" (sprintf "%.1f" airspeed);
  ac.wind_speed <- value "wspeed";
  let deg_dir = value "dir" in
  ac.wind_dir <- (Deg>>Rad)deg_dir;
  ac.misc_page#set_value "Wind speed" (sprintf "%.1f" ac.wind_speed);
  ac.misc_page#set_value "Wind direction" (sprintf "%.1f" deg_dir);

  let ac_id = PprzLink.string_assoc "ac_id" vs in
  if !active_ac = ac_id && ac.wind_speed > 1. then begin
    geomap#wind_sock#set_color ac.color;
    geomap#wind_sock#item#show ();
    geomap#set_wind_sock deg_dir (sprintf "%.1f" ac.wind_speed)
  end


let get_fbw_msg = fun alarm _sender vs ->
  let ac = get_ac vs in
  let status = PprzLink.string_assoc "rc_status" vs
  and rate = (float_of_int ((PprzLink.int_assoc "rc_rate" vs) * 10) ) /. ac.rc_max_rate in
  (* divide by 5 to have normal values between 0 and 10 *)
  (* RC rate max approx. 50 Hz *)
  ac.strip#set_rc rate status;
  let mode = PprzLink.string_assoc "rc_mode" vs in
  if mode = "FAILSAFE" then begin
    log_and_say alarm ac.ac_name (sprintf "%s, mayday, AP Failure. Switch to manual." ac.ac_speech_name)
  end

let get_telemetry_status = fun alarm _sender vs ->
  let ac = get_ac vs in
  let link_id = PprzLink.string_assoc "link_id" vs in
  let link_id = try if int_of_string link_id = -1 then "single" else link_id with _ -> link_id in
  (* Update color and lost time in the strip *)
  let time_lost = PprzLink.float_assoc "time_since_last_msg" vs in
  let (links_up, total_links) = ac.link_page#links_ratio () in
  let link_ratio_string =
    if ac.link_page#multiple_links () then sprintf "%i/%i" links_up total_links else "" in
  ac.strip#set_label "telemetry_status" (if time_lost > 2. then sprintf "%.0f" time_lost else link_ratio_string);
  ac.strip#set_color "telemetry_status" (if time_lost > 5. then alert_color else if links_up < total_links then warning_color else ok_color);
  (* Update link page *)
  let rx_msgs_rate = PprzLink.float_assoc "rx_bytes_rate" vs
  and downlink_bytes_rate = PprzLink.int_assoc "downlink_rate" vs
  and uplink_lost_time = PprzLink.int_assoc "uplink_lost_time" vs in
  let ping_time = PprzLink.float_assoc "ping_time" vs in
  if (not (ac.link_page#link_exists link_id)) then begin
      ac.link_page#add_link link_id;
      log_and_say alarm ac.ac_name (sprintf "%s, link %s detected" ac.ac_speech_name link_id)
    end;
  let link_changed = ac.link_page#update_link link_id time_lost ping_time rx_msgs_rate downlink_bytes_rate uplink_lost_time in
  let (links_up, _) = ac.link_page#links_ratio () in
  match (link_changed, links_up) with
    (_, 0) -> log_and_say alarm ac.ac_name (sprintf "%s, all links lost" ac.ac_speech_name)
  | (Pages.Linkup, _)-> log_and_say alarm ac.ac_name (sprintf "%s, link %s re-connected" ac.ac_speech_name link_id)
  | (Pages.Nochange, _) -> ()
  | (Pages.Linkdown, _) -> log_and_say alarm ac.ac_name (sprintf "%s, link %s lost" ac.ac_speech_name link_id)

let get_engine_status_msg = fun _sender vs ->
  let ac = get_ac vs in
  ac.strip#set_throttle ~kill:ac.in_kill_mode (PprzLink.float_assoc "throttle" vs);
  ac.strip#set_bat (PprzLink.float_assoc "bat" vs)

let get_if_calib_msg = fun _sender vs ->
  let ac = get_ac vs in
  match ac.rc_settings_page with
      None -> ()
    | Some p ->
      p#set_rc_setting_mode (PprzLink.string_assoc "if_mode" vs);
      p#set (PprzLink.float_assoc "if_value1" vs) (PprzLink.float_assoc "if_value2" vs)

let listen_wind_msg = fun (geomap:G.widget) ->
  safe_bind "WIND" (get_wind_msg geomap)

let listen_fbw_msg = fun a ->
  safe_bind "FLY_BY_WIRE" (get_fbw_msg a)

let listen_engine_status_msg = fun () ->
  safe_bind "ENGINE_STATUS" get_engine_status_msg

let listen_if_calib_msg = fun () ->
  safe_bind "INFLIGH_CALIB" get_if_calib_msg

let listen_telemetry_status = fun a ->
  safe_bind "TELEMETRY_STATUS" (get_telemetry_status a)

let aircrafts_msg = fun confirm_kill alert (geomap:G.widget) fp_notebook strips acs ->
  let acs = PprzLink.string_assoc "ac_list" acs in
  let acs = Str.split list_separator acs in
  List.iter (one_new_ac confirm_kill alert geomap fp_notebook strips) acs


let listen_dl_value = fun () ->
  let get_dl_value = fun _sender vs ->
    let ac = get_ac vs in
    match ac.dl_settings_page with
        Some settings ->
          let csv = PprzLink.string_assoc "values" vs in
          let string_value = fun v -> match v with "?" -> None | _ -> Some v in
          let values = Array.map string_value (Array.of_list (Str.split list_separator csv)) in
          ac.dl_values <- values;
          for i = 0 to min (Array.length values) settings#length - 1 do
            try
              settings#set i values.(i)
            with _ -> ()
          done
      | None -> () in
  safe_bind "DL_VALUES" get_dl_value


let highlight_fp = fun ac b s ->
  if (b, s) <> ac.last_stage then begin
    ac.last_stage <- (b, s);
    ac.fp_group#highlight_stage b s
  end


let check_approaching = fun ac geo1 geo2 alert ->
  match ac.track#last with
      None -> ()
    | Some ac_pos ->
      let s_len = LL.wgs84_distance geo1 geo2 in (* length of the segment *)
      let d = LL.wgs84_distance ac_pos geo2 in (* distance to end of the segment *)
      (* only log_and_say "approaching" if close enough but not too much and when flying long segments *)
      if d < ac.speed *. approaching_alert_time && d > approaching_alert_dmin && s_len > approaching_alert_slmin then
        log_and_say alert ac.ac_name (sprintf "%s, approaching" ac.ac_speech_name)


let ac_alt_graph = [14,0;-5,0;-7,-6]
let translate = fun l dx dy -> List.map (fun (x, y) -> (x + dx, y + dy)) l
let rotate_and_translate = fun l angle dx dy ->
  translate (List.map (fun (x, y) -> (
    truncate ((cos angle) *. (float x) -. (sin angle) *. (float y)),
    truncate ((sin angle) *. (float x) +. (cos angle) *. (float y)))
  ) l) dx dy
let flip = fun l -> List.map (fun (x, y) -> (-x, y)) l

let draw_altgraph = fun (da_object:Gtk_tools.pixmap_in_drawin_area) (geomap:MapCanvas.widget) aircrafts ->
  (** First estimate the coverage of the window *)
  let width_c, height_c = Gdk.Drawable.get_size geomap#canvas#misc#window in
  let (xc0, yc0) = geomap#canvas#get_scroll_offsets in
  let (east, _y0) = geomap#window_to_world (float xc0) (float (yc0+height_c))
  and (west, _y1) = geomap#window_to_world (float (xc0+width_c)) (float yc0) in

  let da = da_object#drawing_area in
  let width, height = Gdk.Drawable.get_size da#misc#window in
  let dr = da_object#get_pixmap () in
  dr#set_background `BLACK;
  dr#set_foreground `BLACK;

  (* Background *)
  dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();

  (* Text *)
  let context = da#misc#create_pango_context in
  let print_string = fun x y string color ->
    let layout = context#create_layout in
    let from_codeset = "ISO-8859-15"
    and to_codeset = "UTF-8" in
    Pango.Layout.set_text layout (Glib.Convert.convert ~from_codeset ~to_codeset string);
    let (_w,h) = Pango.Layout.get_pixel_size layout in
    dr#put_layout ~x ~y:(y-h) ~fore:(`NAME color) layout in

  (* find min and max alt *)
  let max_alt = ref 0
  and min_alt = ref 35786000 in
  Hashtbl.iter (fun _ac_id ac ->
    let track = ac.track in
    let alt = (truncate track#last_altitude) in
    let ground_alt = alt - (truncate (track#height ())) in
    if ground_alt < !min_alt then min_alt := ground_alt;
    if alt > !max_alt then max_alt := alt
  ) aircrafts;
  min_alt := min !min_alt !max_alt;
  if (!min_alt mod lines_height) < (min 10 (lines_height / 2)) then
    min_alt := (!min_alt / lines_height - 1) * lines_height
  else
    min_alt := (!min_alt / lines_height) * lines_height;
  min_alt := max !min_alt 0;
  let height_alt = max (!max_alt - !min_alt + 10) min_height in

  (* lines *)
  dr#set_foreground (`NAME "grey");
  let n = height_alt / lines_height in
  for i = 0 to n do
    let y = height - i * height / n in
    dr#line ~x:0 ~y ~x:width ~y;
    print_string 6 y (sprintf "%d" (!min_alt + i * lines_height)) "red";
  done;

  (* aircrafts *)
  Hashtbl.iter (fun _ac_id ac ->
    dr#set_foreground (`NAME ac.color);
    let track = ac.track in
    match track#last with
        Some pos ->
          let (xac, _yac) = geomap#world_of pos in
          let w = float width in
          let eac = (truncate (w *. (xac -. east) /. (west -. east))) in
          let alt = (truncate track#last_altitude) in
          let aac = height - height * (alt - !min_alt) / height_alt in
          let h = track#last_heading in
          let climb_angle = ref 0. in
          if track#last_speed > 0. then
            climb_angle := (atan2 track#last_climb track#last_speed);

          dr#set_line_attributes ~width:4 ~cap:`ROUND ();
          dr#set_foreground (`NAME "white");
          if h > 0. && h <= 180. then begin
            dr#lines (rotate_and_translate ac_alt_graph (-. !climb_angle) eac aac);
            dr#set_line_attributes ~width:2 ();
            dr#set_foreground (`NAME ac.color);
            dr#lines (rotate_and_translate ac_alt_graph (-. !climb_angle) eac aac);
          end
          else begin
            dr#lines (rotate_and_translate (flip ac_alt_graph) !climb_angle eac aac);
            dr#set_line_attributes ~width:2 ();
            dr#set_foreground (`NAME ac.color);
            dr#lines (rotate_and_translate (flip ac_alt_graph) !climb_angle eac aac);
          end;

      (* altitude from ground if available *)
          let alt_from_ground = truncate (track#height ()) in
          let gac = aac + height * alt_from_ground / height_alt in
          dr#set_line_attributes ~width:1 ~cap:`NOT_LAST ();
          dr#line ~x:eac ~y:aac ~x:eac ~y:gac;

      (* history *)
          let v_path = track#v_path in
          for i = 0 to Array.length v_path - 1 do
            let (x, _y) = geomap#world_of (fst v_path.(i)) in
            let e = (truncate (w *. (x -. east) /. (west -. east))) in
            let a = height - height * ((truncate (snd v_path.(i))) - !min_alt) / height_alt in
            dr#point ~x:e ~y:a;
          done
      | None -> ()
  ) aircrafts;

  (new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap


(****** Display the position provided by a connected GPS receiver (../tmtc/gpsd2ivy) *******************)
module GCS_icon = struct
  let status = ref None
  let color = "black"
  let fill_color = "brown"
  let radius = 5.
  let outdated_color = "red"
  let timeout = 10000 (* ms : time before changing to outdated color *)

  let display = fun (geomap:G.widget) vs ->
    let lat = PprzLink.float_assoc "lat" vs
    and lon = PprzLink.float_assoc "long" vs in
    let wgs84 = LL.make_geo_deg lat lon in

    let item =
      match !status with
          None -> (* First call, create the graphical object *)
            let item = GnoCanvas.ellipse ~fill_color ~props:[`WIDTH_PIXELS 2]
              ~x1: ~-.radius ~y1:  ~-.radius ~x2:radius ~y2:radius
              geomap#canvas#root in
            (* connect callback on zoom change *)
            ignore(geomap#zoom_adj#connect#value_changed (fun () ->
              match !status with
              | None -> ()
              | Some (item, _, wgs84) -> geomap#move_item ~z:geomap#current_zoom item wgs84
            ));
            item
        | Some (item, timeout_handle, _) -> (* Remove the timeouted color modification *)
          Glib.Timeout.remove timeout_handle;
          item in

    item#set [`OUTLINE_COLOR color];
    let change_color_if_not_updated =
      Glib.Timeout.add 10000 (fun ()  -> item#set [`OUTLINE_COLOR outdated_color]; false) in

    (* Store the object, the position and the timeout to change its color *)
    status := Some (item, change_color_if_not_updated, wgs84);

    geomap#move_item ~z:geomap#current_zoom item wgs84
end (* module GCS_icon *)


(******************************** FLIGHT_PARAMS ******************************)
let listen_flight_params = fun geomap auto_center_new_ac auto_center_ac alert alt_graph ->
  let get_fp = fun _sender vs ->
    let ac_id = PprzLink.string_assoc "ac_id" vs in
    if ac_id = gcs_id then
      GCS_icon.display geomap vs
    else
      let ac = get_ac vs in
      let pfd_page = ac.pfd_page in
      let a = fun s -> PprzLink.float_assoc s vs in
      let alt = a "alt"
      and climb = a "climb"
      and speed = a "speed" in
      pfd_page#set_attitude (a "roll") (a "pitch");
      pfd_page#set_alt alt;
      pfd_page#set_climb climb;
      pfd_page#set_speed speed;

      let wgs84 = { posn_lat=(Deg>>Rad)(a "lat"); posn_long = (Deg>>Rad)(a "long") } in
      ac.track#move_icon wgs84 (a "heading") alt speed climb;
      ac.speed <- speed;

      let unix_time = a "unix_time" in
      if unix_time > ac.last_unix_time then begin
        let utc = Unix.gmtime unix_time in
        geomap#set_utc_time utc.Unix.tm_hour utc.Unix.tm_min utc.Unix.tm_sec;
        ac.last_unix_time <- unix_time
      end;

      if auto_center_new_ac && ac.first_pos then begin
        center geomap ac.track ();
        ac.first_pos <- false
      end;
      if auto_center_ac = ac_id then begin
        center geomap ac.track ();
      end;

      let set_label = fun lbl_name value ->
        ac.strip#set_label lbl_name (sprintf "%.0fm" value)
      in
      set_label "altitude" alt;
      ac.strip#set_speed speed;
      ac.strip#set_climb climb;
      let agl = (a "agl") in
      ac.alt <- alt;
      ac.strip#set_agl agl;
      if not ac.ground_prox && ac.flight_time > 10 && agl < 20. then begin
        log_and_say alert ac.ac_name (sprintf "%s, %s" ac.ac_speech_name "Ground Proximity Warning");
        ac.ground_prox <- true
      end else if agl > 25. then
          ac.ground_prox <- false;
      try
        if not (alt_graph#drawing_area#misc#parent = None) then
          draw_altgraph alt_graph geomap aircrafts
      with _ -> ()

  in
  safe_bind "FLIGHT_PARAM" get_fp;

  let get_ns = fun _sender vs ->
    let ac = get_ac vs in
    let a = fun s -> PprzLink.float_assoc s vs in
    let wgs84 = { posn_lat = (Deg>>Rad)(a "target_lat"); posn_long = (Deg>>Rad)(a "target_long") } in
    ac.track#move_carrot wgs84;
    let cur_block = PprzLink.int_assoc "cur_block" vs
    and cur_stage = PprzLink.int_assoc "cur_stage" vs in
    highlight_fp ac cur_block cur_stage;
    let set_label = fun l f ->
      ac.strip#set_label l (sprintf "%.0fm" (PprzLink.float_assoc f vs)) in
    set_label "target_altitude" "target_alt";
    let target_alt = PprzLink.float_assoc "target_alt" vs in
    ac.strip#set_label "diff_target_alt" (sprintf "%+.0fm" (ac.alt -. target_alt));
    ac.target_alt <- target_alt;
    let b = try List.assoc cur_block ac.blocks with Not_found -> failwith (sprintf "Error: unknown block %d for A/C %s" cur_block ac.ac_name) in
    if b <> ac.last_block_name then begin
      log_and_say alert ac.ac_name (sprintf "%s, %s" ac.ac_speech_name b);
      ac.last_block_name <- b;
      ac.strip#set_label "block_name" b
    end;
    let block_time = Int64.to_int (PprzLink.uint32_assoc "block_time" vs)
    and stage_time = Int64.to_int (PprzLink.uint32_assoc "stage_time" vs) in
    let bt = sprintf "%02d:%02d" (block_time / 60) (block_time mod 60) in
    ac.strip#set_label "block_time" bt;
    let st = sprintf "%02d:%02d" (stage_time / 60) (stage_time mod 60) in
    ac.strip#set_label "stage_time" st;

    (* Estimated Time Arrival to next waypoint *)
    let d = PprzLink.float_assoc "dist_to_wp" vs in
    let label =
      if d < 0.5 || ac.speed < 0.5 then
        "N/A"
      else
        sprintf "%.0fs" (d /. ac.speed) in
    ac.strip#set_label "eta_time" label;
    ac.last_dist_to_wp <- d;

    (* Estimated Time to HOME *)
    try
      match ac.wp_HOME with
          Some wp_HOME ->
            let (bearing_to_HOME_deg, d) = Latlong.bearing ac.track#pos wp_HOME#pos in
            let bearing_to_HOME = (Deg>>Rad)bearing_to_HOME_deg in
            let wind_north = -. ac.wind_speed *. cos ac.wind_dir
            and wind_east = -. ac.wind_speed *. sin ac.wind_dir in
            let c = ac.wind_speed *. ac.wind_speed -. ac.airspeed *. ac.airspeed
            and scal = wind_east *. sin bearing_to_HOME +. wind_north *. cos bearing_to_HOME in
            let delta = 4. *. (scal*.scal -. c) in
            let ground_speed_to_HOME = scal +. sqrt delta /. 2. in
            let time_to_HOME = d /. ground_speed_to_HOME in
            ac.misc_page#set_value "Time to HOME" (sprintf "%.0fs" time_to_HOME)
        | _ -> ()
    with
        _NotSoImportant -> ()
  in
  safe_bind "NAV_STATUS" get_ns;

  let get_cam_status = fun _sender vs ->
    let ac = get_ac vs in
    let a = fun s -> PprzLink.float_assoc s vs in
    let lats_str = PprzLink.string_assoc "lats" vs in
    let longs_str = PprzLink.string_assoc "longs" vs in
    
    let lats = List.map float_of_string (Str.split list_separator lats_str) in
    let longs = List.map float_of_string (Str.split list_separator longs_str) in

    let target_wgs84 = { posn_lat = (Deg>>Rad)(a "cam_target_lat"); posn_long = (Deg>>Rad)(a "cam_target_long") } in
    
    let geo_1 = { posn_lat = (Deg>>Rad)(List.nth lats 0); posn_long = (Deg>>Rad)(List.nth longs 0) } 
    and geo_2 = { posn_lat = (Deg>>Rad)(List.nth lats 1); posn_long = (Deg>>Rad)(List.nth longs 1) } 
    and geo_3 = { posn_lat = (Deg>>Rad)(List.nth lats 2); posn_long = (Deg>>Rad)(List.nth longs 2) } 
    and geo_4 = { posn_lat = (Deg>>Rad)(List.nth lats 3); posn_long = (Deg>>Rad)(List.nth longs 3) } in

    ac.track#move_cam [|geo_1; geo_2; geo_3; geo_4|] target_wgs84
  in
  safe_bind "CAM_STATUS" get_cam_status;

  let get_circle_status = fun _sender vs ->
    let ac = get_ac vs in
    ac.got_track_status_timer <- 0;
    let a = fun s -> PprzLink.float_assoc s vs in
    let wgs84 = { posn_lat = (Deg>>Rad)(a "circle_lat"); posn_long = (Deg>>Rad)(a "circle_long") } in
    ac.track#draw_circle wgs84 (float_of_string (PprzLink.string_assoc "radius" vs))
  in
  safe_bind "CIRCLE_STATUS" get_circle_status;

  let get_segment_status = fun _sender vs ->
    let ac = get_ac vs in
    ac.got_track_status_timer <- 0;
    let a = fun s -> PprzLink.float_assoc s vs in
    let geo1 = { posn_lat = (Deg>>Rad)(a "segment1_lat"); posn_long = (Deg>>Rad)(a "segment1_long") }
    and geo2 = { posn_lat = (Deg>>Rad)(a "segment2_lat"); posn_long = (Deg>>Rad)(a "segment2_long") } in
    ac.track#draw_segment geo1 geo2;

    (* Check if approaching the end of the segment *)
    check_approaching ac geo1 geo2 alert
  in
  safe_bind "SEGMENT_STATUS" get_segment_status;


  let get_survey_status = fun _sender vs ->
    let ac = get_ac vs in
    let a = fun s -> PprzLink.float_assoc s vs in
    let geo1 = { posn_lat = (Deg>>Rad)(a "south_lat"); posn_long = (Deg>>Rad)(a "west_long") }
    and geo2 = { posn_lat = (Deg>>Rad)(a "north_lat"); posn_long = (Deg>>Rad)(a "east_long") } in
    ac.track#draw_zone geo1 geo2
  in
  safe_bind "SURVEY_STATUS" get_survey_status;


  let get_ap_status = fun _sender vs ->
    let ac = get_ac vs in
    let flight_time = Int64.to_int (PprzLink.uint32_assoc "flight_time" vs) in
    ac.track#update_ap_status (float_of_int flight_time);
    ac.flight_time <- flight_time;
    let ap_mode = PprzLink.string_assoc "ap_mode" vs in
    if ap_mode <> ac.last_ap_mode then begin
      log_and_say alert ac.ac_name (sprintf "%s, %s" ac.ac_speech_name ap_mode);
      ac.last_ap_mode <- ap_mode;
      let label = PprzLink.string_assoc "ap_mode" vs in
      ac.strip#set_label "AP" (if label="MANUAL" then "MANU" else label);
      let color =
        match ap_mode with
            "AUTO2" | "NAV" -> ok_color
          | "AUTO1" | "R_RCC" | "A_RCC" | "ATT_C" | "R_ZH" | "A_ZH" | "HOVER" | "HOV_C" | "H_ZH" | "MODULE" -> "#10F0E0"
          | "MANUAL" | "RATE" | "ATT" | "RC_D" | "CF" | "FWD" | "FLIP" | "GUIDED" -> warning_color
          | _ -> alert_color in
      ac.strip#set_color "AP" color;
    end;
    let status_filter_mode = PprzLink.string_assoc "state_filter_mode" vs in
    let gps_mode =
      if (status_filter_mode <> "UNKNOWN") && (status_filter_mode <> "OK") && (status_filter_mode <> "GPS_LOST")
      then status_filter_mode
      else PprzLink.string_assoc "gps_mode" vs in
    ac.strip#set_label "GPS" gps_mode;
    ac.strip#set_color "GPS" (if gps_mode<>"3D" && gps_mode<>"DGPS" && gps_mode<>"RTK" then alert_color else ok_color);
    let ft =
      sprintf "%02d:%02d:%02d" (flight_time / 3600) ((flight_time / 60) mod 60) (flight_time mod 60) in
    ac.strip#set_label "flight_time" ft;
    let kill_mode = PprzLink.string_assoc "kill_mode" vs in
    if kill_mode <> "OFF" then  begin
      if not ac.in_kill_mode then
        log_and_say alert ac.ac_name (sprintf "%s, mayday, kill mode" ac.ac_speech_name);
      ac.in_kill_mode <- true
    end else
      ac.in_kill_mode <- false;
    match ac.rc_settings_page with
        None -> ()
      | Some p ->
        p#set_rc_mode ap_mode
  in
  safe_bind "AP_STATUS" get_ap_status;

  listen_dl_value ();;

let listen_waypoint_moved = fun () ->
  let get_values = fun _sender vs ->
    let ac = get_ac vs in
    let wp_id = PprzLink.int_assoc "wp_id" vs in
    let a = fun s -> PprzLink.float_assoc s vs in
    let geo = { posn_lat = (Deg>>Rad)(a "lat"); posn_long = (Deg>>Rad)(a "long") }
    and altitude = a "alt"
    and ground_alt = a "ground_alt" in

    try
      let w = ac.fp_group#get_wp wp_id in
      w#set_ground_alt ground_alt;
      w#set ~altitude ~update:true geo;
      ac.fp_group#update_sectors w#name
    with
        Not_found -> () (* Silently ignore unknown waypoints *)
  in
  safe_bind "WAYPOINT_MOVED" get_values

let get_alert_bat_low = fun a _sender vs ->
  let ac = get_ac vs in
  let level = PprzLink.string_assoc "level" vs in
  let unix_time = Unix.gettimeofday() in
  if unix_time > (ac.last_bat_warn_time +. 10.) then begin
    log_and_say a ac.ac_name (sprintf "%s, %s %s" ac.ac_speech_name "BAT LOW" level);
    ac.last_bat_warn_time <- unix_time
  end

let listen_alert = fun a ->
  alert_bind "BAT_LOW" (get_alert_bat_low a)

let get_svsinfo = fun alarm _sender vs ->
  let ac = get_ac vs in
  let gps_page = ac.gps_page in
  let svids = Str.split list_separator (PprzLink.string_assoc "svid" vs)
  and cn0s = Str.split list_separator (PprzLink.string_assoc "cno" vs)
  and flagss = Str.split list_separator (PprzLink.string_assoc "flags" vs)
  and ages = Str.split list_separator (PprzLink.string_assoc "msg_age" vs) in

  let a = Array.make (List.length svids) (0,0,0,0) in
  let rec loop = fun i s c f ages ->
    match (s, c, f, ages) with
        [], [], [], [] -> ()
      | s::ss, c::cs, f::fs, age::ages ->
        a.(i) <- (int_of_string s, int_of_string c, int_of_string f, int_of_string age);
        loop (i+1) ss cs fs ages
      | _ -> assert false in
  loop 0 svids cn0s flagss ages;

  let pacc = PprzLink.int_assoc "pacc" vs in

  gps_page#svsinfo pacc a;

  let new_acc =
    if pacc <= 1000 then GPS_ACC_HIGH
    else if pacc > 1000 && pacc < 2000 then GPS_ACC_LOW
    else if pacc > 999 then GPS_NO_ACC else GPS_ACC_VERY_LOW in
  if ac.last_gps_acc <> new_acc then begin
    match new_acc, ac.last_gps_acc with
    | GPS_ACC_HIGH, GPS_NO_ACC -> () (* nothing if pacc is good from the start *)
    | GPS_ACC_HIGH, _ -> log_and_say alarm "gcs" (sprintf "%s, GPS accuracy better than 10 meter" ac.ac_speech_name)
    | GPS_ACC_LOW, _ -> log_and_say alarm "gcs" (sprintf "%s, low GPS accuracy" ac.ac_speech_name)
    | GPS_ACC_VERY_LOW, _ -> log_and_say alarm "gcs" (sprintf "%s, Warning: GPS accuracy worse than 20 meter" ac.ac_speech_name)
    | _, _ -> ()
  end;
  ac.last_gps_acc <- new_acc

let listen_svsinfo = fun a -> safe_bind "SVSINFO" (get_svsinfo a)

let message_request = Ground_Pprz.message_req

let mark_dcshot = fun (geomap:G.widget) _sender vs ->
  let ac = find_ac !active_ac in
  let photonumber = PprzLink.string_assoc "photo_nr" vs in
  try
    let lat = PprzLink.int_assoc "lat" vs
    and lon = PprzLink.int_assoc "lon" vs in
    let wgs84 = LL.make_geo_deg (float lat /. 1e7) (float lon /. 1e7) in
    let group = geomap#background in
    let point = geomap#photoprojection ~group ~fill_color:"yellow" ~number:photonumber wgs84 3. in
    point#raise_to_top ()
  with _ ->
    match ac.track#last with
        Some geo ->
          begin
            let group = geomap#background in
            let point = geomap#photoprojection ~group ~fill_color:"yellow" ~number:photonumber geo 3. in
            point#raise_to_top ()
          end
      | None -> ()

(*  mark geomap ac.ac_name track !Plugin.frame *)


let listen_dcshot = fun _geom timestamp ->
  tele_bind "DC_SHOT" (mark_dcshot _geom) timestamp

let listen_error = fun a ->
  let get_error = fun _sender vs ->
    let msg = PprzLink.string_assoc "message" vs in
    log_and_say a "gcs" msg in
  safe_bind "TELEMETRY_ERROR" get_error

let listen_info_msg = fun a timestamp ->
  let get_msg = fun a _sender vs ->
    let ac = find_ac _sender in
    let msg_array = PprzLink.assoc "msg" vs in
    log_and_say a ac.ac_name (PprzLink.string_of_value msg_array) in
  tele_bind "INFO_MSG" (get_msg a) timestamp

let listen_autopilot_version_msg = fun a timestamp ->
  let get_msg = fun a _sender vs ->
    let ac = find_ac _sender in
    let desc_array = PprzLink.assoc "desc" vs in
    let version = PprzLink.string_of_value desc_array in
    if ac.version <> version then
      log a ac.ac_name (sprintf "%s version:\n%s" ac.ac_name version);
    ac.version <- version in
  tele_bind "AUTOPILOT_VERSION" (get_msg a) timestamp

let listen_tcas = fun a timestamp ->
  let get_alarm_tcas = fun a txt _sender vs ->
    let ac = find_ac _sender in
    let other_ac = get_ac vs in
    let resolve = try
                    match PprzLink.int_assoc "resolve" vs with
                        1 -> "=> LEVEL"
                      | 2 -> "=> CLIMB"
                      | 3 -> "=> DESCEND"
                      | _ -> ""
      with _ -> "" in
    log_and_say a ac.ac_name (sprintf "%s : %s -> %s %s" txt ac.ac_speech_name other_ac.ac_speech_name resolve)
  in
  tele_bind "TCAS_TA" (get_alarm_tcas a "tcas TA") timestamp;
  tele_bind "TCAS_RA" (get_alarm_tcas a "TCAS RA") timestamp

let get_intruders = fun (geomap:G.widget) _sender vs ->
  let f = fun s -> PprzLink.float_assoc s vs in
  let i = fun s -> float (PprzLink.int_assoc s vs) in
  let name = PprzLink.string_assoc "name" vs
  and id = PprzLink.string_assoc "id" vs
  and time = Unix.gettimeofday ()
  and lat = (i "lat") /. 1e7
  and lon = (i "lon") /. 1e7 in
  let pos = { posn_lat=(Deg>>Rad)lat; posn_long=(Deg>>Rad)lon } in
  if not (Intruders.intruder_exist id) then
    Intruders.new_intruder id name time geomap;
  Intruders.update_intruder id pos (f "course") ((i "alt") /. 1000.) (f "speed") (f "climb") time

let listen_intruders = fun (geomap:G.widget) ->
  safe_bind "INTRUDER" (get_intruders geomap)

open Shapes

let get_shapes = fun (geomap:G.widget)_sender vs ->
  let f = fun s -> PprzLink.float_assoc s vs in
  let i = fun s -> PprzLink.int_assoc s vs in
  let st = fun s -> PprzLink.string_assoc s vs in
  let string_to_scaled_float = fun v -> (float (int_of_string v))/. 1e7 in
  let floatarr = fun s -> Array.map string_to_scaled_float (Array.of_list (Str.split list_separator (st s))) in
  let data =  {
    shid = i "id";
    shlinecolor = st "linecolor";
    shfillcolor = st "fillcolor";
    shopacity = i "opacity";
    shtype = int2shtype (i "shape");
    shstatus = int2shstatus (i "status");
    shlatarr = floatarr "latarr";
    shlonarr = floatarr "lonarr";
    shradius = f "radius";
    shtext = st "text"} in
  new_shmsg data geomap

let listen_shapes = fun (geomap:G.widget) ->
  safe_bind "SHAPE" (get_shapes geomap)

let listen_acs_and_msgs = fun geomap ac_notebook strips confirm_kill my_alert auto_center_new_ac auto_center_ac alt_graph timestamp ->
  (** Probe live A/Cs *)
  let probe = fun () ->
    ignore(message_request "gcs" "AIRCRAFTS" [] (fun _sender vs -> _req_aircrafts := false; aircrafts_msg confirm_kill my_alert geomap ac_notebook strips vs)) in
  let _ = GMain.Timeout.add 1000 (fun () -> probe (); !_req_aircrafts) in

  (** New aircraft message *)
  safe_bind "NEW_AIRCRAFT" (fun _sender vs -> one_new_ac confirm_kill my_alert geomap ac_notebook  strips (PprzLink.string_assoc "ac_id" vs));

  (** Listen for all messages on ivy *)
  listen_flight_params geomap auto_center_new_ac auto_center_ac my_alert alt_graph;
  listen_wind_msg geomap;
  listen_fbw_msg my_alert;
  listen_engine_status_msg ();
  listen_telemetry_status my_alert;
  listen_if_calib_msg ();
  listen_waypoint_moved ();
  listen_svsinfo my_alert;
  listen_alert my_alert;
  listen_error my_alert;
  listen_info_msg my_alert timestamp;
  listen_autopilot_version_msg my_alert timestamp;
  listen_tcas my_alert timestamp;
  listen_dcshot geomap timestamp;
  listen_intruders geomap;
  listen_shapes geomap;

  (** Select the active aircraft on notebook page selection *)
  let callback = fun i ->
    let ac_page = ac_notebook#get_nth_page i in
    Hashtbl.iter
      (fun ac_id ac ->
        if ac.pages#get_oid = ac_page#get_oid
        then select_ac ac_notebook ac_id)
      aircrafts in
  ignore (ac_notebook#connect#switch_page ~callback);

  (** Center the active aircraft *)
  let center_active = fun () ->
    if !active_ac <> "" then
      let ac = find_ac !active_ac in
      center geomap ac.track () in
  let key_press = fun ev ->
    match GdkEvent.Key.keyval ev with
      | k when (k = GdkKeysyms._c) || (k = GdkKeysyms._C) -> center_active () ; true
      | _ -> false in
  ignore (geomap#canvas#event#connect#after#key_press key_press);

  (* call periodic_handle_intruders every second *)
  ignore (Glib.Timeout.add 1000 (fun () -> Intruders.remove_old_intruders (); true));
