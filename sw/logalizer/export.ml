(*
 * GUI to export some values of a log
 *
 * Copyright (C) 2008, ENAC
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
let class_name="telemetry"

let cols = new GTree.column_list
let col_message = cols#add Gobject.Data.string
let col_field = cols#add Gobject.Data.string
let col_to_export = cols#add Gobject.Data.boolean
let col_visible = cols#add Gobject.Data.boolean


(** Toggling a tree element *)
let item_toggled ~(model : GTree.tree_store) ~column path =
  let row = model#get_iter path in
  let b = model#get ~row ~column in
  model#set ~row ~column (not b)


(****************************************************************************)
let display_columns = fun treeview model ->
  let renderer = GTree.cell_renderer_text [`XALIGN 0.] in
  let vc = GTree.view_column ~title:"Message" ~renderer:(renderer, ["text", col_message]) () in
  ignore (treeview#append_column vc);
  let renderer = GTree.cell_renderer_text [`XALIGN 0.] in
  let vc = GTree.view_column ~title:"Field" ~renderer:(renderer, ["text", col_field]) () in
  ignore (treeview#append_column vc);

  let renderer = GTree.cell_renderer_toggle [`XALIGN 0.] in
  let vc = GTree.view_column ~title:"To export" ~renderer:(renderer, ["active", col_to_export; "visible", col_visible]) () in
  vc#set_clickable true;
  ignore (renderer#connect#toggled ~callback:(item_toggled ~model ~column:col_to_export));
  ignore (treeview#append_column vc)


(****************************************************************************)
(** Fill the tree model from the message_xml data structure *)
let fill_data = fun (treeview:GTree.view) (model:GTree.tree_store) messages_xml prefs ->
  List.iter (fun msg ->
    let row = model#append ()
    and msg_name = ExtXml.attrib msg "name" in
    model#set ~row ~column:col_message msg_name;
    List.iter (fun field ->
      let parent = row in
      let row = model#append ~parent ()
      and field_name = ExtXml.attrib field "name" in
      model#set ~row ~column:col_visible true;
      let to_export = List.mem (msg_name,field_name) prefs in
      if to_export then
	treeview#expand_row (model#get_path parent);
      model#set ~row ~column:col_to_export to_export;
      model#set ~row ~column:col_field field_name)
      (List.filter (fun x -> Xml.tag x = "field") (Xml.children msg)))
    (Xml.children messages_xml)

type timestamp =
    Msg of string
  | Period of float (* in s *)


(****************************************************************************)
(** Returns the last available WGS84 position or raise a Failure exception *)
let get_last_geo_pos = fun lookup ->
  if lookup "GPS" "mode" <> "" then
    let utm_east = float_of_string (lookup "GPS" "utm_east") /. 100.
    and utm_north = float_of_string (lookup "GPS" "utm_north") /. 100.
    and utm_zone = int_of_string (lookup "GPS" "utm_zone") in
    Latlong.of_utm WGS84 {utm_x=utm_east; utm_y=utm_north; utm_zone=utm_zone}
  else if lookup "INS_REF" "ecef_x0" <>"" && lookup "ROTORCRAFT_FP" "east" <>"" then
    let getf = fun m f -> float_of_string (lookup m f) in
    let x0 = getf "INS_REF" "ecef_x0" /. 100.
    and y0 = getf "INS_REF" "ecef_y0" /. 100.
    and z0 = getf "INS_REF" "ecef_z0" /. 100.
    and e = getf "ROTORCRAFT_FP" "east" /. 256.
    and n = getf "ROTORCRAFT_FP" "north" /. 256.
    and u = getf "ROTORCRAFT_FP" "up" /. 256. in
    let ecef0 = make_ecef [|x0; y0; z0 |]
    and ned = make_ned [|n; e; -.u|] in
    fst (geo_of_ecef WGS84 (ecef_of_ned ecef0 ned))
  else
    failwith "No geo pos found"

(***************************************************)
let date_of_log_filename = fun filename ->
  let filename = Filename.basename filename in
  try
    Scanf.sscanf filename "%2d_%2d_%2d__%2d_%2d_%2d" (fun y m md h min s ->
      let tm = { Unix.tm_sec = s; tm_min= min; tm_hour = h;
		 tm_mday = md; tm_mon = m-1; tm_year = y+100;
		 tm_wday= -1; tm_yday= -1; tm_isdst=false } in
      fst (Unix.mktime tm))
  with
    exc ->
      fprintf stderr "Error parsing filename to get date (%s) : %s\nUsing current time\n%!" filename (Printexc.to_string exc);
      Unix.gettimeofday ()

let format_time = fun t ->
  let tm = Unix.gmtime t in
  let sec = float  tm.Unix.tm_sec +. fst (modf t) in
  Printf.sprintf "%02d:%02d:%2.3f" tm.Unix.tm_hour tm.Unix.tm_min sec


(*****************************************************************************)
let export_values = fun ?(sep="tab") ?(export_geo_pos=true) (model:GTree.tree_store) data timestamp filename ->
  let sep = if sep = "tab" then "\t" else sep in
  let fields_to_export = ref [] in
  model#foreach (fun _path row ->
    if model#get ~row ~column:col_to_export then begin
      let field = model#get ~row ~column:col_field
      and parent = match model#iter_parent row with Some p -> p | None -> failwith "export_value: no parent ???" in
      let msg = model#get ~row:parent ~column:col_message in
      fields_to_export := (msg, field) :: !fields_to_export
    end;
    false);

  (* Save preferences *)
  let value = String.concat ";" (List.map (fun (msg, field) -> sprintf "%s:%s" msg field) !fields_to_export) in
  let xml = if Sys.file_exists Env.gconf_file then ExtXml.parse_file Env.gconf_file else Xml.Element ("gconf", [], []) in
  let xml = ExtXml.Gconf.add_entry xml "log plotter" "to_export" value in
  let f = open_out Env.gconf_file in
  Printf.fprintf f "%s\n" (ExtXml.to_string_fmt xml);
  close_out f;

  let f = open_out filename in
  (* Print the header *)
  fprintf f "Time%sUTC" sep;
  if export_geo_pos then
    fprintf f "%sGPS_lat(deg)%sGPS_long(deg)" sep sep;
  List.iter (fun (m,field) -> fprintf f "%s%s:%s" sep m field) !fields_to_export;
  fprintf f "\n%!";

  let log_date = date_of_log_filename filename in

  (* Store for the current values *)
  let last_values = Hashtbl.create 97
  and time = ref (match data with (t, _, _)::_ -> t | _ -> 0.) in

  let print_last_values = fun t ->
    let all_values = ref true in

    let lookup = fun m field  ->
      try
	PprzLink.string_of_value (Hashtbl.find last_values (m,String.lowercase field))
      with
	Not_found -> "" in

    let buf = Buffer.create 64 in

    let local_time = log_date +. t in

    bprintf buf "%.3f%s%s" t sep (format_time local_time);

    if export_geo_pos then begin
      try
	let wgs84 = get_last_geo_pos lookup in
	bprintf buf "%s%.9f%s%.9f" sep ((Rad>>Deg) wgs84.posn_lat) sep ((Rad>>Deg) wgs84.posn_long)
      with
	exc ->
	  all_values := false;
	  bprintf buf "%sN/A%sN/A" sep sep
    end;

    (** Encapsulation of lookout function to mark the [all_values] flag *)
    let lookup = fun m f ->
      let s = lookup m f in
      if s = "" then all_values := false;
      s in

    List.iter
      (fun (m,field) ->
	let v = lookup m field  in
	bprintf buf "%s%s" sep v)
      !fields_to_export;

    if !all_values then
      fprintf f "%s\n" (Buffer.contents buf) in

  (* Write one line per time stamp. *)
  List.iter (fun (t, msg, fields) ->
    (* Output values on a time period basis *)
    begin
      match timestamp with
      | Period p -> (* We suppose that the period is higher than the time between
		       any two consecutives messages *)
	  if t >= !time then begin
	    print_last_values !time;
	    time := !time +. p
	  end
      | _ -> ()
    end;

    List.iter (fun (f, v) ->
      Hashtbl.replace last_values (msg, f) v)
      fields;

    (* Output values on a msg name basis *)
    match timestamp with
      Msg m when m =  msg ->
	print_last_values t
    | _ -> ())
    data;

  close_out f;;


(*****************************************************************************)
let read_preferences = fun () ->
  if Sys.file_exists Env.gconf_file then
    try
      let xml = ExtXml.parse_file Env.gconf_file in
      let to_export = ExtXml.Gconf.get_value xml "to_export" in
      let pairs = Str.split (Str.regexp ";") to_export in
      List.map
	(fun s ->
	  match Str.split (Str.regexp ":") s with
	    [m; f] -> (m, f)
	  | _ -> failwith (sprintf "Unexpected pref in '%s'" s))
	pairs
    with
      Not_found -> []
  else
    []


(** The save file dialog box *)
let save_values = fun w filename save ->
  match GToolbox.select_file ~title:"Save Values" ~filename () with
    None -> ()
  | Some file ->
      save file;
      w#export#destroy ()


(*****************************************************************************)
(** The popup window displaying values to export *)
let popup = fun ?(no_gui = false) xml log_filename data ->
  (* Build the list window *)
  let file = Env.paparazzi_src // "sw" // "logalizer" // "export.glade" in
  let w = new Gtk_export.export ~file () in
  let icon = GdkPixbuf.from_file Env.icon_file in
  w#export#set_icon (Some icon);

  (* Build the tree model *)
  let model = GTree.tree_store cols in

  (** Attach the model to the view *)
  w#treeview_messages#set_model (Some model#coerce);

  (** Render the columns *)
  display_columns w#treeview_messages model;

  (** Build list of message name *)
  let msg_names = Hashtbl.create 30 in
  List.iter (fun (_, name, _) -> if not (Hashtbl.mem msg_names name) then Hashtbl.add msg_names name ()) data;
  (** Fill the colums *)
  let xml_class = try ExtXml.child ~select:(fun x -> Xml.attrib x "name" = class_name) xml "msg_class"
    with Not_found -> ExtXml.child ~select:(fun x -> Xml.attrib x "name" = class_name) xml "class" in
  (** Filter xml message *)
  let xml_class = Xml.Element (
    class_name, [],
    List.filter (fun xml ->
      let name = Xml.attrib xml "name" in
      Hashtbl.mem msg_names name) (Xml.children xml_class)
      ) in
  let prefs = read_preferences () in
  fill_data w#treeview_messages model xml_class prefs;

  (* The combo box for the timestamp choice *)
  let strings = "Periodic" :: List.map (fun msg -> Xml.attrib msg "name") (Xml.children xml_class) in
  let (combo, (tree, column)) = GEdit.combo_box_text ~packing:w#box_choose_period#add ~strings () in
  tree#foreach (fun _path row -> combo#set_active_iter (Some row); true); (* Select the first *)

  let get_timestamp = fun () ->
    match combo#active_iter with
    | None -> failwith "get_timestamp"
    | Some row ->
	combo#model#get ~row ~column in

  (* Connect the timestamp chooser to the period entry *)
  ignore (combo#connect#changed
	    (fun () ->
	      let data = get_timestamp () in
	      w#entry_period#misc#set_sensitive (data = "Periodic")));



  (* The combo box for the separator *)
  let strings = ["tab"; ";"; ","] in
  let (combo, (tree, column)) = GEdit.combo_box_text ~packing:w#box_choose_sep#add ~strings () in
  tree#foreach (fun _path row -> combo#set_active_iter (Some row); true); (* Select the first *)
  let get_separator = fun () ->
    match combo#active_iter with
    | None -> failwith "get_timestamp"
    | Some row ->
	combo#model#get ~row ~column in

  ignore (w#button_cancel#connect#clicked (fun () -> w#export#destroy ()));

  (** Connect the Save button to the write action *)
  let callback = fun () ->
    let timestamp =
      let combo_value = get_timestamp () in
      if combo_value = "Periodic" then
	Period (float_of_string w#entry_period#text)
      else
	Msg combo_value in
    let sep = get_separator () in

    let do_export = fun x -> export_values ~sep ~export_geo_pos:w#checkbutton_LL#active model data timestamp x in

    let default_filename = Env.paparazzi_home // "var" // "logs" // log_filename ^ ".csv" in

    if no_gui then
      do_export default_filename
    else
      save_values w default_filename do_export in
  ignore (w#button_save#connect#clicked callback);

  (** Call automatic export or show the popup window *)
  if no_gui then begin
    callback ();
    w#export#destroy ()
  end else
    w#export#show ()

