(*
 * Paparazzi center main module
 *
 * Copyright (C) 2007 ENAC, Pascal Brisset, Antoine Drouin
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
module Utils = Pc_common
module CP = Pc_control_panel
module AC = Pc_aircraft

let (//) = Filename.concat
let ios = int_of_string
let soi = string_of_int


(*********************** Preferences handling **************************)

let get_entry_value = fun xml name ->
  let e = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = name) xml "entry" in
  Xml.attrib e "value"

let read_preferences = fun (gui:Gtk_pc.window) file (ac_combo:Gtk_tools.combo) (session_combo:Gtk_tools.combo) (target_combo:Gtk_tools.combo) ->
  let xml = Xml.parse_file file in

  let read_one = fun name use ->
    try
      let ac_name = get_entry_value xml name in
      use ac_name
    with Not_found -> () in

  (*********** Last A/C *)
  read_one "last A/C" (Gtk_tools.select_in_combo ac_combo);

  (*********** Last session *)
  read_one "last session" (Gtk_tools.select_in_combo session_combo);

  (*********** Last target *)
  read_one "last target" (Gtk_tools.select_in_combo target_combo);

  (*********** Window Size *)
  read_one "width"
    (fun width ->
      read_one "height" (fun height -> gui#window#resize (ios width) (ios height)));

  (*********** Left pane size *)
  read_one "left_pane_width"
    (fun width -> gui#vbox_left_pane#misc#set_size_request ~width:(ios width) ())



let gconf_entry = fun name value ->
  Xml.Element ("entry", ["name", name;
			 "value", value;
			 "application", "paparazzi center"],
	       [])

let add_entry = fun xml name value ->
  let entry = gconf_entry name value in
  let select = fun x -> Xml.attrib x "name" = name in
  let xml = ExtXml.remove_child ~select "entry" xml in
  Xml.Element (Xml.tag xml, Xml.attribs xml, entry::Xml.children xml)


let write_preferences = fun (gui:Gtk_pc.window) file (ac_combo:Gtk_tools.combo) (session_combo:Gtk_tools.combo) (target_combo:Gtk_tools.combo) ->
  let xml = if Sys.file_exists file then Xml.parse_file file else Xml.Element ("gconf", [], []) in

  (* Save A/C name *)
  let xml =
    try
      let ac_name = Gtk_tools.combo_value ac_combo in
      add_entry xml "last A/C" ac_name
    with Not_found -> xml in

  (* Save session *)
  let xml =
    let session_name = Gtk_tools.combo_value session_combo in
    add_entry xml "last session" session_name in

  (* Save target *)
  let xml = (
    try
      let name = Gtk_tools.combo_value target_combo in
      add_entry xml "last target" name
    with _ -> xml) in

  let xml =
    try
      (* Save window size *)
      let width, height = Gdk.Drawable.get_size gui#window#misc#window in
      let xml = add_entry xml "width" (soi width) in
      let xml = add_entry xml "height" (soi height) in

      (* Save left pane width *)
      let width = gui#hpaned#position in
      let xml = add_entry xml "left_pane_width" (soi width) in
      xml
    with
      Gpointer.Null ->
	prerr_endline "Please properly quit to save layout preferences";
	xml in

  let f = open_out file in
  Printf.fprintf f "%s\n" (ExtXml.to_string_fmt xml);
  close_out f

let quit_callback = fun gui ac_combo session_combo target_combo _ ->
  CP.close_programs gui;
  write_preferences gui Env.gconf_file ac_combo session_combo target_combo;
  exit 0

let quit_button_callback = fun gui ac_combo session_combo target_combo () ->
  if Sys.file_exists Utils.backup_xml_file then begin
    let rec question_box = fun () ->
      match GToolbox.question_box ~title:"Quit" ~buttons:["Save changes"; "Discard changes"; "View changes"; "Cancel"] ~default:1 "Configuration changes have not been saved" with
      | 2 ->
	  Sys.rename Utils.backup_xml_file Utils.conf_xml_file;
	  quit_callback gui ac_combo session_combo target_combo ()
      | 3 ->
	  ignore (Sys.command (sprintf "meld %s %s" Utils.backup_xml_file Utils.conf_xml_file));
	  question_box ()
      | 1 ->
	  Sys.remove Utils.backup_xml_file;
	  quit_callback gui ac_combo session_combo target_combo ()
      | _ -> () in
    question_box ()
  end else
    match GToolbox.question_box ~title:"Quit" ~buttons:["Cancel"; "Quit"] ~default:2 "Quit ?" with
    2 -> quit_callback gui ac_combo session_combo target_combo ()
  | _ -> ()



(************************** Main *********************************************)
let () =
  let session = ref ""
  and fullscreen = ref false in
  Arg.parse
    ["-fullscreen", Arg.Set fullscreen, "Fullscreen window";
     "-session", Arg.Set_string session, "<session name> Run a custom session"]
    (fun x -> fprintf stderr "Warning: Don't do anything with '%s'\n%!" x)
    "Usage: ";
  let file = Env.paparazzi_src // "sw" // "supervision" // "paparazzicenter.glade" in
  let gui = new Gtk_pc.window ~file () in

  if !fullscreen then
    gui#window#fullscreen ();
  gui#toplevel#show ();

  let paparazzi_pixbuf = GdkPixbuf.from_file Env.icon_file in
  gui#window#set_icon (Some paparazzi_pixbuf);

  let s = gui#statusbar#new_context "env" in
  ignore (s#push (sprintf "HOME=%s SRC=%s" Env.paparazzi_home Env.paparazzi_src));

  if Sys.file_exists Utils.backup_xml_file then begin
    let rec question_box = fun () ->
      let message = "Configuration changes to conf/conf.xml were not saved during the last session.\nIf you made any manual changes to conf/conf.xml and choose [Discard changes] you will also lose these." in
      match GToolbox.question_box ~title:"Backup" ~buttons:["Keep changes"; "Discard changes"; "View changes"] ~default:2 message with
      | 2 -> Sys.rename Utils.backup_xml_file Utils.conf_xml_file
      | 3 -> ignore (Sys.command (sprintf "meld %s %s" Utils.backup_xml_file Utils.conf_xml_file)); question_box ()
      | _ -> Sys.remove Utils.backup_xml_file in
    question_box ()
  end;

  Utils.build_aircrafts ();

  let ac_combo = AC.parse_conf_xml gui#vbox_ac
  and target_combo = Gtk_tools.combo ["sim";"fbw";"ap"] gui#vbox_target in

  (Gtk_tools.combo_widget target_combo)#misc#set_sensitive false;
  gui#button_clean#misc#set_sensitive false;
  gui#button_build#misc#set_sensitive false;

  AC.ac_combo_handler gui ac_combo target_combo;

  (* Change the buffer of the text view to attach a tag_table *)
  let background_tags =
    List.map (fun color ->
      let tag = GText.tag ~name:color () in
      tag#set_property (`BACKGROUND color);
      (color, tag))
      ["red"; "green"; "orange"; "cyan"] in
  let tag_table = GText.tag_table () in
  List.iter (fun (_color, tag) -> tag_table#add tag#as_tag) background_tags;
  let buffer = GText.buffer ~tag_table () in
  gui#console#set_buffer buffer;

  let errors = "red", ["error"; "no such file"; "undefined reference"; "failure"; "multiple definition"]
  and warnings = "orange", ["warning"]
  and info = "green", ["pragma message"]
  and version = "cyan", ["paparazzi version"] in

  let color_regexps =
    List.map (fun (color, strings) ->
      let s = List.map (fun s -> "\\("^s^"\\)") strings in
      let s = String.concat "\\|" s in
      let s = ".*\\("^s^"\\)" in
      color, Str.regexp_case_fold s)
      [errors; warnings; info; version] in
  let compute_tags = fun s ->
    let rec loop = function
	(color, regexp)::rs ->
	  if Str.string_match regexp s 0 then
	    [List.assoc color background_tags]
	  else
	    loop rs
      | [] -> [] in
    loop color_regexps in

  let log = fun s ->
    let iter = gui#console#buffer#end_iter in
    let tags = compute_tags s in
    gui#console#buffer#insert ~iter ~tags s;
    (* Scroll to the bottom line *)
    let end_iter = gui#console#buffer#end_iter in
    let end_mark = gui#console#buffer#create_mark end_iter in
    gui#console#scroll_mark_onscreen (`MARK end_mark) in

  AC.build_handler ~file gui ac_combo target_combo log;

  let session_combo, execute_session = CP.supervision ~file gui log ac_combo target_combo in

  (* Quit button *)
  ignore (gui#menu_item_quit#connect#activate ~callback:(quit_button_callback gui ac_combo session_combo target_combo));

  ignore (gui#window#event#connect#delete ~callback:(quit_callback gui ac_combo session_combo target_combo));

  (* Fullscreen menu entry *)
  let callback = fun () ->
    fullscreen := not !fullscreen;
    if !fullscreen then
      gui#window#fullscreen ()
    else
      gui#window#unfullscreen () in
  ignore (gui#menu_item_fullscreen#connect#activate ~callback);

  (* Help/About menu entry *)
  let callback = fun () ->
    ignore (GToolbox.message_box ~title:"About Paparazzi Center" ~icon:(GMisc.image ~pixbuf:paparazzi_pixbuf ())#coerce "Copyright (C) 2007-2008 ENAC, Pascal Brisset\nhttp://paparazzi.enac.fr") in
  ignore (gui#menu_item_about#connect#activate ~callback);

  (* Help/Get Help menu entry *)
  let callback = fun () ->
    ignore (GToolbox.message_box ~title:"Getting Help with Paparazzi" ~icon:(GMisc.image ~pixbuf:paparazzi_pixbuf ())#coerce "The primary documentation for Paparazzi is on the wiki:\nhttp://paparazzi.enac.fr\n\nCommunity-based support is through the paparazzi-devel mailing list:\nhttp://paparazzi.enac.fr/wiki/Contact\n\nThe Paparazzi auto-generated developer documentation is found on GitHub:\nhttp://paparazzi.github.io/docs/\n\nThe Paparazzi sourcecode can be found on GitHub:\nhttps://github.com/paparazzi/paparazzi\n\nIf you think you have found a bug or would like to make a feature request, feel\nfree to visit the Issues page found on GitHub (link found on the above webpage).") in
  ignore (gui#menu_item_get_help#connect#activate ~callback);

  (* Read preferences *)
  if Sys.file_exists Env.gconf_file then begin
    read_preferences gui Env.gconf_file ac_combo session_combo target_combo
  end;

  (* Run the command line session *)
  if !session <> "" then begin
    Gtk_tools.select_in_combo session_combo !session;
    execute_session !session
  end;

  GMain.Main.main ();;
