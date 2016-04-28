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

let always_keep_changes = ref false

let get_entry_value = fun xml name ->
  let e = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = name) xml "entry" in
  Xml.attrib e "value"

let read_preferences = fun (gui:Gtk_pc.window) file (ac_combo:Gtk_tools.combo) (session_combo:Gtk_tools.combo) (target_combo:Gtk_tools.combo) ->
  let xml = ExtXml.parse_file file in

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
    (fun width -> gui#vbox_left_pane#misc#set_size_request ~width:(ios width) ());

  (*********** Left pane size *)
  read_one "always keep changes"
    (fun keep_changes ->
      match keep_changes with
      | "true" -> always_keep_changes := true
      | _ -> always_keep_changes := false)


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
  let xml = if Sys.file_exists file then ExtXml.parse_file file else Xml.Element ("gconf", [], []) in

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

  (* save always_keep_changes choice *)
  let xml =
    add_entry xml "always keep changes" (string_of_bool !always_keep_changes) in

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

let backup_file_differs = fun () ->
  if Sys.file_exists Utils.backup_xml_file then
    ExtXml.parse_file Utils.backup_xml_file <> ExtXml.parse_file Utils.conf_xml_file
  else
    false

let quit_callback = fun gui ac_combo session_combo target_combo _ ->
  CP.close_programs gui;
  write_preferences gui Env.gconf_file ac_combo session_combo target_combo;
  exit 0

let quit_button_callback = fun gui ac_combo session_combo target_combo ?(confirm_quit = true) () ->
  if (not !always_keep_changes) && backup_file_differs () then begin
    let dialog = GWindow.dialog ~title:"Quit" ~modal:true () in
    dialog#add_button "Keep changes" `APPLY;
    dialog#add_button "Revert" `REJECT;
    dialog#add_button "View changes" `HELP;
    dialog#add_button "Cancel" `CANCEL;
    let _ = GMisc.label ~text:"Configuration has been changed since startup.\nIf you want to undo the changes choose [Revert]" ~packing:dialog#vbox#pack () in
    let checkbox = GButton.check_button ~label:"Always keep changes" ~active:!always_keep_changes ~packing:dialog#vbox#pack () in
    ignore (checkbox#connect#toggled (fun () ->
      always_keep_changes := checkbox#active;
      gui#menu_item_always_keep_changes#set_active checkbox#active;));

    match dialog#run () with
      `APPLY ->
        Sys.remove Utils.backup_xml_file;
	    quit_callback gui ac_combo session_combo target_combo ()
    | `REJECT ->
      ignore (Sys.command (sprintf "cp %s %s" Utils.backup_xml_file Utils.conf_xml_file));
	  Sys.remove Utils.backup_xml_file;
	  quit_callback gui ac_combo session_combo target_combo ()
    | `HELP ->
      ignore (Sys.command (sprintf "meld %s %s" Utils.backup_xml_file Utils.conf_xml_file))
    | `CANCEL ->
      dialog#destroy ()
    | _ -> ()
  end else begin
    if Sys.file_exists Utils.backup_xml_file then
      Sys.remove Utils.backup_xml_file;
    if confirm_quit then
      match GToolbox.question_box ~title:"Quit" ~buttons:["Cancel"; "Quit"] ~default:2 "Quit ?" with
        2 -> quit_callback gui ac_combo session_combo target_combo ()
      | _ -> ()
    else
      quit_callback gui ac_combo session_combo target_combo ()
  end

let quit_window_callback = fun gui ac_combo session_combo target_combo _ ->
  quit_button_callback gui ac_combo session_combo target_combo ~confirm_quit:false ();
  true

let keep_changes_callback = fun gui _ ->
  always_keep_changes := gui#menu_item_always_keep_changes#active;
  ()

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

    (* version string with whitespace/newline at the end stripped *)
  let version_str = Env.get_paparazzi_version () in
  let build_str =
    try
      let f = open_in (Env.paparazzi_home ^ "/var/build_version.txt") in
      let s = try input_line f with _ -> "UNKNOWN" in
      close_in f;
      s
    with _ -> "UNKNOWN" in

  let s = gui#statusbar#new_context "env" in
  ignore (s#push (sprintf "HOME=%s SRC=%s \tVersion=%s \tBuild=%s" Env.paparazzi_home Env.paparazzi_src version_str build_str));

  if Sys.file_exists Utils.backup_xml_file then begin
    let rec question_box = fun () ->
      let message = "Seems that Paparazzi Center wasn't quit cleanly.\nFound an older copy of conf.xml, do you want to restore it?" in
      match GToolbox.question_box ~title:"Backup" ~buttons:["Keep current"; "Restore"; "View changes"] ~default:1 message with
      | 2 -> Sys.rename Utils.backup_xml_file Utils.conf_xml_file
      | 3 -> ignore (Sys.command (sprintf "meld %s %s" Utils.backup_xml_file Utils.conf_xml_file)); question_box ()
      | _ -> Sys.remove Utils.backup_xml_file in
    question_box ()
  end;

  Utils.build_aircrafts ();

  let ac_combo = AC.parse_conf_xml gui#vbox_ac
  and target_combo = Gtk_tools.combo ~width:50 ["sim";"fbw";"ap"] gui#vbox_target
  and flash_combo = Gtk_tools.combo ~width:50 ["Default"] gui#vbox_flash in

  (Gtk_tools.combo_widget target_combo)#misc#set_sensitive false;
  (Gtk_tools.combo_widget flash_combo)#misc#set_sensitive false;
  gui#button_clean#misc#set_sensitive false;
  gui#button_build#misc#set_sensitive false;

  (* Change the buffer of the text view to attach a tag_table *)
  let background_tags =
    List.map (fun color ->
      let tag = GText.tag ~name:color () in
      tag#set_property (`BACKGROUND color);
      (color, tag))
      (* since tcl8.6 "green" refers to "darkgreen" and the former "green" is now "lime", but that is not available in older versions, so hardcode the color to #00ff00*)
      ["red"; "#00ff00"; "orange"; "cyan"; "yellow"] in
  let tag_table = GText.tag_table () in
  List.iter (fun (_color, tag) -> tag_table#add tag#as_tag) background_tags;
  let buffer = GText.buffer ~tag_table () in
  gui#console#set_buffer buffer;

  let errors = "red", ["error:"; "error "; "no such file"; "undefined reference"; "failure"; "multiple definition"]
  and warnings = "orange", ["warning"]
  and minor_warnings = "yellow", ["no srtm data found"]
  and info = "#00ff00", ["pragma message"; "info:"]
  and version = "cyan", ["paparazzi version"; "build aircraft"] in

  let color_regexps =
    List.map (fun (color, strings) ->
      let s = List.map (fun s -> "\\("^s^"\\)") strings in
      let s = String.concat "\\|" s in
      let s = ".*\\("^s^"\\)" in
      color, Str.regexp_case_fold s)
      [errors; warnings; minor_warnings; info; version] in
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

  AC.ac_combo_handler gui ac_combo target_combo flash_combo log;

  AC.build_handler ~file gui ac_combo target_combo flash_combo log;

  let session_combo, execute_session = CP.supervision ~file gui log ac_combo target_combo in

  (* Autosave on quit check box *)
  ignore (gui#menu_item_always_keep_changes#connect#toggled ~callback:(keep_changes_callback gui));

  (* Quit button *)
  ignore (gui#menu_item_quit#connect#activate ~callback:(quit_button_callback gui ac_combo session_combo target_combo));

  ignore (gui#window#event#connect#delete ~callback:(quit_window_callback gui ac_combo session_combo target_combo));

  (* Fullscreen menu entry *)
  let callback = fun () ->
    fullscreen := not !fullscreen;
    if !fullscreen then
      gui#window#fullscreen ()
    else
      gui#window#unfullscreen () in
  ignore (gui#menu_item_fullscreen#connect#activate ~callback);

  (* Help/About menu entry *)
  let aboutDialog = GWindow.about_dialog
    ~name:"Paparazzi Center"
    ~logo:paparazzi_pixbuf
    ~authors:["Pascal Brisset"]
    ~copyright:"Copyright (C) 2007-2008 ENAC, Pascal Brisset"
    ~license:"GPLv2"
    ~website:"http://paparazziuav.org"
    ~website_label:"http://paparazziuav.org"
    (*~version:version_str*)
    ~position:`CENTER_ON_PARENT
    ~destroy_with_parent:true
    ~parent:gui#window
    ()
  in
  ignore (gui#menu_item_about#connect#activate ~callback:(fun () -> ignore (aboutDialog#run ()); aboutDialog#misc#hide ()));

  let pprzInfoDialog (title,msg) =
    (* somehow doen't show the pprz icon, but the default info icon instead *)
    let dlg = GWindow.message_dialog
      ~title:title
      ~message:msg
      ~icon:paparazzi_pixbuf
      ~use_markup:true
      ~modal:true
      ~message_type:`INFO
      ~position:`CENTER_ON_PARENT
      ~destroy_with_parent:true
      ~parent:gui#window
      ~buttons:GWindow.Buttons.close () in
    let res = dlg#run () = `CLOSE in
    dlg#destroy ();
    res
  in

  (* Help/Get Help menu entry *)
  let help_text = "Primary documentation: Paparazzi wiki:\n<a href='https://wiki.paparazziuav.org'>https://wiki.paparazziuav.org</a>\n\nCommunity-based support, mailing list: <a href='https://wiki.paparazziuav.org/wiki/Contact'>Contact</a>\n\nThe Paparazzi auto-generated developer documentation:\n<a href='http://docs.paparazziuav.org'>http://docs.paparazziuav.org</a>\n\nPaparazzi sourcecode and issue tracker:\n<a href='https://github.com/paparazzi/paparazzi'>https://github.com/paparazzi/paparazzi</a>" in
  ignore (gui#menu_item_get_help#connect#activate ~callback:(fun () -> ignore (pprzInfoDialog ("Getting Help with Paparazzi",help_text))));

  (* Version *)
  let version_msg = ("Run version:\t" ^ version_str ^ "\nBuild version:\t" ^ build_str) in
  ignore (gui#menu_item_version#connect#activate ~callback:(fun () -> ignore (pprzInfoDialog ("Version",version_msg))));

  (* Read preferences *)
  if Sys.file_exists Env.gconf_file then begin
    read_preferences gui Env.gconf_file ac_combo session_combo target_combo
  end;

  gui#menu_item_always_keep_changes#set_active !always_keep_changes;

  (* Run the command line session *)
  if !session <> "" then begin
    Gtk_tools.select_in_combo session_combo !session;
    execute_session !session
  end;

  GMain.Main.main ();;
