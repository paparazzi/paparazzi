(*
 * $Id$
 *
 * Paparazzi center process handling
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
open Pc_common

let socket_GCS_id = ref (Int32.of_int 0)

let control_panel_xml_file = conf_dir // "control_panel.xml"
let control_panel_xml = Xml.parse_file control_panel_xml_file
let programs =
  let h = Hashtbl.create 7 in
  let s = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = "programs") control_panel_xml "section" in
  List.iter 
    (fun p -> Hashtbl.add h (ExtXml.attrib p "name") p)
    (Xml.children s);
  h
let program_command = fun x ->
  let xml = Hashtbl.find programs x in
  Env.paparazzi_src // ExtXml.attrib xml "command"

let sessions =
  let h = Hashtbl.create 7 in
  let s = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = "sessions") control_panel_xml "section" in
  List.iter 
    (fun p -> Hashtbl.add h (ExtXml.attrib p "name") p)
    (Xml.children s);
  h



let not_sessions_section = fun x -> ExtXml.attrib x "name" <> "sessions"

let write_control_panel_xml = fun () ->
  Sys.rename control_panel_xml_file (control_panel_xml_file^"~");
  let l = Hashtbl.fold (fun _ a r -> a::r) sessions [] in
  let s = Xml.Element ("section", ["name","sessions"], l) in
  let children = List.filter not_sessions_section (Xml.children control_panel_xml) @ [s] in
  let c = Xml.Element ("control_panel", Xml.attribs control_panel_xml, children) in
  let f = open_out control_panel_xml_file in
  output_string f (ExtXml.to_string_fmt ~tab_attribs:false c);
  close_out f

let run_and_monitor = fun ?file ?(plugged=false) gui log com_name args ->
  let com = program_command com_name in
  let c = sprintf "%s %s" com args in
  let p = new Gtk_process.hbox_program ?file () in
  (gui#vbox_programs:GPack.box)#pack p#toplevel#coerce;
  p#label_com_name#set_text com_name;
  p#entry_program#set_text c;
  let pid = ref (-1) in
  let run = fun callback ->
    let c = p#entry_program#text in
    let c = if plugged then sprintf "%s -wid 0x%lx" c !socket_GCS_id else c in
    if plugged then
      gui#notebook#goto_page 2; (* FIXME *)
    log (sprintf "Run '%s'" c);
    let (pi, out) = run_and_log log ("exec "^c) in
    pid := pi;
    ignore (Glib.Io.add_watch [`HUP] (fun _ -> callback true; false) out) in
  let rec callback = fun stop ->
    match p#button_stop#label, stop with
      "gtk-stop", _ ->
	ignore (Unix.kill !pid Sys.sigkill);
	p#button_stop#set_label "gtk-redo";
	p#button_remove#misc#set_sensitive true;
	if stop && p#checkbutton_autolaunch#active then
	  callback false
    | "gtk-redo", false ->
	p#button_stop#set_label "gtk-stop";
	run callback;
	p#button_remove#misc#set_sensitive false
    | _ -> ()
  in
  ignore (p#button_stop#connect#clicked ~callback:(fun () -> callback false));
  run callback;
  
  (* Stop the program if the box is removed *)
  let callback = fun w ->
    callback true in
  ignore(p#toplevel#connect#destroy ~callback);

  (* Remove button *)
  let callback = fun () ->
    gui#vbox_programs#remove p#toplevel#coerce in
  ignore (p#button_remove#connect#clicked ~callback)


let close_programs = fun gui ->
  List.iter (fun w -> 
    gui#vbox_programs#remove w;
    w#destroy ()) 
    gui#vbox_programs#children

let parse_process_args = fun (name, args) ->
  (* How to do it with a simple regexp split ??? *)
  (* Mark spaces into args *)
  let marked_space = Char.chr 0 in
  let in_quotes = ref false in
  for i = 0 to String.length args - 1 do
    match args.[i] with
      ' ' when !in_quotes -> args.[i] <- marked_space
    | '"' -> in_quotes := not !in_quotes
    | _ -> ()
  done;
  (* Split *)
  let args = Str.split (Str.regexp "[ ]+") args in
  (* Restore spaces and remove quotes *)
  let restore_spaces = fun s ->
    let n = String.length s in
    for i = 0 to n - 1 do
      if s.[i] = marked_space then s.[i] <- ' '
    done;
    if n >= 2 && s.[0] = '"' then
      String.sub s 1 (n-2)
    else
      s in
  let args = List.map restore_spaces args in
  (* Remove the first "arg" which is the command *)
  let args = List.tl args in
  (* Build the XML arg list *)
  let is_option = fun s -> String.length s > 0 && s.[0] = '-' in
  let rec xml_args = function
      [] -> []
    | option::value::l when not (is_option value) ->
	Xml.Element("arg", ["flag",option; "constant", value],[])::xml_args l
    | option::l ->
	Xml.Element("arg", ["flag",option],[])::xml_args l in
  Xml.Element("program", ["name", name], xml_args args)

let save_session = fun gui ->
  (* Ask for a session name *)
  let text = gui#entry_session_name#text in
  let text = if text = "" then "My session" else text in
  match GToolbox.input_string ~ok:"Save" ~text ~title:"Session name" "Save user session ?" with
    None -> false
  | Some name ->
      let current_processes =
	List.map (fun hbox ->
	  let hbox = new GPack.box (Gobject.unsafe_cast hbox#as_widget) in
	  match hbox#children with
	    label::entry::_ ->
	      let label = new GMisc.label (Gobject.unsafe_cast label#as_widget)
	      and entry = new GEdit.entry (Gobject.unsafe_cast entry#as_widget) in
	      (label#text, entry#text)
	  | _ -> failwith "Internal error: save session") 
	  gui#vbox_programs#children in
      let current_programs = List.map parse_process_args current_processes in
      let session = Xml.Element("session", ["name", name], current_programs) in
      begin try Hashtbl.remove sessions name with _ -> () end;
      Hashtbl.add sessions name session;
      write_control_panel_xml ();
      true

let double_quote = fun s ->
  if String.contains s ' ' then
    sprintf "\"%s\"" s
  else
    s


let supervision = fun ?file gui log ->
  let supervision_page = 1 in (* FIXME *)

  let run_gcs = fun () ->
    run_and_monitor ?file ~plugged:true gui log "gcs" ""
  and run_server = fun () ->
    run_and_monitor ?file gui log "server" ""
  and run_link = fun args ->
    run_and_monitor ?file gui log "link" args
  and run_sitl = fun ac_name ->
    let args = sprintf "%s -boot -norc" ac_name in
    run_and_monitor ?file gui log "sim" args
  in

  (* Replay menu *)
  let callback = fun () ->
    gui#entry_session_name#set_text "Replay";
    run_and_monitor ?file gui log "play" "";
    run_server ();
    run_gcs ()
  in
  ignore (gui#replay_menu_item#connect#activate ~callback);

  (* Close session *)
  let callback = fun () ->
    close_programs gui in
  ignore (gui#button_remove_all_processes#connect#clicked ~callback);

  (* Programs *)
  let entries = ref [] in
  Hashtbl.iter
    (fun name prog ->
      let cb = fun () ->
	run_and_monitor ?file gui log name "" in
      entries := `I (name, cb) :: !entries)
    programs;
  let compare = fun x y ->
    match x, y with
      `I (x, _), `I (y, _) -> compare x y
    | _ -> compare x y in
  let menu = GMenu.menu ()
  and sorted_entries = List.sort compare !entries in
  GToolbox.build_menu menu sorted_entries;
  gui#programs_menu_item#set_submenu menu;

  (* Simulations *)
  let insert_sims_in_menu = fun num_page ->
    if num_page = supervision_page && !aircrafts_table_has_changed then
      let entries = ref [] in
      Hashtbl.iter
	(fun ac_name ac -> 
	  let cb = fun () ->
	    gui#entry_session_name#set_text (sprintf "Sim %s" ac_name);
	    run_gcs ();
	  run_server ();
	    run_sitl ac_name
	  in
	  entries := `I (ac_name, cb) :: !entries)
	aircrafts;
      let menu = GMenu.menu ()
      and sorted_entries = List.sort compare !entries in
      GToolbox.build_menu menu sorted_entries;
      gui#sim_menu_item#set_submenu menu;
      aircrafts_table_has_changed := false in

  ignore (gui#notebook#connect#switch_page ~callback:insert_sims_in_menu);

  (* Sessions *)
  let insert_sessions_in_menu = fun () ->
    let entries = ref [] in
    let cb = fun name session () ->
      gui#entry_session_name#set_text name;
      List.iter
	(fun program ->
	  let name = ExtXml.attrib program "name" in
	  let p = ref "" in
	  List.iter
	    (fun arg ->
	      let constant = 
		try double_quote (Xml.attrib arg "constant") with _ -> "" in
	      p := sprintf "%s %s %s" !p (ExtXml.attrib arg "flag") constant)
	    (Xml.children program);
	  run_and_monitor ~plugged:(name="gcs") ?file gui log name !p)
	(Xml.children session)
    in
    Hashtbl.iter
      (fun name session -> 
	entries := `I (name, cb name session) :: !entries)
      sessions;
    let menu = GMenu.menu ()
    and sorted_entries = List.sort compare !entries in
    GToolbox.build_menu menu sorted_entries;
    gui#session_menu_item#set_submenu menu in

  insert_sessions_in_menu ();

  (* Add new session *)
  let callback = fun () ->
    if save_session gui then
      insert_sessions_in_menu () in
  ignore (gui#button_save_session#connect#clicked ~callback);

  (* Remove current session *)
  let callback = fun () ->
    let session_name = gui#entry_session_name#text in
    match GToolbox.question_box ~title:"Delete user session" ~buttons:["Cancel"; "Delete"] ~default:2 (sprintf "Delete '%s' user session ? (NO undo)" session_name) with
      2 ->
	if Hashtbl.mem sessions session_name then begin
	  Hashtbl.remove sessions session_name;
	  write_control_panel_xml ();
	  insert_sessions_in_menu ()
	end;
	close_programs gui;
	gui#entry_session_name#set_text ""
    | _ -> ()
  in
  ignore (gui#button_delete_session#connect#clicked ~callback);

  (* Flights *)
  let cb = fun name args () ->
    gui#entry_session_name#set_text (sprintf "Fly with %s" name);
    run_gcs ();
    run_server ();
    run_link args
  in
  let entries = 
    [`I ("XBee", cb "XBee" "-transport xbee -uplink");
     `I ("Aerocomm", cb "Aerocomm" "-s 57600 -aerocomm -uplink"); 
     `I ("Serial", cb "Serial" "-uplink")] in
  let menu = GMenu.menu ()
  and sorted_entries = List.sort compare entries in
  GToolbox.build_menu menu sorted_entries;
  gui#fly_menu_item#set_submenu menu

