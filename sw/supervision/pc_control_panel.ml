(*
 * Paparazzi center processes handling
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

let (//) = Filename.concat

let control_panel_xml_file = Utils.conf_dir // "control_panel.xml"
let control_panel_xml = ExtXml.parse_file control_panel_xml_file
let programs =
  let h = Hashtbl.create 7 in
  let s = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = "programs") control_panel_xml "section" in
  List.iter
    (fun p -> Hashtbl.add h (ExtXml.attrib p "name") p)
    (Xml.children s);
  h
let program_command = fun x ->
  try
    let xml = Hashtbl.find programs x in
    let cmd = ExtXml.attrib xml "command" in
    if cmd.[0] = '/' then
      cmd
    else
      Env.paparazzi_src // cmd
  with Not_found ->
    failwith (sprintf "Fatal Error: Program '%s' not found in control_panel.xml" x)

let sessions =
  let h = Hashtbl.create 7 in
  let s = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = "sessions") control_panel_xml "section" in
  List.iter
    (fun p -> Hashtbl.add h (ExtXml.attrib p "name") p)
    (Xml.children s);
  h

let flash_modes_xml_file = Utils.conf_dir // "flash_modes.xml"
let flash_mode_xml = ExtXml.parse_file flash_modes_xml_file
let flash_modes =
  let modes = Hashtbl.create 7 in (* table mode -> options *)
  let boards = Hashtbl.create 7 in (* table board -> modes *)
  let fm_common = Xml.children flash_mode_xml in (* common modes in dedicated file *)
  let fm_custom = try
    Xml.children (ExtXml.child ~select:(fun x -> Xml.attrib x "name" = "flash_modes") control_panel_xml "section") with
    _ -> [] in (* custom mode can be added to personal control_panel.xml file *)
  List.iter (fun m ->
    let mode = Xml.attrib m "name" in
    (* list of boards *)
    let board_list = try Xml.children (ExtXml.child m "boards") with _ -> [] in
    let board_list = List.map (fun x -> Xml.attrib x "name") board_list in
    (* build options for this mode *)
    let options = List.map (fun o ->
      sprintf "%s=%s" (Xml.attrib o "name") (Xml.attrib o "value")
      ) (List.filter (fun t -> Xml.tag t = "variable") (Xml.children m)) in
    let options = String.concat " " options in
    (* add to hash tables *)
    Hashtbl.add modes mode options;
    List.iter (fun b ->
      (* look if board is already in the table *)
      let _modes = try Hashtbl.find boards b with _ -> [] in
      (* add the new mode with together with the old ones *)
      Hashtbl.replace boards b ([mode] @ _modes)
    ) board_list;
  ) (fm_common @ fm_custom);
  (* convert string to regexp *)
  modes, boards


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


let run_and_monitor = fun ?file gui log com_name args ->
  Utils.run_and_monitor ?file gui log com_name (program_command com_name) args

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

let save_session = fun gui session_combo ->
  (* Ask for a session name *)
  let text = Gtk_tools.combo_value session_combo in
  let text = if text = "" then "My session" else text in
  match GToolbox.input_string ~ok:"Save" ~text ~title:"Session name" "Save custom session ?" with
    None -> ""
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
      name

let double_quote = fun s ->
  if String.contains s ' ' then
    sprintf "\"%s\"" s
  else
    s

let get_simtype = fun (target_combo : Gtk_tools.combo) ->
  (* get the list of possible targets *)
  let targets = Gtk_tools.combo_values_list target_combo in
  (* filter non simulator targets *)
  let sim_targets = ["sim"; "nps"] in
  let targets = List.filter (fun t -> List.mem t sim_targets) targets in
  (* open question box and return corresponding simulator type *)
  match targets with
    [] -> "none"
  | [t] -> t
  | l ->
      match GToolbox.question_box ~title:"Simulator type" ~buttons:l "Choose the simulator type:" with
      | 0 -> "none"
      | choice -> List.nth targets (choice-1)

let supervision = fun ?file gui log (ac_combo : Gtk_tools.combo) (target_combo : Gtk_tools.combo) ->
  let run_gcs = fun () ->
    run_and_monitor ?file gui log "GCS" ""
  and run_server = fun args ->
    run_and_monitor ?file gui log "Server" args
  and choose_and_run_sitl = fun ac_name ->
    let get_args = fun simtype ac_name ->
      match simtype with
          "sim" -> sprintf "-a %s -t %s --boot --norc" ac_name simtype
        | "jsbsim" -> sprintf "-a %s -t %s" ac_name simtype
        | "nps" -> sprintf "-a %s -t %s" ac_name simtype
        | _ -> "none"
    in
    let sim_type = get_simtype target_combo in
    let args = get_args sim_type ac_name in
    if args <> "none" then begin
      run_and_monitor ?file gui log "Simulator" args;
      run_and_monitor ?file gui log "GCS" "";
      run_and_monitor ?file gui log "Server" "-n"
    end
  in

  (* Sessions *)
  let session_combo = Gtk_tools.combo [] gui#vbox_session in

  let remove_custom_sessions = fun () ->
    let (store, _column) = Gtk_tools.combo_model session_combo in
    store#clear ()
  in

  let register_custom_sessions = fun () ->
    remove_custom_sessions ();
    Gtk_tools.add_to_combo session_combo "Simulation";
    Gtk_tools.add_to_combo session_combo "Replay";
    Gtk_tools.add_to_combo session_combo Gtk_tools.combo_separator;
    let strings = ref [] in
    Hashtbl.iter (fun name _session -> strings := name :: !strings) sessions;
    let ordered = List.sort String.compare !strings in
    List.iter (fun name -> Gtk_tools.add_to_combo session_combo name) ordered
  in

  register_custom_sessions ();
  Gtk_tools.select_in_combo session_combo "Simulation";

  let execute_custom = fun session_name ->
    let session = try Hashtbl.find sessions session_name with Not_found -> failwith (sprintf "Unknown session: %s" session_name) in
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
	run_and_monitor ?file gui log name !p)
      (Xml.children session)
  in

  (* Replay session *)
  let replay = fun () ->
    run_and_monitor ?file gui log "Log File Player" "";
    run_server "-n";
    run_gcs () in

  (* Simulations *)
  let simulation = fun () ->
    choose_and_run_sitl (Gtk_tools.combo_value ac_combo) in

  (* Run session *)
  let callback = fun () ->
    match Gtk_tools.combo_value session_combo with
      "Simulation" -> simulation ()
    | "Replay" -> replay ()
    | custom -> execute_custom custom in
  ignore (gui#button_execute#connect#clicked ~callback);

  (* Close session *)
  let callback = fun () ->
    close_programs gui in
  ignore (gui#button_remove_all_processes#connect#clicked ~callback);

  (* Tools *)
  let entries = ref [] in
  Hashtbl.iter
    (fun name _prog ->
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

  (* New session *)
  let callback = fun () ->
    match GToolbox.input_string ~title:"New session" ~text:"My session" "New session name ?" with
      None -> ()
    | Some s ->
	Gtk_tools.add_to_combo session_combo s in
  ignore (gui#menu_item_new_session#connect#activate ~callback);

  (* Save new session *)
  let callback = fun () ->
    match save_session gui session_combo with
      "" -> ()
    | session_name ->
	register_custom_sessions ();
	Gtk_tools.select_in_combo session_combo session_name
  in
  ignore (gui#menu_item_save_session#connect#activate ~callback);

  (* Remove current session *)
  let callback = fun () ->
    let session_name = Gtk_tools.combo_value session_combo in
    match GToolbox.question_box ~title:"Delete custom session" ~buttons:["Cancel"; "Delete"] ~default:2 (sprintf "Delete '%s' custom session ? (NO undo)" session_name) with
      2 ->
	if Hashtbl.mem sessions session_name then begin
	  Hashtbl.remove sessions session_name;
	  write_control_panel_xml ();
	  register_custom_sessions ()
	end;
	close_programs gui
    | _ -> ()
  in
  ignore (gui#menu_item_delete_session#connect#activate ~callback);
  session_combo, execute_custom

