(*
 * $Id$
 *
 * Paparazzi center aircraft handling
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

module Utils = Pc_common
open Printf

let (//) = Filename.concat

let gcs = Env.paparazzi_src // "sw/ground_segment/cockpit/gcs"

let string_of_gdkcolor = fun c ->
  sprintf "#%2x%2x%2x" (Gdk.Color.red c) (Gdk.Color.green c) (Gdk.Color.blue c)

let aircraft_sample = fun name ac_id ->
  Xml.Element ("aircraft",
	       ["name", name;
		"ac_id", ac_id;
		"airframe", "airframes/microjet5.xml";
		"radio", "radios/cockpitSX.xml";
		"telemetry", "telemetry/default.xml";
		"flight_plan", "flight_plans/basic.xml";
		"settings", "settings/basic.xml";
		"gui_color", "blue"],
	       [])

				   
let write_conf_xml = fun ?(user_save = false) () ->
  let l = Hashtbl.fold (fun _ a r -> a::r) Utils.aircrafts [] in
  let l = List.sort (fun ac1 ac2 -> compare (Xml.attrib ac1 "name") (Xml.attrib ac2 "name")) l in
  let c = Xml.Element ("conf", [], l) in
  if c <> Xml.parse_file Utils.conf_xml_file then begin
    if not (Sys.file_exists Utils.backup_xml_file) then
      ignore (Sys.command (sprintf "cp %s %s" Utils.conf_xml_file Utils.backup_xml_file));
    let f = open_out Utils.conf_xml_file in
    fprintf f "%s\n" (ExtXml.to_string_fmt ~tab_attribs:true c);
    close_out f
  end;
  if user_save && Sys.file_exists Utils.backup_xml_file then begin
    let today = Unix.localtime (Unix.gettimeofday ()) in
    Sys.rename Utils.backup_xml_file (sprintf "%s.%04d-%02d-%02d_%02d:%02d" Utils.conf_xml_file (1900+today.Unix.tm_year) (today.Unix.tm_mon+1) today.Unix.tm_mday today.Unix.tm_hour today.Unix.tm_min)
  end

let new_ac_id = fun () ->
  let m = ref 0 in
  Hashtbl.iter
    (fun _  x ->
      m := max !m (int_of_string (ExtXml.attrib x "ac_id")))
    Utils.aircrafts ;
  !m + 1

let parse_conf_xml = fun vbox ->
  let strings = ref [] in
  Hashtbl.iter (fun name _ac -> strings := name :: !strings) Utils.aircrafts;
  Utils.combo ("" :: !strings) vbox


let combo_connect = fun ((combo: #GEdit.combo_box), (_,column)) cb ->
  ignore (combo#connect#changed
	    (fun () ->
	      match combo#active_iter with
	      | None -> ()
	      | Some row ->
		  let data = combo#model#get ~row ~column in
		  cb data))
	    
let editor =
  try Sys.getenv "EDITOR" with _ -> "gedit"

let edit = fun file ->
  ignore (Sys.command (sprintf "%s '%s'&" editor file))


let gcs_or_edit = fun file ->
  match GToolbox.question_box ~title:"Flight plan editing" ~default:2 ~buttons:["Text editor"; "GCS"] "Which editor do you want to use ?" with
    1 -> edit file
  | 2 -> ignore (Sys.command (sprintf "%s -edit '%s'&" gcs file))
  | _ -> failwith "Internal error: gcs_or_edit"
    
let ac_files = fun gui ->
  ["airframe", gui#label_airframe, gui#button_browse_airframe, gui#button_edit_airframe, edit, false;
   "flight_plan", gui#label_flight_plan, gui#button_browse_flight_plan, gui#button_edit_flight_plan, gcs_or_edit, false;
   "settings", gui#label_settings, gui#button_browse_settings, gui#button_edit_settings, edit, true;
   "radio", gui#label_radio, gui#button_browse_radio, gui#button_edit_radio, edit, false;
   "telemetry", gui#label_telemetry, gui#button_browse_telemetry, gui#button_edit_telemetry, edit, false]


(* Awful but easier *)
let current_color = ref "white"

let save_callback = fun ?user_save gui ac_combo () ->
  let ac_name = Utils.combo_value ac_combo in
  if ac_name <> "" then begin
    let color = !current_color in
    let aircraft =
      Xml.Element ("aircraft",
		   ["name", ac_name;
		    "ac_id", gui#entry_ac_id#text;
		    "airframe", gui#label_airframe#text;
		    "radio", gui#label_radio#text;
		    "telemetry", gui#label_telemetry#text;
		    "flight_plan", gui#label_flight_plan#text;
		    "settings", gui#label_settings#text;
		    "gui_color", color],
		   []) in
    if gui#entry_ac_id#text <> "" then begin
      begin try Hashtbl.remove Utils.aircrafts ac_name with _ -> () end;
      Hashtbl.add Utils.aircrafts ac_name aircraft
    end
  end;
  write_conf_xml ?user_save ()


let first_word = fun s ->
  try
    let n = String.index s ' ' in
    String.sub s 0 n
  with
    Not_found -> s



(* Link A/C to airframe & flight_plan labels *)
let ac_combo_handler = fun gui (ac_combo:Utils.combo) target_combo ->
  let update_params = fun ac_name ->
    try
      let aircraft = Hashtbl.find Utils.aircrafts ac_name in
      let sample = aircraft_sample ac_name "42" in
      let value = fun a ->
	try (ExtXml.attrib aircraft a) with _ -> Xml.attrib sample a in
      List.iter
	(fun (a, label, _, _, _, _) -> label#set_text (value a))
	(ac_files gui);
	let ac_id = ExtXml.attrib aircraft "ac_id"
	and gui_color = ExtXml.attrib_or_default aircraft "gui_color" "white" in
	gui#button_clean#misc#set_sensitive true;
	gui#button_build#misc#set_sensitive true;
	gui#eventbox_gui_color#misc#modify_bg [`NORMAL, `NAME gui_color];
	current_color := gui_color;
	gui#entry_ac_id#set_text ac_id;
	(Utils.combo_widget target_combo)#misc#set_sensitive true;
    with
      Not_found ->
	gui#label_airframe#set_text "";
	gui#label_flight_plan#set_text "";
	gui#button_clean#misc#set_sensitive false;
	gui#button_build#misc#set_sensitive false;
	(Utils.combo_widget target_combo)#misc#set_sensitive false
  in
  combo_connect ac_combo update_params;

  (* New A/C button *)
  let callback = fun _ ->
    match GToolbox.input_string ~title:"New A/C" ~text:"MYAC" "New A/C name ?" with
      None -> ()
    | Some s ->
	Utils.add_to_combo ac_combo s;
	let a = aircraft_sample s (string_of_int (new_ac_id ())) in
	Hashtbl.add Utils.aircrafts s a;
	update_params s
  in
  ignore (gui#menu_item_new_ac#connect#activate ~callback);

  (* Delete A/C *)
  let callback = fun _ ->
    let ac_name = Utils.combo_value ac_combo in
    if ac_name <> "" then
      match GToolbox.question_box ~title:"Delete A/C" ~buttons:["Cancel"; "Delete"] ~default:2 (sprintf "Delete %s ? (no undo after Save)" ac_name) with
	2 -> begin
	  begin try Hashtbl.remove Utils.aircrafts ac_name with _ -> () end;
	  let combo_box = Utils.combo_widget ac_combo in
	  match combo_box#active_iter with
	  | None -> ()
	  | Some row ->
	      let (store, _column) = Utils.combo_model ac_combo in
	      ignore (store#remove row);
	      combo_box#set_active 1
	end
      | _ -> ()
  in
  ignore (gui#delete_ac_menu_item#connect#activate ~callback);

  (* GUI color *)
  let callback = fun _ ->
    let csd = GWindow.color_selection_dialog ~show:true () in
    let callback = fun _ ->
      let colorname = string_of_gdkcolor csd#colorsel#color in
      gui#eventbox_gui_color#misc#modify_bg [`NORMAL, `NAME colorname];
      current_color := colorname;
      save_callback gui ac_combo ();
      csd#destroy () in
    ignore (csd#ok_button#connect#clicked ~callback);
    ignore (csd#cancel_button#connect#clicked ~callback:csd#destroy) in
  ignore(gui#button_gui_color#connect#clicked ~callback);

  (* A/C id *)
  ignore(gui#entry_ac_id#connect#changed ~callback:(fun () -> save_callback gui ac_combo ()));
  
  (* Conf *)
  List.iter (fun (name, label, button_browse, button_edit, editor, multiple) ->
    let callback = fun _ ->
      editor (Utils.conf_dir // label#text) in
    ignore (button_edit#connect#clicked ~callback);
    let callback = fun _ ->
      let subdir = Filename.dirname (first_word label#text) in
      let cb = fun selected ->
	let names = List.map (fun name -> subdir//name) selected in
	let names = String.concat " " names in
	label#set_text names;
	save_callback gui ac_combo ()
      in
      Utils.choose_xml_file ~multiple name subdir cb in
    ignore (button_browse#connect#clicked ~callback))
    (ac_files gui);


  (* Save button *)
  ignore(gui#menu_item_save_ac#connect#activate ~callback:(save_callback ~user_save:true gui ac_combo))


let build_handler = fun ~file gui ac_combo (target_combo:Utils.combo) (log:string->unit) ->
  (* Link target to upload button *)
  combo_connect target_combo
    (fun target ->
      gui#button_upload#misc#set_sensitive (target <> "sim"));

  (* New Target button *)
  let callback = fun _ ->
    match GToolbox.input_string ~title:"New Target" ~text:"tunnel" "New build target ?" with
      None -> ()
    | Some s ->
	let (store, column) = Utils.combo_model target_combo in
	let row = store#append () in
	store#set ~row ~column s;
	(Utils.combo_widget target_combo)#set_active_iter (Some row)
  in
  ignore (gui#button_new_target#connect#clicked ~callback);

  (* Clean button *)
  let callback = fun () ->
    Utils.command ~file gui log (Utils.combo_value ac_combo) "clean_ac" in
  ignore (gui#button_clean#connect#clicked ~callback);
  
  (* Build button *)
  let callback = fun () ->
    let ac_name = Utils.combo_value ac_combo
    and target = Utils.combo_value target_combo in
    let target = if target="sim" then target else sprintf "%s.compile" target in
    Utils.command ~file gui log ac_name target in
  ignore (gui#button_build#connect#clicked ~callback);
  
  (* Upload button *)
  let callback = fun () ->
    let ac_name = Utils.combo_value ac_combo
    and target = Utils.combo_value target_combo in
    Utils.command ~file gui log ac_name (sprintf "%s.upload" target) in
  ignore (gui#button_upload#connect#clicked ~callback)

