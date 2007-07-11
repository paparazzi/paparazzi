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

open Pc_common
open Printf

let gcs = Env.paparazzi_src // "sw/ground_segment/cockpit/gcs"

let string_of_gdkcolor = fun c ->
  sprintf "#%2x%2x%2x" (Gdk.Color.red c) (Gdk.Color.green c) (Gdk.Color.blue c)

let aircraft_sample = fun name ac_id ->
  Xml.Element ("aircraft",
	       ["name", name;
		"ac_id", ac_id;
		"airframe", "airframes/microjet5.xml";
		"radio", "radios/cockpitMM.xml";
		"telemetry", "telemetry/default.xml";
		"flight_plan", "flight_plans/versatile.xml";
		"settings", "settings/basic.xml";
		"gui_color", "blue"],
	       [])

				   

let airframes_dir = conf_dir // "airframes"
let flight_plans_dir = conf_dir // "flight_plans"

let write_conf_xml = fun () ->
  Sys.rename conf_xml_file (conf_xml_file^"~");
  let l = Hashtbl.fold (fun _ a r -> a::r) aircrafts [] in
  let c = Xml.Element ("conf", [], l) in
  let f = open_out conf_xml_file in
  output_string f (ExtXml.to_string_fmt ~tab_attribs:true c);
  close_out f

let new_ac_id = fun () ->
  let m = ref 0 in
  Hashtbl.iter
    (fun _  x ->
      m := max !m (int_of_string (ExtXml.attrib x "ac_id")))
    aircrafts ;
  !m + 1

let parse_conf_xml = fun vbox ->
  let strings = ref [] in
  Hashtbl.iter (fun name _ac -> strings := name :: !strings) aircrafts;
  combo ~others:[""] !strings vbox


let combo_connect = fun ((combo: #GEdit.combo_box), (_,column)) cb ->
  ignore (combo#connect#changed
	    (fun () ->
	      match combo#active_iter with
	      | None -> ()
	      | Some row ->
		  let data = combo#model#get ~row ~column in
		  cb data))
	    
let combo_value = fun ((combo: #GEdit.combo_box), (_,column)) ->
  match combo#active_iter with
  | None -> raise Not_found
  | Some row -> combo#model#get ~row ~column
      
let is_xml_file = fun s ->
  let n = String.length s in
  n >= 4 && String.sub s (n-4) 4 = ".xml"

let combo_dir = fun ?others directory vbox ->
  let files = Array.to_list (Sys.readdir directory) in
  let xml_files = List.filter is_xml_file files in
  combo ?others xml_files vbox




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
  

(* Link A/C to airframe & flight_plan labels *)
let ac_combo_handler = fun gui (ac_combo:combo) target_combo ->
  combo_connect ac_combo
    (fun ac_name ->
      try
	let sample = aircraft_sample ac_name "42" in
	let aircraft = Hashtbl.find aircrafts ac_name in
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
	(combo_widget target_combo)#misc#set_sensitive true;
      with
	Not_found ->
	  gui#label_airframe#set_text "";
	  gui#label_flight_plan#set_text "";
	  gui#button_clean#misc#set_sensitive false;
	  gui#button_build#misc#set_sensitive false;
	  (combo_widget target_combo)#misc#set_sensitive false
    );

  (* New A/C button *)
  let callback = fun _ ->
    match GToolbox.input_string ~title:"New A/C" ~text:"MYAC" "New A/C name ?" with
      None -> ()
    | Some s ->
	let (store, column) = combo_model ac_combo in
	let row = store#append () in
	store#set ~row ~column s;
	let a = aircraft_sample s (string_of_int (new_ac_id ())) in
	Hashtbl.add aircrafts s a;
	aircrafts_table_has_changed := true;
	(combo_widget ac_combo)#set_active_iter (Some row)
  in
  ignore (gui#button_new_ac#connect#clicked ~callback);

  (* Delete A/C *)
  let callback = fun _ ->
    let ac_name = combo_value ac_combo in
    if ac_name <> "" then
      match GToolbox.question_box ~title:"Delete A/C" ~buttons:["Cancel"; "Delete"] ~default:2 (sprintf "Delete %s ? (NO undo)" ac_name) with
	2 -> begin
	  begin try Hashtbl.remove aircrafts ac_name with _ -> () end;
	  aircrafts_table_has_changed := true;
	  let combo_box = combo_widget ac_combo in
	  match combo_box#active_iter with
	  | None -> ()
	  | Some row ->
	      let (store, column) = combo_model ac_combo in
	      ignore (store#remove row);
	      combo_box#set_active 1
	end
      | _ -> ()
  in
  ignore (gui#button_delete_ac#connect#clicked ~callback);

  (* GUI color *)
  let callback = fun _ ->
    let csd = GWindow.color_selection_dialog ~show:true () in
    let callback = fun _ ->
      let colorname = string_of_gdkcolor csd#colorsel#color in
      gui#eventbox_gui_color#misc#modify_bg [`NORMAL, `NAME colorname];
      current_color := colorname;
      csd#destroy () in
    ignore (csd#ok_button#connect#clicked ~callback);
    ignore (csd#cancel_button#connect#clicked ~callback:csd#destroy) in
  ignore(gui#button_gui_color#connect#clicked ~callback);

  (* Save button *)
  let callback = fun _ ->
    match GToolbox.question_box ~title:"Save conf.xml" ~buttons:["Cancel"; "Save"] ~default:2 "Save in conf.xml ? (backup in conf.xml~)" with
      2 ->
	let ac_name = combo_value ac_combo in
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
	  begin try Hashtbl.remove aircrafts ac_name with _ -> () end;
	  Hashtbl.add aircrafts ac_name aircraft
	end;
	write_conf_xml ()
    | _ -> () in
  ignore(gui#button_save_ac#connect#clicked ~callback)


let build_handler = fun gui ac_combo (target_combo:combo) (log:string->unit) ->
  (* Link target to upload button *)
  combo_connect target_combo
    (fun target ->
      gui#button_upload#misc#set_sensitive (target <> "sim"));

  (* New Target button *)
  let callback = fun _ ->
    match GToolbox.input_string ~title:"New Target" ~text:"tunnel" "New build target ?" with
      None -> ()
    | Some s ->
	let (store, column) = combo_model target_combo in
	let row = store#append () in
	store#set ~row ~column s;
	(combo_widget target_combo)#set_active_iter (Some row)
  in
  ignore (gui#button_new_target#connect#clicked ~callback);


  (* Clean button *)
  let callback = fun () ->
    command log (combo_value ac_combo) "clean_ac" in
  ignore (gui#button_clean#connect#clicked ~callback);
  
  (* Build button *)
  let callback = fun () ->
    let ac_name = combo_value ac_combo
    and target = combo_value target_combo in
    let target = if target="sim" then target else sprintf "%s.compile" target in
    command log ac_name target in
  ignore (gui#button_build#connect#clicked ~callback);
  
  (* Upload button *)
  let callback = fun () ->
    let ac_name = combo_value ac_combo
    and target = combo_value target_combo in
    command log ac_name (sprintf "%s.upload" target) in
  ignore (gui#button_upload#connect#clicked ~callback)

let choose_xml_file = fun ?(multiple = false) title subdir cb ->
  let dir = conf_dir // subdir in
  let dialog = GWindow.file_chooser_dialog ~action:`OPEN ~title () in
  ignore (dialog#set_current_folder dir);
  dialog#add_filter (GFile.filter ~name:"xml" ~patterns:["*.xml"] ());
  dialog#add_button_stock `CANCEL `CANCEL ;
  dialog#add_select_button_stock `OPEN `OPEN ;
  dialog#set_select_multiple multiple;
  begin match dialog#run (), dialog#filename with
  | `OPEN, _ when multiple ->
      let names = dialog#get_filenames in
      dialog#destroy ();
      cb (List.map Filename.basename names)
  | `OPEN, Some name ->
      dialog#destroy ();
      cb [Filename.basename name]
  | _ -> dialog#destroy ()
  end

let first_word = fun s ->
  try
    let n = String.index s ' ' in
    String.sub s 0 n
  with
    Not_found -> s

let conf_handler = fun gui ->
  List.iter (fun (name, label, button_browse, button_edit, editor, multiple) ->
    let callback = fun _ ->
      editor (conf_dir // label#text) in
    ignore (button_edit#connect#clicked ~callback);
    let callback = fun _ ->
      let subdir = Filename.dirname (first_word label#text) in
      let cb = fun selected ->
	let names = List.map (fun name -> subdir//name) selected in
	let names = String.concat " " names in
	label#set_text names in
      choose_xml_file ~multiple name subdir cb in
    ignore (button_browse#connect#clicked ~callback))
    (ac_files gui)
