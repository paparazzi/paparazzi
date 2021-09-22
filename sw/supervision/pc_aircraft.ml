(*
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
module CP = Pc_control_panel
open Printf

let (//) = Filename.concat

let gcs = Env.paparazzi_src // "sw/ground_segment/cockpit/gcs"

let regexp_space = Str.regexp "[ ]+"

let string_of_gdkcolor = fun c ->
  sprintf "#%04x%04x%04x" (Gdk.Color.red c) (Gdk.Color.green c) (Gdk.Color.blue c)

let aircraft_sample = fun name ac_id ->
  Xml.Element ("aircraft",
    [ "name", name;
      "ac_id", ac_id;
      "airframe", "airframes/examples/microjet.xml";
      "radio", "radios/cockpitSX.xml";
      "telemetry", "telemetry/default_fixedwing.xml";
      "flight_plan", "flight_plans/basic.xml";
      "settings", "settings/fixedwing_basic.xml";
      "settings_modules", "";
      "gui_color", "blue";
      "release", "" ],
      [])


let write_conf_xml = fun ?(user_save = false) () ->
  let l = Hashtbl.fold (fun _ a r -> a::r) Utils.aircrafts [] in
  let l = List.sort (fun ac1 ac2 -> compare (Xml.attrib ac1 "name") (Xml.attrib ac2 "name")) l in
  let c = Xml.Element ("conf", [], l) in
  if c <> ExtXml.parse_file Utils.conf_xml_file then begin
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
  let used = Array.make 256 false in
  Hashtbl.iter
    (fun _  x ->
      used.(int_of_string (ExtXml.attrib x "ac_id")) <- true)
    Utils.aircrafts ;
  let rec first_unused = fun i ->
    if i < 256 then
      if not used.(i) then i else first_unused (i+1)
    else
      failwith "Already 256 A/C in your conf.xml file !" in
  first_unused 1

let parse_conf_xml = fun vbox ->
  let strings = ref [] in
  Hashtbl.iter (fun name _ac -> strings := name :: !strings) Utils.aircrafts;
  let compare_ignore_case = fun s1 s2 ->
    String.compare (String.lowercase_ascii s1) (String.lowercase_ascii s2) in
  let ordered = List.sort compare_ignore_case ("" :: !strings) in
  Gtk_tools.combo ordered vbox

let editor =
  try Sys.getenv "EDITOR" with _ -> (
    if Os_calls.contains (Os_calls.os_name) "Darwin" then
      "open"
    else
      "gedit"
    )

let edit = fun file ->
  ignore (Sys.command (sprintf "%s %s&" editor file))


let gcs_or_edit = fun file ->
  match GToolbox.question_box ~title:"Flight plan editing" ~default:2 ~buttons:["Text editor"; "GCS"] "Which editor do you want to use ?" with
    1 -> edit file
  | 2 -> ignore (Sys.command (sprintf "%s -edit '%s'&" gcs file))
  | _ -> failwith "Internal error: gcs_or_edit"

let gitk_version = fun sha ->
  ignore (Sys.command (sprintf "gitk '%s'&" sha))


let execute_cmd_and_return_text = fun cmd ->
  let tmp_file = Filename.temp_file "" ".txt" in
  let _ = Sys.command @@ cmd ^ " > " ^ tmp_file in
  let chan = open_in tmp_file in
  let s = input_line chan in
  close_in chan;
  s

let tag_this_version = fun _ ->
  (execute_cmd_and_return_text "git rev-parse HEAD")

let get_commits_after_version = fun sha ->
  (execute_cmd_and_return_text (sprintf "git rev-list %s..HEAD --count" sha))

let get_commits_outside_version = fun sha ->
  (execute_cmd_and_return_text (sprintf "git rev-list HEAD..%s --count" sha))

let show_gitk_of_version = fun sha ->
  GToolbox.message_box ~title:"Compare" ("There have been " ^ (get_commits_after_version sha ) ^ " commits since the last reported test.\n The last reported test used " ^ (get_commits_outside_version sha ) ^ " commits that are not in this branch.");
  (execute_cmd_and_return_text (sprintf "gitk %s..HEAD & gitk HEAD..%s &" sha sha))

type ac_data =
    Label of GMisc.label
  | Tree of Gtk_tools.tree

let string_of_ac_data = fun d ->
  match d with
    Label l -> l#text
  | Tree  t -> Gtk_tools.tree_values t


(* Awful but easier *)
let current_color = ref "white"

let correct_ac_id = fun s ->
  try
    let n = int_of_string s in
    0 < n && n < 256
  with
    _ -> false

let correct_ac_name = fun s ->
  let allowed_char = function
      'a'..'z' | 'A'..'Z' | '0'..'9' | '_' -> ()
    | _ -> raise Exit in
  try
    String.iter allowed_char s;
    s <> ""
  with
    Exit -> false

(*TODO function text of date_type*)
let save_callback = fun ?user_save gui ac_combo tree tree_modules () ->
  let ac_name = Gtk_tools.combo_value ac_combo
  and ac_id = gui#entry_ac_id#text in

  if ac_name <> "" && ac_id <> "" then begin
    if not (correct_ac_id ac_id) then
      GToolbox.message_box ~title:"Error on A/C id" "A/C id must be a non null number less than 255"
    else
      let color = !current_color in
      let attribs = ["name", ac_name;
          "ac_id", ac_id;
          "airframe", gui#label_airframe#text;
          "radio", gui#label_radio#text;
          "telemetry", gui#label_telemetry#text;
          "flight_plan", gui#label_flight_plan#text;
          "settings", Gtk_tools.tree_values ~only_checked:false tree;
          "settings_modules", Gtk_tools.tree_values ~only_checked:false tree_modules;
          "gui_color", color ] in
      let attribs = if gui#label_release#text = "" then attribs else attribs @ ["release", gui#label_release#text ] in
      let aircraft = Xml.Element ("aircraft", attribs, []) in
      begin try Hashtbl.remove Utils.aircrafts ac_name with _ -> () end;
      Hashtbl.add Utils.aircrafts ac_name aircraft
  end;
  write_conf_xml ?user_save ()

(* selected state type *)
type selected_t = Selected | Unselected | Unknown

(* Get the settings (string list) with current modules *)
let get_settings_modules = fun ac_id aircraft_xml settings_modules ->
  (* get modules *)
  let ac = Aircraft.parse_aircraft ~parse_af:true ~parse_ap:true ~parse_fp:true "" aircraft_xml in
  let modules = List.map (fun m -> (m.Module.xml, m.Module.xml_filename)) ac.Aircraft.all_modules in
  (* get list of settings files *)
  let settings = List.fold_left (fun l (m, f) ->
    (* get list of settings_file xml node if any *)
    let settings_file_list = List.filter (fun t -> Xml.tag t = "settings_file") (Xml.children m) in
    let file_list = List.map (fun s -> "settings/"^(Xml.attrib s "name")) settings_file_list in
    (* include module file in the list only if it has a 'settings' node *)
    let settings_list = List.filter (fun t -> Xml.tag t = "settings") (Xml.children m) in
    (*let module_file = if List.length settings_list > 0 then [Env.filter_absolute_path f] else [] in*)
    (* include module file with specific name if they exist *)
    let settings_list = List.fold_left (fun l s ->
      try
        let name = Xml.attrib s "name" in
        (* test if there is no white space in settings name *)
        if Str.string_match (Str.regexp ".* .*") name 0
        then failwith "Paparazzicenter: no white space allowed in modules settings name";
        l @ [(Env.filter_absolute_path f)^"~"^name^"~"]
      with
      | Failure x -> prerr_endline x; l @ [Env.filter_absolute_path f]
      | _ -> l @ [Env.filter_absolute_path f]
    ) [] settings_list in
    l @ file_list (*@ module_file*) @ settings_list
  ) [] modules in
  (* store current state in a hashtable *)
  let current = Hashtbl.create 7 in
  let set = Str.split regexp_space settings_modules in
  List.iter (fun s ->
    let l = String.length s in
    if s.[0] == '[' && s.[l - 1] = ']'
    then Hashtbl.add current (String.sub s 1 (l - 2)) Unselected
    else Hashtbl.add current s Selected
  ) set;
  (* build list with previous state if necessary *)
  List.map (fun s ->
    (* get previous state, unknonw otherwise (new module, will be selected by default) *)
    let checked = try Hashtbl.find current s with _ -> Unknown in
    (* add to tree with correct state *)
    match checked with
    | Selected | Unknown -> s
    | Unselected -> ("["^s^"]")
  ) settings

let first_word = fun s ->
  try
    let n = String.index s ' ' in
    String.sub s 0 n
  with
    Not_found -> s

(** Test if an element is available for the current target *)

(** Get list of targets of an airframe *)
let get_targets_list = fun ac_xml ->
  let firmwares = List.filter (fun x -> ExtXml.tag_is x "firmware") (Xml.children ac_xml) in
  let targets = List.map (fun f -> List.filter (fun x -> ExtXml.tag_is x "target") (Xml.children f)) firmwares in
  List.flatten targets

(** Parse Airframe File for Targets **)
let parse_ac_targets = fun target_combo ac_file (log:string->unit) ->
  (* remember last target *)
  let last_target = try Gtk_tools.combo_value target_combo with _ -> "" in
  (* Clear ComboBox *)
  let (store, column) = Gtk_tools.combo_model target_combo in
  store#clear ();
  (* add targets *)
  try
    let af_xml = ExtXml.parse_file (Env.paparazzi_home // "conf" // ac_file) in
    let targets = get_targets_list af_xml in
    if List.length targets > 0 then
      List.iter (fun t -> Gtk_tools.add_to_combo target_combo (Xml.attrib t "name")) targets
    else begin
      Gtk_tools.add_to_combo target_combo "ap";
      Gtk_tools.add_to_combo target_combo "sim"
    end;
    Gtk_tools.select_in_combo target_combo last_target
  with _ ->
    log (sprintf "Error while parsing targets from file %s\n" ac_file);
    raise Not_found

(* Parse AC file for flash mode *)
let parse_ac_flash = fun target flash_combo ac_file ->
  (* remember last flash mode *)
  let last_flash_mode = Gtk_tools.combo_value flash_combo in
  (* Clear ComboBox *)
  let (store, column) = Gtk_tools.combo_model flash_combo in
  store#clear ();
  Gtk_tools.add_to_combo flash_combo "Default";
  try
    let af_xml = ExtXml.parse_file (Env.paparazzi_home // "conf" // ac_file) in
    let targets = get_targets_list af_xml in
    let board = Xml.attrib (List.find (fun t -> Xml.attrib t "name" = target) targets) "board" in
    (* board names as regexp *)
    let flash_modes = ref [] in
    Hashtbl.iter (fun b m ->
      if Str.string_match (Str.regexp b) board 0 then
        flash_modes := !flash_modes @ m;
      ) (snd CP.flash_modes);
    List.iter (fun m ->  Gtk_tools.add_to_combo flash_combo m) !flash_modes;
    (Gtk_tools.combo_widget flash_combo)#misc#set_sensitive (List.length !flash_modes > 0);
    Gtk_tools.select_in_combo flash_combo last_flash_mode
  with _ ->
    (* not a valid airframe file *)
    (Gtk_tools.combo_widget flash_combo)#misc#set_sensitive false;
    raise Not_found

(* Link A/C to airframe & flight_plan labels *)
let ac_combo_handler = fun gui (ac_combo:Gtk_tools.combo) target_combo flash_combo (log:string->unit) ->
  (* build tree for settings *)
  let tree_set = Gtk_tools.tree ~check_box:true gui#tree_settings in
  (* build tree for modules settings *)
  let tree_set_mod = Gtk_tools.tree ~check_box:true gui#tree_settings_modules in

  (* connect save_callback to the two toggle signals
   * it can't be done before because we need the two tree models
  *)
  let (_, _, _, tree_signal) = Gtk_tools.tree_model tree_set in
  ignore (tree_signal#toggled ~callback:(fun _ -> save_callback gui ac_combo tree_set tree_set_mod ()));
  let (_, _, _, tree_signal) = Gtk_tools.tree_model tree_set_mod in
  ignore (tree_signal#toggled ~callback:(fun _ -> save_callback gui ac_combo tree_set tree_set_mod ()));

  (* Link AC conf with labels and buttons *)
  let ac_files =
    [ "airframe", "airframes", Label gui#label_airframe, Some gui#button_browse_airframe, Some gui#button_edit_airframe, edit, None;
      "flight_plan", "flight_plans", Label gui#label_flight_plan, Some gui#button_browse_flight_plan, Some gui#button_edit_flight_plan, gcs_or_edit, None;
      "settings", "settings", Tree tree_set, Some gui#button_browse_settings, Some gui#button_edit_settings, edit, Some gui#button_remove_settings;
      "settings_modules", "settings", Tree tree_set_mod, None, None, (fun _ -> ()), None;
      "radio", "radios", Label gui#label_radio, Some gui#button_browse_radio, Some gui#button_edit_radio, edit, None;
      "telemetry", "telemetry", Label gui#label_telemetry, Some gui#button_browse_telemetry, Some gui#button_edit_telemetry, edit, None;
      "release", "release", Label gui#label_release, None, None, edit, None]
  in

  (* Update_params callback *)
  let update_params = fun ac_name ->
    try
      let aircraft = Hashtbl.find Utils.aircrafts ac_name in
      let sample = aircraft_sample ac_name "42" in
      (* update list of modules settings *)
      let ac_id = ExtXml.attrib aircraft "ac_id" in
      let settings_modules = try
        get_settings_modules ac_id aircraft (ExtXml.attrib_or_default aircraft "settings_modules" "")
      with
      | Failure x -> prerr_endline x; []
      | _ -> []
      in
      (* update aicraft hashtable *)
      let aircraft = ExtXml.subst_attrib "settings_modules" (String.concat " " settings_modules) aircraft in
      begin try Hashtbl.remove Utils.aircrafts ac_name with _ -> () end;
      Hashtbl.add Utils.aircrafts ac_name aircraft;
      let value = fun a -> try (ExtXml.attrib aircraft a) with _ -> Xml.attrib sample a in
      (* update elements *)
      List.iter (fun (a, _subdir, label, _, _, _, _) ->
        match label with
        | Label l -> l#set_text (value a)
        | Tree t ->
          ignore (Gtk_tools.clear_tree t);
          let names = Str.split regexp_space (value a) in
          List.iter (Gtk_tools.add_to_tree t) names;
      ) ac_files;
      let gui_color = ExtXml.attrib_or_default aircraft "gui_color" "white" in
      gui#button_clean#misc#set_sensitive true;
      gui#button_build#misc#set_sensitive true;
      gui#button_upload#misc#set_sensitive true;
      gui#eventbox_gui_color#misc#modify_bg [`NORMAL, `NAME gui_color];
      current_color := gui_color;
      gui#entry_ac_id#set_text ac_id;
      (Gtk_tools.combo_widget target_combo)#misc#set_sensitive true;
      (Gtk_tools.combo_widget flash_combo)#misc#set_sensitive true;
      let last_flash_mode = try Gtk_tools.combo_value flash_combo with _ -> "Default" in
      begin
        (* try parsing target from airframe file, may fail if not valid *)
        try parse_ac_targets target_combo (ExtXml.attrib aircraft "airframe") log with _ ->
          (Gtk_tools.combo_widget target_combo)#misc#set_sensitive false;
          gui#button_build#misc#set_sensitive false
      end;
      begin
        try parse_ac_flash (Gtk_tools.combo_value target_combo) flash_combo (ExtXml.attrib aircraft "airframe") with _ ->
          (Gtk_tools.combo_widget flash_combo)#misc#set_sensitive false;
          gui#button_upload#misc#set_sensitive false
      end;
      Gtk_tools.select_in_combo flash_combo last_flash_mode;
    with
      Not_found ->
        (* Not found in aircrafts hashtbl *)
        gui#button_build#misc#set_sensitive false;
        gui#button_clean#misc#set_sensitive false;
        (Gtk_tools.combo_widget target_combo)#misc#set_sensitive false;
        (Gtk_tools.combo_widget flash_combo)#misc#set_sensitive false;
        log (sprintf "Aircraft %s not in conf\n" ac_name) 
  in
  Gtk_tools.combo_connect ac_combo update_params;

  (* New A/C button *)
  let callback = fun _ ->
    match GToolbox.input_string ~title:"New A/C" ~text:"MYAC" "New A/C name ?" with
    | None -> ()
    | Some s ->
        if not (correct_ac_name s) then
          GToolbox.message_box ~title:"Error on A/C name" "A/C name must contain only letters, digits or underscores"
        else if (Hashtbl.mem Utils.aircrafts s) then
          GToolbox.message_box ~title:"Error on A/C name" "A/C name already exists in this conf"
        else begin
          let a = aircraft_sample s (string_of_int (new_ac_id ())) in
          (* add to hashtbl before combo to avoid update errors *)
          Hashtbl.add Utils.aircrafts s a;
          Gtk_tools.add_to_combo ac_combo s;
          update_params s
      end
  in
  ignore (gui#menu_item_new_ac#connect#activate ~callback);

  (* Copy A/C button *)
  let callback = fun _ ->
    let selected_ac_name = Gtk_tools.combo_value ac_combo in
    if selected_ac_name <> "" then
      match GToolbox.input_string ~title:"Copy A/C" ~text:"MYAC" "New A/C name ?" with
      | None -> ()
      | Some s ->
          if not (correct_ac_name s) then
            GToolbox.message_box ~title:"Error on A/C name" "A/C name must contain only letters, digits or underscores"
          else if (Hashtbl.mem Utils.aircrafts s) then
            GToolbox.message_box ~title:"Error on A/C name" "A/C name already exists in this conf"
          else begin
            let a = Hashtbl.find Utils.aircrafts selected_ac_name in
            let af_old = Env.paparazzi_home // "conf" // (ExtXml.attrib a "airframe") in
            let af_new =
              match GToolbox.select_file ~title:"Copy to new airframe file" ~filename:af_old () with
              | None -> af_old
              | Some x -> x
            in
            let af_new =
              if af_old = af_new then af_new
              else
                if Sys.command (sprintf "cp -f %s %s" af_old af_new) = 0 then af_new
                else begin
                  GToolbox.message_box ~title:"Error on airframe copy" ("Using original airframe " // af_old);
                  af_old
                end
            in
            let a = ExtXml.subst_attrib "name" s a in
            let a = ExtXml.subst_attrib "airframe" (Env.filter_absolute_path af_new) a in
            let a = ExtXml.subst_attrib "ac_id" (string_of_int (new_ac_id ())) a in
            (* add to hashtbl before combo to avoid update errors *)
            Hashtbl.add Utils.aircrafts s a;
            Gtk_tools.add_to_combo ac_combo s;
            update_params s
        end
  in
  ignore (gui#menu_item_copy_ac#connect#activate ~callback);

  (* Delete A/C *)
  let callback = fun _ ->
    let ac_name = Gtk_tools.combo_value ac_combo in
    if ac_name <> "" then
      match GToolbox.question_box ~title:"Delete A/C" ~buttons:["Cancel"; "Delete"] ~default:2 (sprintf "Delete %s ? (no undo after Save)" ac_name) with
      | 2 -> begin
          begin try Hashtbl.remove Utils.aircrafts ac_name with _ -> () end;
          let combo_box = Gtk_tools.combo_widget ac_combo in
          match combo_box#active_iter with
          | None -> ()
          | Some row ->
              let (store, _column) = Gtk_tools.combo_model ac_combo in
              ignore (store#remove row);
              combo_box#set_active 1
        end
      | _ -> ()
  in
  ignore (gui#delete_ac_menu_item#connect#activate ~callback);

  (* New Target button *)
  let callback = fun _ ->
    match GToolbox.input_string ~title:"New Target" ~text:"tunnel" "New build target ?" with
    | None -> ()
    | Some s ->
        let (store, column) = Gtk_tools.combo_model target_combo in
        let row = store#append () in
        store#set ~row ~column s;
        (Gtk_tools.combo_widget target_combo)#set_active_iter (Some row)
  in
  ignore (gui#menu_item_new_target#connect#activate ~callback);

  (* GUI color *)
  let callback = fun _ ->
    let csd = GWindow.color_selection_dialog ~show:true () in
    let callback = fun _ ->
      let colorname = string_of_gdkcolor csd#colorsel#color in
      gui#eventbox_gui_color#misc#modify_bg [`NORMAL, `NAME colorname];
      current_color := colorname;
      save_callback gui ac_combo tree_set tree_set_mod ();
      csd#destroy () in
    ignore (csd#ok_button#connect#clicked ~callback);
    ignore (csd#cancel_button#connect#clicked ~callback:csd#destroy) in
  ignore(gui#button_gui_color#connect#clicked ~callback);

  (* A/C id *)
  ignore(gui#entry_ac_id#connect#changed ~callback:(fun () -> save_callback gui ac_combo tree_set tree_set_mod ()));

  let callback = fun _ ->
    update_params (Gtk_tools.combo_value ac_combo);
    save_callback gui ac_combo tree_set tree_set_mod () in
  (* refresh button *)
  ignore(gui#button_refresh#connect#clicked ~callback);
  (* update with build and upload button *)
  ignore(gui#button_build#connect#clicked ~callback);
  ignore(gui#button_upload#connect#clicked ~callback);

  (* Conf *)
  List.iter (fun (name, subdir, label, button_browse, button_edit, editor, button_remove) ->
    (* editor button callback *)
    let callback = fun _ ->
      let rel_files = match label with
                        Label l -> Str.split regexp_space l#text
                      | Tree t -> Str.split regexp_space (Gtk_tools.tree_values ~only_checked:true t)
      in
      let abs_files = List.map (Filename.concat Utils.conf_dir) rel_files in
      let quoted_files = List.map (fun s -> "'"^s^"'") abs_files in
      let arg = String.concat " " quoted_files in
      editor arg in
    (* connect editor button *)
    ignore (match button_edit with Some e -> ignore(e#connect#clicked ~callback) | _ -> ());

    (* browse button callback *)
    let callback = fun _ ->
      let cb = fun names ->
        ignore (match label with
          Label l ->
            let names = String.concat " " names in
            l#set_text names
        | Tree t ->
            List.iter (Gtk_tools.add_to_tree t) names
        );
        save_callback gui ac_combo tree_set tree_set_mod ();
        let ac_name = Gtk_tools.combo_value ac_combo in
        update_params ac_name
      in
      Utils.choose_xml_file name subdir cb in
    (* connect browse button *)
    ignore (match button_browse with Some b -> ignore(b#connect#clicked ~callback) | _ -> ());

    (* remove button callback *)
    let callback = fun _ ->
      match label with
      Tree t ->
        Gtk_tools.remove_selected_from_tree t;
        save_callback gui ac_combo tree_set tree_set_mod ()
        | _ -> ()
    in
    (* connect remove button *)
    ignore (match button_remove with Some r -> ignore(r#connect#clicked ~callback) | _ -> ())
  )
  ac_files;

  (* Tag Current Commit-Aircraft *)
  let callback = fun _ ->
    match GToolbox.question_box ~title:"Mark Test-flight Successfull" ~default:2 ~buttons:["Yes"; "Cancel"] "Are you sure you tested this airframe in all its modes (e.g. GPS) and confirm all works well." with
    | 1 ->
        begin
        gui#label_release#set_text (tag_this_version () );
        save_callback gui ac_combo tree_set tree_set_mod ();
        let ac_name = Gtk_tools.combo_value ac_combo in
        update_params ac_name

        end
    | _ -> ()
  in
  ignore (gui#button_store_release#connect#clicked ~callback);

  (* Compare *)
  let callback = fun _ ->
    ignore (show_gitk_of_version gui#label_release#text)
  in
  ignore (gui#button_compare_release#connect#clicked ~callback);

  (* Browse Version *)
  let callback = fun _ ->
    gitk_version gui#label_release#text
  in
  ignore (gui#button_gitk#connect#clicked ~callback);



  (* Save button *)
  ignore(gui#menu_item_save_ac#connect#activate ~callback:(save_callback ~user_save:true gui ac_combo tree_set tree_set_mod))


let build_handler = fun ~file gui ac_combo (target_combo:Gtk_tools.combo) (flash_combo:Gtk_tools.combo) (log:string->unit) ->
  (* Link target to upload button *)
  Gtk_tools.combo_connect target_combo
    (fun target ->
      let ac_name = Gtk_tools.combo_value ac_combo in
      let aircraft = Hashtbl.find Utils.aircrafts ac_name in
      parse_ac_flash (Gtk_tools.combo_value target_combo) flash_combo (ExtXml.attrib aircraft "airframe");
      (* if target is sim or nps, deactivate the upload button *)
      gui#button_upload#misc#set_sensitive (target <> "sim" && target <> "nps"));

  (* Clean button *)
  let callback = fun () ->
    Utils.command ~file gui log (Gtk_tools.combo_value ac_combo) "clean_ac" in
  ignore (gui#button_clean#connect#clicked ~callback);

  (* Build button *)
  let callback = fun () ->
    try (
      let ac_name = Gtk_tools.combo_value ac_combo
      and target = Gtk_tools.combo_value target_combo
      and config = if gui#checkbutton_printconfig#active then "PRINT_CONFIG=1 " else "" in
      let target_cmd = sprintf "%s%s.compile" config target in
      gui#button_build#misc#set_sensitive false;
      gui#button_upload#misc#set_sensitive false;
      let finished_callback = fun () ->
        gui#button_build#misc#set_sensitive true;
        gui#button_upload#misc#set_sensitive true in
      Utils.command ~file ~finished_callback gui log ac_name target_cmd
    ) with _ -> log "ERROR: Nothing to build!!!\n" in
    ignore (gui#button_build#connect#clicked ~callback);

  (* Upload button *)
  let callback = fun () ->
    let ac_name = Gtk_tools.combo_value ac_combo
    and target = Gtk_tools.combo_value target_combo
    and flash = Gtk_tools.combo_value flash_combo
    and config = if gui#checkbutton_printconfig#active then "PRINT_CONFIG=1 " else "" in
    let options = try Hashtbl.find (fst CP.flash_modes) flash with _ -> "" in
    let target_cmd = sprintf "%s%s %s.upload" config options target in
    gui#button_build#misc#set_sensitive false;
    gui#button_upload#misc#set_sensitive false;
    let finished_callback = fun () ->
      gui#button_build#misc#set_sensitive true;
      gui#button_upload#misc#set_sensitive true in
    Utils.command ~file ~finished_callback gui log ac_name target_cmd in
  ignore (gui#button_upload#connect#clicked ~callback)

