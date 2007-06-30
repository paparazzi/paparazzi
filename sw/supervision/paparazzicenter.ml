open Printf

let (//) = Filename.concat
let bn = Filename.basename
let conf_dir = Env.paparazzi_home // "conf"

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
		"settings", "settings/tuning.xml";
		"gui_color", "blue"],
	       [])

				   

let control_panel_xml = Xml.parse_file (conf_dir // "control_panel.xml")
let programs =
  let h = Hashtbl.create 7 in
  let s = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = "programs") control_panel_xml "section" in
  List.iter 
    (fun p -> Hashtbl.add h (ExtXml.attrib p "name") p)
    (Xml.children s);
  h
let program_command = fun x ->
  let xml = Hashtbl.find programs x in
  ExtXml.attrib xml "command"

let conf_xml_file = conf_dir // "conf.xml"
let conf_xml = Xml.parse_file conf_xml_file
let aircrafts = Hashtbl.create 7
let _ =
  List.iter (fun aircraft ->
    Hashtbl.add aircrafts (ExtXml.attrib aircraft "name") aircraft)
    (Xml.children conf_xml)
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


type combo = GEdit.combo_box * (GTree.list_store * string GTree.column)

let combo = fun ?(others = []) strings vbox ->
  let strings = others @ strings in
  let (combo, (tree, column)) =
    GEdit.combo_box_text ~packing:vbox#add ~strings () in
  combo#set_active 0;
  (combo, (tree, column))

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
      
let combo_widget = fst
let combo_model = snd


let is_xml_file = fun s ->
  let n = String.length s in
  n >= 4 && String.sub s (n-4) 4 = ".xml"

let combo_dir = fun ?others directory vbox ->
  let files = Array.to_list (Sys.readdir directory) in
  let xml_files = List.filter is_xml_file files in
  combo ?others xml_files vbox


let command = fun log ac_name target ->
  let com = sprintf "cd %s; export PATH=/usr/bin:$PATH; make AIRCRAFT=%s %s" Env.paparazzi_home ac_name target in
  log com;
  let com_stdout, com_stdin, com_stderr = Unix.open_process_full com [||] in
  let channel_out = GMain.Io.channel_of_descr (Unix.descr_of_in_channel com_stdout) in
  let channel_err = GMain.Io.channel_of_descr (Unix.descr_of_in_channel com_stderr) in
  let cb = fun c _ -> log (input_line c); true in
  let io_watch_out = Glib.Io.add_watch [`IN] (cb com_stdout) channel_out in
  let io_watch_err = Glib.Io.add_watch [`IN] (cb com_stderr) channel_err in
  ignore (Glib.Io.add_watch [`HUP] (fun _ -> Glib.Io.remove io_watch_out; Glib.Io.remove io_watch_err; log "\nDONE\n"; false) channel_out)


let run = prerr_endline


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
  ["airframe", gui#label_airframe, gui#button_browse_airframe, gui#button_edit_airframe, edit;
   "flight_plan", gui#label_flight_plan, gui#button_browse_flight_plan, gui#button_edit_flight_plan, gcs_or_edit;
(*    "settings", gui#label_settings, gui#button_browse_settings, gui#button_edit_settings, edit; *)
   "radio", gui#label_radio, gui#button_browse_radio, gui#button_edit_radio, edit;
   "telemetry", gui#label_telemetry, gui#button_browse_telemetry, gui#button_edit_telemetry, edit]


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
	  (fun (a, label, _, _, _) -> label#set_text (value a))
	  (ac_files gui);
	gui#entry_settings#set_text
	  (value "settings");
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
			  "settings", gui#entry_settings#text;
			  "gui_color", color],
			 []) in
	  begin try Hashtbl.remove aircrafts ac_name with _ -> () end;
	  Hashtbl.add aircrafts ac_name aircraft
	end;
	write_conf_xml ()
    | _ -> () in
  ignore(gui#button_save_ac#connect#clicked ~callback)


let build_handler = fun gui ac_combo target_combo log ->
  (* Link target to upload button *)
  combo_connect target_combo
    (fun target ->
      gui#button_upload#misc#set_sensitive (target <> "sim"));

  (* Clean button *)
  let callback = fun () ->
    command log (combo_value ac_combo) "clean_ac" in
  ignore (gui#button_clean#connect#clicked ~callback);
  
  (* Build button *)
  let callback = fun () ->
    let ac_name = combo_value ac_combo
    and target = combo_value target_combo in
    command log ac_name target in
  ignore (gui#button_build#connect#clicked ~callback);
  
  (* Upload button *)
  let callback = fun () ->
    let ac_name = combo_value ac_combo
    and target = combo_value target_combo in
    command log ac_name (sprintf "%s.upload" target) in
  ignore (gui#button_upload#connect#clicked ~callback)

let choose_xml_file = fun title subdir cb ->
  let dir = conf_dir // subdir in
  let dialog = GWindow.file_chooser_dialog ~action:`OPEN ~title () in
  ignore (dialog#set_current_folder dir);
  dialog#add_filter (GFile.filter ~name:"log" ~patterns:["*.xml"] ());
  dialog#add_button_stock `CANCEL `CANCEL ;
  dialog#add_select_button_stock `OPEN `OPEN ;
  begin match dialog#run (), dialog#filename with
    `OPEN, Some name ->
      dialog#destroy ();
      cb (bn name)
  | _ -> dialog#destroy ()
  end

let first_word = fun s ->
  try
    let n = String.index s ' ' in
    String.sub s 0 n
  with
    Not_found -> s

let conf_handler = fun gui ->
  List.iter (fun (name, label, button_browse, button_edit, editor) ->
    let callback = fun _ ->
      editor (conf_dir // label#text) in
    ignore (button_edit#connect#clicked ~callback);
    let callback = fun _ ->
      let subdir = Filename.dirname label#text in
      let cb = fun name -> label#set_text (subdir // name) in
      choose_xml_file name subdir cb in
    ignore (button_browse#connect#clicked ~callback))
    (ac_files gui);
  (* Special case for settings (not a single file) *)
  let callback = fun _ ->
    edit (conf_dir // (first_word gui#entry_settings#text)) in
  ignore (gui#button_edit_settings#connect#clicked ~callback)


let supervision = fun gui ->
  (* Replay menu *)
  ()

  (* GCS button
  let callback = fun () ->
    match (combo_value session_combo) with
      "SIM" ->
	let ac_name = combo_value ac_combo in
	let sim_com = program_command "sim"
	and server = program_command "server"
	and gcs_com = program_command "gcs" in
	let sim = sprintf "%s -a %s" sim_com ac_name
	and gcs = sprintf "%s" gcs_com in

	run sim;
	run server;
	run gcs
    | x -> fprintf stderr "%s not yet\n" x
  in
  ignore (gui#button_GCS#connect#clicked ~callback);
*)

   

  

let () =
  let file = Env.paparazzi_src // "sw" // "supervision" // "paparazzicenter.glade" in
  let gui = new Gui.window ~file () in
  ignore (gui#window#connect#destroy ~callback:(fun _ -> exit 0));
  gui#toplevel#show ();

  let ac_combo = parse_conf_xml gui#vbox_ac
  and target_combo = combo ["sim";"fbw";"ap"] gui#vbox_target in

  (combo_widget target_combo)#misc#set_sensitive false;
  gui#button_clean#misc#set_sensitive false;
  gui#button_build#misc#set_sensitive false;

  ac_combo_handler gui ac_combo target_combo;

  conf_handler gui;

  let log = fun s ->
    gui#console#buffer#insert s;
    gui#console#buffer#insert "\n";
    (* Scroll to the bottom line *)
    let end_iter = gui#console#buffer#end_iter in
    let end_mark = gui#console#buffer#create_mark end_iter in
    gui#console#scroll_mark_onscreen (`MARK end_mark) in

  build_handler gui ac_combo target_combo log;

  supervision gui;

  GMain.Main.main ()
