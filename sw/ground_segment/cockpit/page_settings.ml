(*
* $Id$
*
* Widget to pack settings buttons
*
* Copyright (C) 2004-2009 ENAC, Pascal Brisset, Antoine Drouin
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

let (//) = Filename.concat


class setting = fun (i:int) (xml:Xml.xml) (current_value:GMisc.label) set_default ->
  object
    method index = i
    method xml = xml
    method current_value =
      let auc = Pprz.alt_unit_coef_of_xml xml in
      let (alt_a, alt_b) = Ocaml_tools.affine_transform auc in
      (float_of_string current_value#text -. alt_b) /. alt_a
    method update = fun s ->
      if current_value#text <> s then begin
	current_value#set_text s;
	try set_default (float_of_string s) with Failure "float_of_string" -> ()
      end
  end

let pipe_regexp = Str.regexp "|"
let values_of_dl_setting = fun dl_setting ->
  try
    Array.of_list (Str.split pipe_regexp (Xml.attrib dl_setting "values"))
  with
    _ -> [||]


(* Look for the index of a value in a array. May raise Not_found *)
let search_index = fun value array ->
  let i = ref 0 in
  while !i < Array.length array && value <> array.(!i) do incr i done;
  if !i < Array.length array then !i else raise Not_found


let add_key = fun xml do_change keys ->
  let key, modifiers = GtkData.AccelGroup.parse (Xml.attrib xml "key")
  and value = ExtXml.float_attrib xml "value" in
  keys := (key, (modifiers, fun () -> do_change value)) :: !keys



let one_setting = fun (i:int) (do_change:int -> float -> unit) packing dl_setting (tooltips:GData.tooltips) strip keys ->
  let f = fun a -> float_of_string (ExtXml.attrib dl_setting a) in
  let lower = f "min"
  and upper = f "max"
  and step_incr =
    try f "step" with _ ->
      fprintf stderr "Warning: 'step' attribute missing in '%s' setting. Default to 1\n%!" (Xml.to_string dl_setting);
      1.
  in
  let page_incr = step_incr
  and page_size = step_incr
  and show_auto = try ExtXml.attrib dl_setting "auto" = "true" with _ -> false in
  let auc = Pprz.alt_unit_coef_of_xml dl_setting in
  let (alt_a, alt_b) = Ocaml_tools.affine_transform auc in

  let hbox = GPack.hbox ~packing () in
  let varname = ExtXml.attrib dl_setting "var" in
  let text = try ExtXml.attrib dl_setting "shortname" with _ -> varname in
  let _l = GMisc.label ~width:100 ~text ~packing:hbox#pack () in
  let eb = GBin.event_box ~packing:hbox#pack () in
  let current_value = GMisc.label ~width:50 ~text:"N/A" ~packing:eb#add () in

  let auto_but = GButton.check_button ~label:"Auto" ~active:false () in

  (** For a small number of values, radio buttons,
      For a large number of values, combo box,
      else a slider *)
  let values = values_of_dl_setting dl_setting
  and modified = ref false in
  let commit, set_default =
    if step_incr = 1. && upper -. lower <= 2. || Array.length values > 0 then
      (* Discrete values *)
      let value = ref lower in
      let callback = fun () -> do_change i !value in
      let update_value = fun index ->
	modified := true;
	value := float index;
	if auto_but#active then callback () in
      if Array.length values > 2 then (* Combo box *)
	let strings = Array.to_list values in
	let combo = Gtk_tools.combo strings hbox in

	let update_string = fun string ->
	  try
	    update_value ((search_index string values) + truncate lower)
	  with
	    Not_found -> failwith (sprintf "Internal error: Settings, %s not found" string) in
	Gtk_tools.combo_connect combo update_string;

	(callback, fun j -> try Gtk_tools.select_in_combo combo values.(truncate j) with _ -> ())
      else (* radio buttons *)
	let ilower = truncate lower
	and iupper = truncate upper in
	let callback = fun _ -> do_change i !value in
	let group = (GButton.radio_button ())#group in (* Group shared by the buttons *)
	let buttons = Array.init (iupper-ilower+1)
	    (fun j ->
	      (* Build the button *)
	      let label =
		if Array.length values = 0
		then Printf.sprintf "%d" (ilower + j)
		else values.(j) in
	      let b = GButton.radio_button ~group ~label ~packing:hbox#add () in

	      (* Connect the event *)
	      ignore (b#connect#pressed (fun () -> update_value (ilower + j)));
	      b) in
	(callback, fun j -> try buttons.(truncate j - ilower)#set_active true with _ -> ())
    else (* slider *)
      let value = (lower +. upper) /. 2. in
      let adj = GData.adjustment ~value ~lower ~upper:(upper+.step_incr) ~step_incr ~page_incr ~page_size () in
      let _scale = GRange.scale `HORIZONTAL ~digits:3 ~update_policy:`DELAYED ~adjustment:adj ~packing:hbox#add () in
      let f = fun _ -> do_change i ((adj#value-.alt_b)/.alt_a)  in
      let callback = fun () -> modified := true; if auto_but#active then f () in
      ignore (adj#connect#value_changed ~callback);
      ignore (auto_but#connect#toggled ~callback);
      (f, fun x -> try adj#set_value x with _ -> ())
  in
  let set_default = fun x ->
    if not !modified then set_default x else () in

  (* Update value *)
  let callback = fun _ ->
    do_change i infinity; true in
  ignore (eb#event#connect#button_press ~callback);

  (* Auto check button *)
  if show_auto then begin
    hbox#pack auto_but#coerce
  end;
  (* Apply button *)
  let prev_value = ref None in
  let commit_but = GButton.button ~packing:hbox#pack () in
  commit_but#set_border_width 2;
  let _icon = GMisc.image ~stock:`APPLY ~packing:commit_but#add () in
  let callback = fun x ->
    prev_value := (try Some ((float_of_string current_value#text-.alt_b)/.alt_a) with _ -> None);
    commit x
  in
  ignore (commit_but#connect#clicked ~callback);
  tooltips#set_tip commit_but#coerce ~text:"Commit";

  (* Undo button *)
  let undo_but = GButton.button ~packing:hbox#pack () in
  let _icon = GMisc.image ~stock:`UNDO ~packing:undo_but#add () in
  let callback = fun _ ->
    match !prev_value with
      None -> ()
    | Some v -> do_change i v in
  ignore (undo_but#connect#clicked ~callback);
  tooltips#set_tip undo_but#coerce ~text:"Undo";

  ignore (auto_but#connect#toggled
    (fun () ->
      commit_but#misc#set_sensitive (not auto_but#active);
      undo_but#misc#set_sensitive (not auto_but#active)));

  (** Insert the related buttons in the strip and prepare the papgets DnD *)
  List.iter (fun x ->
    match String.lowercase (Xml.tag x) with
      "strip_button" ->
	let label = ExtXml.attrib x "name"
	and sp_value = ExtXml.float_attrib x "value"
        and group = ExtXml.attrib_or_default x "group" "" in
	let b =
	  try (* Is it an icon ? *)
	    let icon = Xml.attrib x "icon" in
	    let b = GButton.button () in
	    let pixbuf = GdkPixbuf.from_file (Env.gcs_icons_path // icon) in
	    ignore (GMisc.image ~pixbuf ~packing:b#add ());

	    (* Drag for Drop *)
	    let papget = Papget_common.xml "variable_setting" "button"
		["variable", varname;
		 "value", ExtXml.attrib x "value";
		 "icon", icon] in
	    Papget_common.dnd_source b#coerce papget;

            (* Associates the label as a tooltip *)
	    tooltips#set_tip b#coerce ~text:label;
	    b
	  with
	    Xml.No_attribute "icon" -> GButton.button ~label ()
	  | exc ->
	      prerr_endline (Printexc.to_string exc);
	      GButton.button ~label () in
	(strip group b#coerce: unit);
	ignore (b#connect#clicked (fun _ -> do_change i sp_value))
    | "key_press" -> add_key x (do_change i) keys
    | t -> failwith (sprintf "Page_settings.one_setting, Unexpected tag: '%s'" t))
    (Xml.children dl_setting);

  new setting i dl_setting current_value set_default



let same_tag_for_all = function
    [] -> failwith "Page_settings: unreachable, empty dl_settings element"
  | x::xs ->
      let tag_first = Xml.tag x in
      List.iter (fun y -> assert(ExtXml.tag_is y tag_first)) xs;
      String.lowercase tag_first


(** Build the tree of settings *)
let rec build_settings = fun do_change i flat_list keys xml_settings packing tooltips strip ->
  match same_tag_for_all xml_settings with
    "dl_setting" ->
      List.iter
	(fun dl_setting ->
	  let label_value = one_setting !i do_change packing dl_setting tooltips strip keys in
	  flat_list := label_value :: !flat_list;
	  incr i)
	xml_settings
  | "dl_settings" ->
      let n = GPack.notebook ~packing () in

      List.iter (fun dl_settings ->
	let text = ExtXml.attrib dl_settings "name" in
	let _sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
	let vbox = GPack.vbox  () in

	let tab_label = (GMisc.label ~text ())#coerce in
	ignore (n#append_page ~tab_label vbox#coerce);

	let children = Xml.children dl_settings in
	build_settings do_change i flat_list keys children vbox#pack tooltips strip)
	xml_settings
  | tag -> failwith (sprintf "Page_settings.build_settings, unexpected tag '%s'" tag)


class settings = fun ?(visible = fun _ -> true) xml_settings do_change strip ->
  let sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
  let vbox = GPack.vbox ~packing:sw#add_with_viewport () in
  let tooltips = GData.tooltips () in
  let i = ref 0 and l = ref [] and keys = ref [] in
  let ordered_list =
    build_settings do_change i l keys xml_settings vbox#add tooltips strip;
    List.rev !l in
  let variables = Array.of_list ordered_list in
  let length = Array.length variables in
  let assocs =
    List.map (fun setting -> (ExtXml.attrib setting#xml "var", setting#index)) ordered_list in
  object (self)
    method widget = sw#coerce
    method length = length
    method keys = !keys
    method set = fun i v ->
      if visible self#widget then
	let setting = variables.(i) in
	let auc = Pprz.alt_unit_coef_of_xml setting#xml in
	let (alt_a, alt_b) = Ocaml_tools.affine_transform auc in
	let v = alt_a *. v +. alt_b in
	let s = string_of_float v in
	if i < 0 || i >= Array.length variables then
	  failwith (sprintf "Pages.settings#set: %d out of bounnds (length=%d)" i (Array.length variables));
	let s =
	  let values = values_of_dl_setting setting#xml in
	  try
      let lower = int_of_string (ExtXml.attrib setting#xml "min") in
	    values.(truncate v - lower)
	  with
	    _ -> s in
	setting#update s
    method assoc var = List.assoc var assocs
    method save = fun airframe_filename ->
      let settings = Array.fold_right (fun setting r -> try (setting#index, setting#xml, setting#current_value)::r with _ -> r) variables [] in
      SaveSettings.popup airframe_filename (Array.of_list settings) do_change
  end


