(*
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
  val mutable last_known_value = None
  method last_known_value =
    match last_known_value with
    | None -> raise Not_found
    | Some v ->
        let auc = PprzLink.alt_unit_coef_of_xml xml in
        let (alt_a, alt_b) = Ocaml_tools.affine_transform auc in
        (v -. alt_b) /. alt_a
  method current_value =
    let auc = PprzLink.alt_unit_coef_of_xml xml in
    let (alt_a, alt_b) = Ocaml_tools.affine_transform auc in
    (float_of_string current_value#text -. alt_b) /. alt_a
  method update = fun s ->
    (* if not yet confirmed, display "?" *)
    if s = "?" then
      current_value#set_text "?"
    else
      if current_value#text <> s then begin
        current_value#set_text s;
        try
          let v = float_of_string s in
          last_known_value <- Some v;
          set_default v
        with Failure "float_of_string" -> ()
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
  let key, modifiers = GtkData.AccelGroup.parse (Env.key_modifiers_of_string (Xml.attrib xml "key"))
  and value = ExtXml.float_attrib xml "value" in
  keys := (key, (modifiers, fun () -> do_change value)) :: !keys



let one_setting = fun (i:int) (do_change:int -> float -> unit) ac_id packing dl_setting (tooltips:GData.tooltips) icons_theme strip keys ->
  let f = fun a -> float_of_string (ExtXml.attrib dl_setting a) in
  let lower = f "min"
  and upper = f "max"
  and step_incr =
    try f "step" with _ ->
      fprintf stderr "Warning: 'step' attribute missing in '%s' setting. Default to 1\n%!" (Xml.to_string dl_setting);
      1.
  in
  (* get number of digits after decimal dot *)
  let digits = try String.length (ExtXml.attrib dl_setting "step") - String.index (ExtXml.attrib dl_setting "step") '.' - 1 with _ -> 0 in
  let page_incr = step_incr
  and page_size = step_incr
  and show_auto = try ExtXml.attrib dl_setting "auto" = "true" with _ -> false in
  let auc = PprzLink.alt_unit_coef_of_xml dl_setting in
  let (alt_a, alt_b) = Ocaml_tools.affine_transform auc in

  let hbox = GPack.hbox ~packing () in
  let varname = ExtXml.attrib dl_setting "var" in
  let text = try ExtXml.attrib dl_setting "shortname" with _ -> varname in
  let _l = GMisc.label ~width:100 ~text ~packing:hbox#pack () in
  let eb = GBin.event_box ~packing:hbox#pack () in
  let current_value = GMisc.label ~width:80 ~text:"N/A" ~packing:eb#add () in

  let auto_but = GButton.check_button ~label:"Auto" ~active:false () in

  (** Either choose type of widged explicitly by 'widget' attribute:
      Allowed attribute values: "radio_button", "combo_box", "slider", "spin_button"
      For a small number of values, radio buttons,
      For a large number of values, combo box,
      For float values with range up to 2^16, slider
      else spin button.
  *)
  let values = values_of_dl_setting dl_setting
  and modified = ref false in
  let widget_attrib = try ExtXml.attrib dl_setting "widget" with _ -> "auto" in
  let widget_t =
    if Str.string_match (Str.regexp_case_fold "radio.*") widget_attrib 0 then
      "radio_button"
    else if Str.string_match (Str.regexp_case_fold "combo.*") widget_attrib 0 then
      "combo_box"
    else if Str.string_match (Str.regexp_case_fold "slider.*") widget_attrib 0 then
      "slider"
    else if Str.string_match (Str.regexp_case_fold "spin.*") widget_attrib 0 then
      "spin_button"
    else (* auto *)
        if step_incr = 1. && upper -. lower <= 2. || Array.length values > 0 then
          if Array.length values > 2 then (* Combo box *)
           "combo_box"
          else (* radio buttons *)
            "radio_button"
        else (* no values given, slider or spin button *)
          let range = upper -. lower in
          if range > 65536. then (* spin button *)
            "spin_button"
          else
            "slider"
  in
  let commit, set_default =
    if widget_t = "radio_button" || widget_t = "combo_box" then
      (* Discrete values *)
      let value = ref lower in
      let callback = fun () -> do_change i !value in
      let update_value = fun index ->
        modified := true;
        value := float index;
        if auto_but#active then callback () in
      if widget_t = "combo_box" then (* Combo box *)
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
    else (* no values given, slider or spin button *)
      let value = (lower +. upper) /. 2. in
      if widget_t = "spin_button" then
        let adj = GData.adjustment ~value ~lower ~upper:(upper+.step_incr) ~step_incr ~page_incr ~page_size:0. () in
        let _spinbutton = GEdit.spin_button ~adjustment:adj ~digits ~numeric:true ~packing:hbox#add () in
        let f = fun _ -> do_change i ((adj#value-.alt_b)/.alt_a)  in
        let callback = fun () -> modified := true; if auto_but#active then f () in
        ignore (adj#connect#value_changed ~callback);
        ignore (auto_but#connect#toggled ~callback);
        (f, fun x -> try adj#set_value x with _ -> ())
      else
        let adj = GData.adjustment ~value ~lower ~upper:(upper+.step_incr) ~step_incr ~page_incr ~page_size () in
        let _scale = GRange.scale `HORIZONTAL ~digits ~update_policy:`DELAYED ~adjustment:adj ~packing:hbox#add () in
        let f = fun _ -> do_change i ((adj#value-.alt_b)/.alt_a)  in
        let callback = fun () -> modified := true; if auto_but#active then f () in
        ignore (adj#connect#value_changed ~callback);
        ignore (auto_but#connect#toggled ~callback);
        (f, fun x -> try adj#set_value x with _ -> ())
  in
  let set_default = fun x ->
    if not !modified then set_default x else () in

  (* build setting *)
  let setting = new setting i dl_setting current_value set_default in

  (* click current_value label to request an update, a value of infinity for do_change requests new value *)
  let callback = fun _ ->
    do_change i infinity;
    current_value#set_text "?";
    true in
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
  let idx = ref 0 in
  let callback = fun x ->
    prev_value := (try Some setting#last_known_value with _ ->
      idx := -1;
      Array.iteri (fun i v -> if current_value#text = v then idx := i) values;
      if !idx >= 0 then Some (lower +. (float_of_int !idx)) else None);
    commit x;
    current_value#set_text "?"
  in
  ignore (commit_but#connect#clicked ~callback);
  tooltips#set_tip commit_but#coerce ~text:"Commit";
  tooltips#set_tip current_value#coerce ~text:"Current value, click to request update.";

  (* Undo button *)
  let undo_but = GButton.button ~packing:hbox#pack () in
  let _icon = GMisc.image ~stock:`UNDO ~packing:undo_but#add () in
  let callback = fun _ ->
    match !prev_value with
        None -> ()
      | Some v -> current_value#set_text "?"; do_change i v in
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
              let pixbuf = GdkPixbuf.from_file (Env.get_gcs_icon_path icons_theme icon) in
              ignore (GMisc.image ~pixbuf ~packing:b#add ());

              (* Drag for Drop *)
              let papget = Papget_common.xml "variable_setting" "button"
                ["variable", varname;
                 "value", ExtXml.attrib x "value";
                 "ac_id", ac_id;
                 "icon", icons_theme // icon] in
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

  (* return setting *)
  setting



let same_tag_for_all = function
    [] -> failwith "Page_settings: unreachable, empty dl_settings element"
  | x::xs ->
    let tag_first = Xml.tag x in
    List.iter (fun y -> assert(ExtXml.tag_is y tag_first)) xs;
    String.lowercase tag_first


(** Build the tree of settings *)
let rec build_settings = fun do_change ac_id i flat_list keys xml_settings packing tooltips icons_theme strip ->
  match same_tag_for_all xml_settings with
      "dl_setting" ->
        List.iter
          (fun dl_setting ->
            let label_value = one_setting !i do_change ac_id packing dl_setting tooltips icons_theme strip keys in
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
        build_settings do_change ac_id i flat_list keys children vbox#pack tooltips icons_theme strip)
        xml_settings
    | tag -> failwith (sprintf "Page_settings.build_settings, unexpected tag '%s'" tag)


class settings = fun ?(visible = fun _ -> true) xml_settings do_change ac_id icons_theme strip ->
  let sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
  let vbox = GPack.vbox ~packing:sw#add_with_viewport () in
  let tooltips = GData.tooltips () in
  let i = ref 0 and l = ref [] and keys = ref [] in
  let ordered_list =
    build_settings do_change ac_id i l keys xml_settings vbox#add tooltips icons_theme strip;
    List.rev !l in
  let variables = Array.of_list ordered_list in
  let length = Array.length variables in
  let assocs =
    List.map (fun setting -> (ExtXml.attrib setting#xml "var", setting#index)) ordered_list in
object (self)
  method widget = sw#coerce
  method length = length
  method keys = !keys
  method set = fun i value ->
    if visible self#widget then
      let setting = variables.(i) in
      let s, v = match value with
        | None -> "?", -1
        | Some x ->
          let v = try float_of_string x with _ -> failwith (sprintf "Pages.settings#set:wrong values.(%d) = %s" i x) in
          let auc = PprzLink.alt_unit_coef_of_xml setting#xml in
          let (alt_a, alt_b) = Ocaml_tools.affine_transform auc in
          let v = alt_a *. v +. alt_b in
          string_of_float v, truncate v
      in
      if i < 0 || i >= Array.length variables then
        failwith (sprintf "Pages.settings#set: %d out of bounnds (length=%d)" i (Array.length variables));
      let s =
        let values = values_of_dl_setting setting#xml in
        try
          let lower = int_of_string (ExtXml.attrib setting#xml "min") in
          values.(v - lower)
        with
            _ -> s in
      setting#update s
  method assoc var = List.assoc var assocs
  method save = fun airframe_filename ->
    let settings = Array.fold_right (fun setting r -> try (setting#index, setting#xml, setting#last_known_value)::r with _ -> r) variables [] in
    SaveSettings.popup airframe_filename (Array.of_list settings) do_change
end

