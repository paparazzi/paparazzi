(*
 * GUI to save settings in the airframe file
 *
 * Copyright (C) 2008, Cyril Allignol, Pascal Brisset
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


module U = Unix

(** How to have them local ? *)
let cols = new GTree.column_list
let col_index = cols#add Gobject.Data.int
let col_param = cols#add Gobject.Data.string
and col_airframe_value = cols#add Gobject.Data.string
and col_settings_value = cols#add Gobject.Data.string
and col_airframe_value_new = cols#add Gobject.Data.string
and col_code_value = cols#add Gobject.Data.float
and col_to_save = cols#add Gobject.Data.boolean
and col_integer = cols#add Gobject.Data.boolean

let (//) = Filename.concat

(** Float not equal to 0.1% *)
let floats_not_equal = fun f1 f2 ->
  f2 = 0. && f1 <> 0. ||
  let r = abs_float (f1 /. f2) in
  r < 0.999 || r > 1.001


(** The save file dialog box *)
let save_airframe = fun w filename save ->
  match GToolbox.select_file ~title:"Save Airframe" ~filename () with
      None -> ()
    | Some file ->
      save file;
      w#save_settings#destroy ()

(** Toggling a tree element *)
let item_toggled ~(model : GTree.tree_store) ~column path =
  let row = model#get_iter path in
  let b = model#get ~row ~column in
  model#set ~row ~column (not b)


let display_columns = fun w model ->
  let renderer = GTree.cell_renderer_text [`XALIGN 0.] in
  let vc = GTree.view_column ~title:"Parameter" ~renderer:(renderer, ["text", col_param]) () in
  ignore (w#treeview_settings#append_column vc);
  let text_columns = [(col_airframe_value, "Airframe Value"); (col_settings_value, "Setting Value")] in
  List.iter (fun (col, title) ->
    let renderer = GTree.cell_renderer_text [`XALIGN 0.] in
    let vc = GTree.view_column ~title ~renderer:(renderer, ["text", col]) () in
    vc#set_clickable true;
    ignore (w#treeview_settings#append_column vc))
    text_columns;
  let renderer = GTree.cell_renderer_toggle [`XALIGN 0.] in
  let vc = GTree.view_column ~renderer:(renderer, ["active", col_to_save]) () in
  let save_all = GButton.check_button ~draw_indicator:true ~active:false () in
  vc#set_widget (Some save_all#coerce);
  vc#set_clickable true;

  (* Connect the column header click to the save_all check button *)
  ignore (vc#connect#clicked ~callback:(fun () -> save_all#clicked ()));
  ignore (renderer#connect#toggled ~callback:(item_toggled ~model ~column:col_to_save));
  ignore (w#treeview_settings#append_column vc);

  (** Connect the save_all button to all the rows*)
  let callback = fun () ->
    model#foreach  (fun _path row ->
      model#set ~row ~column:col_to_save save_all#active; false) in
  ignore (save_all#connect#toggled ~callback)



let write_xml = fun (model:GTree.tree_store) old_file airframe_xml file ->
  let new_xml = ref airframe_xml in
  model#foreach (fun _path row ->
    if model#get ~row ~column:col_to_save then begin
      let new_value = model#get ~row ~column:col_airframe_value_new
      and param = model#get ~row ~column:col_param in
      new_xml := EditAirframe.set !new_xml param new_value
    end;
    false);
  if old_file = file then begin
    let now = U.localtime (Unix.gettimeofday ()) in
    let backup_file = Printf.sprintf "%s.%d-%02d-%02d_%02d%02d%02d" old_file (now.U.tm_year + 1900) (now.U.tm_mon+1) now.U.tm_mday now.U.tm_hour now.U.tm_min now.U.tm_sec in
    Sys.rename old_file backup_file
  end;
  XmlCom.to_file !new_xml file



let send_airframe_values = fun (model:GTree.tree_store) send_value ->
  model#foreach (fun _path row ->
    if model#get ~row ~column:col_to_save then begin
      let index = model#get ~row ~column:col_index
      and code_value = model#get ~row ~column:col_code_value in
      send_value index code_value
    end;
    false)



let fill_data = fun (model:GTree.tree_store) settings airframe_xml ->
  let not_in_airframe_file = ref [] in
  Array.iter (fun (index, dl_setting, value) ->
    let attrib = fun a -> Xml.attrib dl_setting a in
    try
      let param = attrib "param" in
      let (airframe_value, airframe_unit, code_unit) = EditAirframe.get airframe_xml param in
      (*
       * Get the scaling between the unit set in the airframe file to the real unit used (code_unit)
       * Print error and use a factor of 1 when code_unit (in airframe file) and unit (in settings file) not equal
       *)
      let unit_setting = try Some (attrib "unit") with _ -> None in
      let airframe_scale =
        try
          let unit_code =
            match code_unit, unit_setting with
              | Some uc, Some us ->
                if uc = us then uc
                else invalid_arg (Printf.sprintf "Warning: code unit in airframe (%s) and setting file (%s) are not matching for param %s\n" uc us param) (* raise Invalid_argument *)
              | Some u, None | None, Some u -> u
              | None, None -> ""
          and unit_airframe =
            match airframe_unit with
              | Some u -> u
              | None -> ""
          in
          (* Printf.fprintf stderr "param %s: unit_code=%s unit_airframe=%s\n" param unit_code unit_airframe; flush stderr; *)
          PprzLink.scale_of_units unit_airframe unit_code
        with
          | Invalid_argument s -> prerr_endline s; flush stderr; raise Exit
          |  _ -> 1.
      in
      (*
       * settings are displayed in alt_unit specified in settings file
       * first try the alt_coef, otherwise try to convert the units
       *)
      let display_scale = float_of_string (PprzLink.alt_unit_coef_of_xml dl_setting) in
      let val_list = Str.split (Str.regexp "[ ()]+") airframe_value in
      let (scale_macros, str_val) = List.partition (fun x -> Str.string_match (Str.regexp "RadOfDeg\\|DegOfRad") x 0) val_list in
      let extra_scale =
        try
          match (List.hd scale_macros) with
              "RadOfDeg" -> Latlong.pi /. 180.
            | "DegOfRad" -> 180. /. Latlong.pi
            | _ -> 1.
        with
            _ -> 1. in
      let airframe_value_scaled =
        try
          float_of_string (List.hd str_val) *. airframe_scale *. extra_scale
        with
            Failure _ -> raise (EditAirframe.No_param param)
      in
      let airframe_value_new = value /. airframe_scale in
      (* test if is has to be saved as integer or float *)
      let integer = try ignore(int_of_string (attrib "step")); true with _ -> false in
      (* Printf.fprintf stderr "param %s: airframe_scale=%f display_scale=%f extra_scale=%f\n" param airframe_scale display_scale extra_scale; flush stderr; *)
      let row = model#append () in
      model#set ~row ~column:col_index index;
      model#set ~row ~column:col_param param;
      model#set ~row ~column:col_airframe_value (string_of_float (airframe_value_scaled *. display_scale));
      if integer then begin
        model#set ~row ~column:col_settings_value (string_of_int (truncate (floor (value *. display_scale +. 0.5))));
        model#set ~row ~column:col_airframe_value_new (string_of_int (truncate (floor (airframe_value_new +. 0.5))))
      end
      else begin
        model#set ~row ~column:col_settings_value (string_of_float (value *. display_scale));
        model#set ~row ~column:col_airframe_value_new (string_of_float airframe_value_new)
      end;
      model#set ~row ~column:col_code_value value;
      model#set ~row ~column:col_to_save (floats_not_equal airframe_value_scaled value);
      model#set ~row ~column:col_integer integer
    with
        Xml.No_attribute _ | Exit -> ()
      | EditAirframe.No_param param ->
        not_in_airframe_file := param :: !not_in_airframe_file ) (* Not savable *)
    settings;

  (* Warning if needed *)
  if !not_in_airframe_file <> [] then begin
    GToolbox.message_box ~title:"Warning" (Printf.sprintf "Some parameters not writable in the airframe file:\n\n%s" (String.concat "\n" !not_in_airframe_file));
  end




(** The popup window displaying airframe and settings values *)
let popup = fun airframe_filename settings send_value ->
  (* Build the list window *)
  let file = Env.paparazzi_src // "sw" // "ground_segment" // "cockpit" // "gcs.glade" in
  let w = new Gtk_save_settings.save_settings ~file () in
  let icon = GdkPixbuf.from_file Env.icon_file in
  w#save_settings#set_icon (Some icon);

  (* Build the tree model *)
  let model = GTree.tree_store cols in

  (** Attach the model to the view *)
  w#treeview_settings#set_model (Some model#coerce);

  (** Render the columns *)
  display_columns w model;

  (* Parse the airframe file *)
  let airframe_xml = XmlCom.parse_file airframe_filename in

  (** Insert the row data in the tree *)
  fill_data model settings airframe_xml;

  (** The Cancel button *)
  ignore (w#button_cancel#connect#clicked (fun () -> w#save_settings#destroy ()));

  (** Connect the Save button to the write action *)
  ignore (w#button_upload#connect#clicked (fun ()-> send_airframe_values model send_value));

  (** Connect the Save button to the write action *)
  ignore (w#button_save#connect#clicked (fun () -> save_airframe w airframe_filename (write_xml model airframe_filename airframe_xml)))
