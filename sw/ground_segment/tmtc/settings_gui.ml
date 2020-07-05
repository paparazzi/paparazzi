(*
 * Multi aircraft settings handler
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

module Ground_Pprz = PprzLink.Messages(struct let name = "ground" end)
module Tele_Pprz = PprzLink.Messages(struct let name = "telemetry" end)

let (//) = Filename.concat
let conf_dir = Env.paparazzi_home // "conf"
let conf_xml = ExtXml.parse_file (conf_dir // "conf.xml")



let one_ac = fun (notebook:GPack.notebook) ac_name ->
  (* Get the setting file *)
  let xml_file = Env.paparazzi_home // "var" // "aircrafts" // ac_name // "settings.xml" in
  if not (Sys.file_exists xml_file) then
    Printf.fprintf stderr "A/C '%s' not compiled: %s not found\n%!" ac_name xml_file
  else
    (* Get the A/C id *)
    let aircraft = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = ac_name) conf_xml "aircraft" in
    let ac_id = Xml.attrib aircraft "ac_id" in

    (* Call to ivy to set a value *)
    let callback = fun idx value ->

      let vs = ["ac_id", PprzLink.String ac_id; "index", PprzLink.Int idx] in
      if classify_float value = FP_normal || classify_float value =FP_zero then
        let vs' = ("value", PprzLink.Float value) :: vs in
        Ground_Pprz.message_send "dl" "DL_SETTING" vs'
      else
        Ground_Pprz.message_send "dl" "GET_DL_SETTING" vs in

    (* Build the buttons and sliders *)
    let xml = ExtXml.parse_file xml_file in
    let xmls = Xml.children (ExtXml.child xml "dl_settings") in
    let settings = new Page_settings.settings xmls callback ac_id Env.gcs_default_icons_theme (fun _ _ -> ()) in

    (* Bind to values updates *)
    let get_dl_value = fun _sender vs ->
      settings#set (PprzLink.int_assoc "index" vs) (Some (string_of_float (PprzLink.float_assoc "value" vs)))
    in
    ignore (Tele_Pprz.message_bind "DL_VALUE" get_dl_value);

    (* Get the aiframe file *)
    let af = Xml.attrib aircraft "airframe" in
    let af_file = conf_dir // af in

    (* Show the page *)
    let tab_label = GPack.hbox () in
    let _label = GMisc.label ~text:ac_name ~packing:tab_label#pack () in
    let button_save_settings = GButton.button ~packing:tab_label#pack () in
    ignore (GMisc.image ~stock:`SAVE ~packing:button_save_settings#add ());
    ignore (button_save_settings#connect#clicked (fun () -> settings#save af_file));
    ignore (notebook#append_page ~tab_label:tab_label#coerce settings#widget)


let _ =

  let ivy_bus = ref Defivybus.default_ivy_bus in
  let acs = ref [] in
  let geometry = ref "" in

  let anon_fun = (fun x -> prerr_endline ("WARNING: don't do anything with "^x)) in
  let speclist =
    [ "-b", Arg.String (fun x -> ivy_bus := x), (sprintf "<ivy bus> Default is %s" !ivy_bus);
      "-ac",  Arg.String (fun x -> acs := x :: !acs), "A/C name";
      "-g", Arg.String (fun x -> geometry := x), "<geometry>  Set the window geometry ( '500x500+100+100' )"]
  and usage_msg = "Usage: " in

  Arg.parse speclist anon_fun usage_msg;

  if List.length !acs = 0 then begin
    prerr_endline "Error: Specify at least one Aircraft for which to display the settings!";
    Arg.usage speclist usage_msg;
    exit 1
  end;

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi settings" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (** Open the window container with its notebook*)
  let icon = GdkPixbuf.from_file Env.icon_file in
  let window = GWindow.window ~icon ~width:400 ~height:300 ~allow_shrink:true ~title:"PaSettings" () in
  ignore (window#parse_geometry !geometry);
  let notebook = GPack.notebook ~packing:window#add ~tab_pos:`TOP () in

  List.iter (one_ac notebook) !acs;

  (** Start the main loop *)
  window#show ();
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done

