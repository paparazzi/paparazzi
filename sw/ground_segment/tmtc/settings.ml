(*
 * $Id$
 *
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

module Ground_Pprz = Pprz.Messages(struct let name = "ground" end)
module Tele_Pprz = Pprz.Messages(struct let name = "telemetry" end)

let (//) = Filename.concat
let conf_dir = Env.paparazzi_home // "conf"
let conf_xml = Xml.parse_file (conf_dir // "conf.xml")



let one_ac = fun (notebook:GPack.notebook) ac_name ->
  (* Get the setting file *)
  let xml_file = Env.paparazzi_home // "var" // ac_name // "settings.xml" in
  if not (Sys.file_exists xml_file) then
    Printf.fprintf stderr "A/C '%s' not compiled: %s not found\n%!" ac_name xml_file
  else
    (* Get the A/C id *)
    let aircraft = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = ac_name) conf_xml "aircraft" in
    let ac_id = Xml.attrib aircraft "ac_id" in

    (* Call to ivy to set a value *)
    let callback = fun idx value ->
      let vs = ["ac_id", Pprz.String ac_id; "index", Pprz.Int idx;"value", Pprz.Float value] in
      Ground_Pprz.message_send "dl" "DL_SETTING" vs in

    (* Build the buttons and sliders *)
    let xml = Xml.parse_file xml_file in
    let xmls = Xml.children (ExtXml.child xml "dl_settings") in
    let settings = new Pages.settings xmls callback (fun _ -> ()) in

    (* Bind to values updates *)
    let get_dl_value = fun _sender vs ->
      settings#set (Pprz.int_assoc "index" vs) (Pprz.float_assoc "value" vs)
      in
    ignore (Tele_Pprz.message_bind "DL_VALUE" get_dl_value);

    (* Show the page *)
    let label = GMisc.label ~text:ac_name () in
    notebook#append_page ~tab_label:label#coerce settings#widget


let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let acs = ref [] in
  Arg.parse
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.255:2010";
      "-ac",  Arg.String (fun x -> acs := x :: !acs), "A/C name"]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi messages" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (** Open the window container with its notebook*)
  let window = GWindow.window ~width:400 ~height:300 ~allow_shrink:true ~title:"Paparazzi settings" () in

  let notebook = GPack.notebook ~packing:window#add ~tab_pos:`TOP () in

  List.iter (one_ac notebook) !acs;
  
  (** Start the main loop *)
  window#show ();
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done

