(*
 * $Id$
 *
 * Paparazzi center utilities
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

let (//) = Filename.concat
let conf_dir = Env.paparazzi_home // "conf"

(** From OCaml otherlibs/unix/unix.ml *)
let my_open_process_in = fun cmd ->
  let (in_read, in_write) = Unix.pipe () in
  let inchan = Unix.in_channel_of_descr in_read in
  let pid = Unix.create_process "/bin/sh" [|"/bin/sh"; "-c"; cmd|] Unix.stdin in_write Unix.stderr in
  Unix.close in_write;
  pid, inchan


let run_and_log = fun log com ->
  let com = com ^ " 2>&1" in
  let pid, com_stdout = my_open_process_in com in
  let channel_out = GMain.Io.channel_of_descr (Unix.descr_of_in_channel com_stdout) in
  let cb = fun ev -> if List.mem `IN ev then begin log (input_line com_stdout); true end else begin log (sprintf "\nDONE (%s)\n" com); false end in
  let _io_watch_out = Glib.Io.add_watch [`IN; `HUP] cb channel_out in
  pid, channel_out

let command = fun (log:string->unit) ac_name target ->
  let com = sprintf "export PATH=/usr/bin:$PATH; make -C %s -f Makefile.ac AIRCRAFT=%s %s" Env.paparazzi_src ac_name target in
  log com;
  ignore (run_and_log log com)

type combo = GEdit.combo_box * (GTree.list_store * string GTree.column)
let combo_widget = fst
let combo_model = snd


let combo = fun ?(others = []) strings vbox ->
  let strings = others @ strings in
  let (combo, (tree, column)) =
    GEdit.combo_box_text ~packing:vbox#add ~strings () in
  combo#set_active 0;
  (combo, (tree, column))


let conf_is_set = fun home ->
  Sys.file_exists home &&
  Sys.file_exists (home // "conf") &&
  Sys.file_exists (home // "data")

let druid = fun home ->
  let w = GWindow.window ~title:"Configuring Paparazzi" () in

  let  d = GnoDruid.druid ~packing:w#add () in

  ignore (d#connect#cancel (fun () -> exit 1));

  begin
    let fp = GnoDruid.druid_page_edge ~position:`START ~aa:true ~title:"Configure Paparazzi !!" () in
    fp#set_text (sprintf "Configuration files need to be installed in your Paparazzi home (%s). To use another directory, please exit this utility, set the PAPARAZZI_HOME variable to the desired folder and restart." home);
    d#append_page fp;
    ignore (fp#connect#next
	      (fun _ ->	
		command prerr_endline "" "init";
		false
	      ))
    
  end;

  begin
    let ep = GnoDruid.druid_page_edge ~position:`FINISH ~aa:true ~title:"The end" () in
    ep#set_text "You are ready. Congratulations!" ;
    d#append_page ep ;

    ignore (ep#connect#finish
	      (fun _ ->	
		w#destroy ();
		GMain.quit ()
	      ))
  end;
  w#show ();
  GMain.main ()

let _ = 
  let home = Env.paparazzi_home in
  if not (conf_is_set home) then
    druid home

let conf_xml_file = conf_dir // "conf.xml"
let conf_xml = Xml.parse_file conf_xml_file
let aircrafts = Hashtbl.create 7
let aircrafts_table_has_changed = ref false
let _ =
  List.iter (fun aircraft ->
    Hashtbl.add aircrafts (ExtXml.attrib aircraft "name") aircraft)
    (Xml.children conf_xml);
  aircrafts_table_has_changed := true
