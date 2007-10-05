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

let buf_size = 128

let run_and_log = fun log com ->
  let com = com ^ " 2>&1" in
  let pid, com_stdout = my_open_process_in com in
  let channel_out = GMain.Io.channel_of_descr (Unix.descr_of_in_channel com_stdout) in
  let cb = fun ev ->
    if List.mem `IN ev then begin
      let buf = String.create buf_size in
      let rec loop = fun () ->
	let n = input com_stdout buf 0 buf_size in
	if n < buf_size then
	  log (String.sub buf 0 n)
	else begin
	  log buf;
	  loop ()
	end in
      loop ();
      true
    end else begin 
      log (sprintf "\nDONE (%s)\n\n" com);
      false
    end in
  let io_watch_out = Glib.Io.add_watch [`IN; `HUP] cb channel_out in
  pid, channel_out, com_stdout, io_watch_out

type combo = GEdit.combo_box * (GTree.list_store * string GTree.column)
let combo_widget = fst
let combo_model = snd

let combo_value = fun ((combo: #GEdit.combo_box), (_,column)) ->
  match combo#active_iter with
  | None -> raise Not_found
  | Some row -> combo#model#get ~row ~column

let combo_separator = "--"
      
let combo = fun strings vbox ->
  let (combo, (tree, column)) =
    GEdit.combo_box_text ~packing:vbox#add ~strings () in
  combo#set_active 0;
  combo#set_row_separator_func
    (Some (fun m row -> m#get ~row ~column = combo_separator)) ;
  (combo, (tree, column))

let add_to_combo = fun (combo : combo) string ->
  let (store, column) = combo_model combo in
  let row = store#append () in
  store#set ~row ~column string;
  (combo_widget combo)#set_active_iter (Some row)


let select_in_combo = fun  (combo : combo) string ->
  let (store, column) = combo_model combo in
  store#foreach 
    (fun _path row ->
      if store#get ~row ~column = string then begin
	(combo_widget combo)#set_active_iter (Some row);
	true
      end else
	false)

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




let run_and_monitor = fun ?(once = false) ?file gui log com_name com args ->
  let c = sprintf "%s %s" com args in
  let p = new Gtk_process.hbox_program ?file () in
  (gui#vbox_programs:GPack.box)#pack p#toplevel#coerce;
  p#label_com_name#set_text com_name;
  p#entry_program#set_text c;
  let pid = ref (-1)
  and outchan = ref stdin
  and watches = ref [] in
  let run = fun callback ->
    let c = p#entry_program#text in
    log (sprintf "Run '%s'\n" c);
    let (pi, out, unixfd, io_watch) = run_and_log log ("exec "^c) in
    pid := pi;
    outchan := unixfd;
    let io_watch' = Glib.Io.add_watch [`HUP] (fun _ -> callback true;false) out in
    watches := [ io_watch; io_watch'] in

  let remove_callback = fun () ->
    gui#vbox_programs#remove p#toplevel#coerce in

  let rec callback = fun stop ->
    match p#button_stop#label, stop with
      "gtk-stop", _ ->
	List.iter Glib.Io.remove !watches;
	close_in !outchan;
	ignore (Unix.kill !pid Sys.sigkill);
	ignore (Unix.waitpid [] !pid);
	p#button_stop#set_label "gtk-redo";
	p#button_remove#misc#set_sensitive true;
	if once then
	  remove_callback ()
	else if stop && p#checkbutton_autolaunch#active then
	  callback false
    | "gtk-redo", false ->
	p#button_stop#set_label "gtk-stop";
	run callback;
	p#button_remove#misc#set_sensitive false
    | _ -> ()
  in
  ignore (p#button_stop#connect#clicked ~callback:(fun () -> callback false));
  run callback;
  
  (* Stop the program if the box is closed *)
  let callback = fun () ->
    callback true in
  ignore(p#toplevel#connect#destroy ~callback);

  (* Remove button *)
  ignore (p#button_remove#connect#clicked ~callback:remove_callback)


let basic_command = fun (log:string->unit) ac_name target ->
  let com = sprintf "export PATH=/usr/bin:$PATH; make -C %s -f Makefile.ac AIRCRAFT=%s %s" Env.paparazzi_src ac_name target in
  log com;
  ignore (run_and_log log com)


let command = fun ?file gui (log:string->unit) ac_name target ->
  let com = sprintf "make -C %s -f Makefile.ac AIRCRAFT=%s %s" Env.paparazzi_src ac_name target in
  run_and_monitor ~once:true ?file gui log "make" com ""


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
		basic_command prerr_endline "" "init";
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
let backup_xml_file = conf_xml_file ^ "~"
let aircrafts = Hashtbl.create 7
let aircrafts_table_has_changed = ref false
let build_aircrafts = fun () ->
  let conf_xml = Xml.parse_file conf_xml_file in
  List.iter (fun aircraft ->
    Hashtbl.add aircrafts (ExtXml.attrib aircraft "name") aircraft)
    (Xml.children conf_xml);
  aircrafts_table_has_changed := true



