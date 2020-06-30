(*
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
  let pid = Unix.create_process_env "/bin/sh" [|"/bin/sh"; "-c"; cmd|] (Array.append (Unix.environment ()) [|"GTK_SETLOCALE=0";"LANG=C"|]) in_read in_write Unix.stderr in
  Unix.close in_write;
  pid, inchan

let buf_size = 512

let run_and_log = fun log exit_cb com ->
  let com = com ^ " 2>&1" in
  let pid, com_stdout = my_open_process_in com in
  let channel_out_fd = Unix.descr_of_in_channel com_stdout in
  let channel_out = GMain.Io.channel_of_descr channel_out_fd in
  let cb = fun _ ->
      let buf = Bytes.create buf_size in
      (* loop until input returns zero *)
      let rec log_input = fun out ->
        let n = input out buf 0 buf_size in
        (* split on beginning of new line *)
        let s = Str.split (Str.regexp "^") (Bytes.to_string (Bytes.sub buf 0 n)) in
        List.iter (fun l -> log l) s;
        if n = buf_size then (log_input out) + n else n
      in
      let count = log_input com_stdout in
      if count = 0 then exit_cb true;
      true
    in
  let io_watch_out = Glib.Io.add_watch [`IN] cb channel_out in
  pid, channel_out, com_stdout, io_watch_out

let strip_prefix = fun dir file subdir ->
  let n = String.length dir in
  if not (String.length file > n && String.sub file 0 n = dir) then begin
    let home = Env.paparazzi_home in
    let nn = String.length home in
    if (String.length file > nn && String.sub file 0 nn = home) then begin
      ".." // String.sub file (nn+1) (String.length file - nn -1)
    end else
     let msg = sprintf "Selected file '%s' should be in '%s'" file dir in
     GToolbox.message_box ~title:"Error" msg;
     raise Exit
  end else
    subdir // String.sub file (n+1) (String.length file - n - 1)


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
      cb (List.map (fun f -> strip_prefix dir f subdir) names)
    | `OPEN, Some name ->
      dialog#destroy ();
      cb [strip_prefix dir name subdir]
    | _ -> dialog#destroy ()
  end



let run_and_monitor = fun ?(once = false) ?file ?(finished_callback = fun () -> ()) gui log com_name com args ->
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
    log (sprintf "RUN '%s'\n" c);

  let (pi, out, unixfd, io_watch) = run_and_log log callback ("exec "^c) in
    pid := pi;
    outchan := unixfd;
    (* watch for hangup/end on the out io, after small delay call callback to stop/remove prog *)
    let io_watch' = Glib.Io.add_watch ~cond:[`HUP] ~callback:
      (fun _ ->
         (* call with a delay of 200ms, not strictly needed anymore, but seems more pleasing to the eye *)
         ignore (Glib.Timeout.add 200 (fun () -> callback true; false));
        (* return true to not automatically remove event source,
           otherwise will try to remove non existent source in callback, resulting in:
           GLib-CRITICAL **: Source ID xxx was not found when attempting to remove it *)
         true) out in
    watches := [ io_watch; io_watch' ] in

  let remove_callback = fun () ->
    gui#vbox_programs#remove p#toplevel#coerce in

  let rec callback = fun stop ->
    match p#button_stop#label, stop with
        "gtk-stop", _ ->
          List.iter Glib.Io.remove !watches;
          close_in !outchan;
          ignore (Unix.kill !pid Sys.sigkill);
          begin match Unix.waitpid [] !pid with
            | (x, Unix.WEXITED 0) ->
              log (sprintf "\nDONE '%s'\n\n" com);
            | (x, Unix.WEXITED i) ->
              log (sprintf "\nFAILED '%s' with code %i\n\n" com i);
            | (x, _) ->
              log (sprintf "\nSTOPPED '%s'\n\n" com);
          end;
          finished_callback ();
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
  ignore (p#entry_program#connect#activate ~callback:(fun () -> callback false));
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
  ignore (run_and_log log (fun _ -> ()) com)


let command = fun ?file ?finished_callback gui (log:string->unit) ac_name target ->
  let com = sprintf "make -C %s -f Makefile.ac AIRCRAFT=%s %s" Env.paparazzi_src ac_name target in
  run_and_monitor ~once:true ?file ?finished_callback gui log "make" com ""


let conf_is_set = fun home ->
  Sys.file_exists home &&
    Sys.file_exists (home // "conf") &&
    Sys.file_exists (home // "data")

(* This was the place where GnoDruid used to create a wizard configuring your
 * paparazzi installation. This could be replaced with an implementation using
 * GtkAssistant instead. The issue tracking this can be found at:
 * https://github.com/paparazzi/paparazzi/issues/923
 *)

let _ =
  let home = Env.paparazzi_home in
  if not (conf_is_set home) then
        printf "ERROR: Configuration files need to be installed in your \
        Paparazzi home (%s). Run `make init` in the toplevel paparazzi \
        directory to do that in your Paparazzi home (%s) directory. To \
        use another directory, set the PAPARAZZI_HOME variable to the \
        desired folder.\n" home home

let conf_xml_file = conf_dir // "conf.xml"
let backup_xml_file = conf_xml_file ^ "~"
let aircrafts = Hashtbl.create 7
let build_aircrafts = fun () ->
  let conf_xml = ExtXml.parse_file conf_xml_file in
  List.iter (fun aircraft ->
    Hashtbl.add aircrafts (ExtXml.attrib aircraft "name") aircraft)
    (Xml.children conf_xml)
