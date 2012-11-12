(*
 * Log files utilities
 *
 * Copyright (C) 2007 ENAC
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

let logs_dir = Env.paparazzi_home // "var" // "logs"

let log_suffix = fun s ->
  let n = String.length s in
  n > 4 && String.sub s (n-4) 4 = ".log"

let length_data = fun data_file ->
  let f = Ocaml_tools.open_compress data_file in
  let line = ref "" in
  try
    while true do line := input_line f done; failwith "unreachable"
  with
    End_of_file ->
      let i = String.index !line '.' in
      int_of_string (String.sub !line 0 i)

let chooser = fun ~callback () ->
  let dialog = GWindow.file_chooser_dialog ~action:`OPEN ~title:"Open Log" () in
  ignore (dialog#set_current_folder logs_dir);
  dialog#add_filter (GFile.filter ~name:"log" ~patterns:["*.log"] ());
  dialog#add_button_stock `CANCEL `CANCEL ;
  dialog#add_select_button_stock `OPEN `OPEN ;
  let info_label = GMisc.label ~text:"1234" () in
  dialog#set_extra_widget info_label#coerce;

  let preview_cb = fun () ->
    match dialog#preview_filename with
    | Some log_file when log_suffix log_file ->
	let basename = Filename.chop_extension (Filename.basename log_file) in
	let data_file = basename ^ ".data" in
	let info =
	  try
	    let data_file = Ocaml_tools.find_file [Filename.dirname log_file] data_file in
	    let size = (Unix.stat data_file).Unix.st_size / 1000 in
	    let time = length_data data_file in
	    sprintf "Data Size: %dk, Time Length: %ds" size time
	  with
	    Not_found -> "No associated data file found" in
	info_label#set_text (sprintf "Log %s: %s" basename info)
    | _ -> info_label#set_text "" in

  ignore(dialog#connect#update_preview ~callback:preview_cb);

  begin match dialog#run (), dialog#filename with
    `OPEN, Some name ->
      dialog#destroy ();
      callback name
  | _ -> dialog#destroy ()
  end
