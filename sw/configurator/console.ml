(*
 *  $Id$
 *
 * Console (text widget) handling
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

let console = ref None

let create = fun parent ->
  let c = Text.create parent in
  console := Some c;
  Text.tag_configure ~tag:"red" ~foreground:`Red c;
  Text.tag_configure ~tag:"blue" ~foreground:`Blue c;
  c

let write = fun ?(tags=[]) s ->
  match !console with
    None -> failwith "Console.write"
  | Some c ->
      Text.insert ~index:(`End,[]) ~tags ~text:s c;
      Text.see c (`End,[])  
  

let buffer_len = 256


let copy_from = 
  let buffer = String.create buffer_len in
  fun c kill color ->
    let n = Unix.read c buffer 0 buffer_len in
    if n = 0 then begin
      Fileevent.remove_fileinput c;
      kill ()
    end else
      write ~tags:[color] (String.sub buffer 0 n)

      
let exec = fun command ->
  write command; write "\n";
  let (proc_out, _, proc_err) as triple = Unix.open_process_full command [||] in
  let proc_out = Unix.descr_of_in_channel proc_out
  and proc_err = Unix.descr_of_in_channel proc_err in
  let kill =
    let twice = ref false in
    fun () ->
      if !twice then begin
	write
	  (match Unix.close_process_full triple with
	    Unix.WEXITED c -> Printf.sprintf "--------- terminated(%d)\n\n" c
	  | Unix.WSIGNALED c ->  Printf.sprintf "--------- killed(%d)\n\n" c
	  | Unix.WSTOPPED c ->   Printf.sprintf "--------- stopped(%d)\n\n" c)
      end else
	twice := true
  in
  Fileevent.add_fileinput proc_out (fun () -> copy_from proc_out kill "blue");
  Fileevent.add_fileinput proc_err (fun () -> copy_from proc_err kill "red")
