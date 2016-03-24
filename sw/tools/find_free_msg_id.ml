(*
 * Call to Makefile.ac with the appropriate attributes from conf.xml
 *
 * Copyright (C) 2003-2010 ENAC
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

(******************************************************************************)
(* quick and dirty messages.xml free id tool                                  *)
(* Usage:                                                                     *)
(*       ./find_free_msg_id                                                   *)
(* print free id of PAPARZZI_HOME/conf/_messages.xml                          *)
(******************************************************************************)

(** FIXME: Get file names with Arg.parse *)

let (//) =  Filename.concat
let messages_xml = Env.paparazzi_home // "var" // "messages.xml"

let nb_msg = 255

(* parse the messages.xml file *)
let used_messages_id = fun xml ->
  let class_xmls = Xml.children xml in
  let id_of_message = fun xml ->
    int_of_string (Xml.attrib xml "id")
  in
  let rec find_message_ids = fun xml ids ->
    if Xml.tag xml = "message"
    then (id_of_message xml) ::ids
    else List.fold_right
      (fun c l -> find_message_ids c l) (Xml.children xml) ids
  in
  List.map
    (fun c -> ((Xml.attrib c "name"), find_message_ids c [])) class_xmls

(* useful to display grouped ids *)
let group = fun l ->
  let gl = ref [] in
  try
    let s = ref (List.hd l) in
    let n = List.length l in
    for i=1 to n-1 do
      let li = List.nth l i and
          li1= List.nth l (i-1) in
      if (li - li1 > 1)
      then begin
        gl :=
          (
            if !s = li1
            then Printf.sprintf "%d" !s
            else Printf.sprintf "%d-%d" !s li1) :: !gl;
        s := li;
      end
      else if i = n -1
      then
        gl := (Printf.sprintf "%d-%d" !s li) :: !gl;
    done;
    List.rev !gl
  with Not_found -> []

(* MAIN *)
let () =
  (* reading files *)
  let xml = ExtXml.parse_file messages_xml in
  let messages =
    List.map (fun c -> ((Xml.attrib c "name"), Xml.children c)) (Xml.children xml)
  in

  let id_list = Array.to_list (Array.init nb_msg (fun i -> i+1)) in

  (* finding free IDs *)
  let free_msg_id =
    let umi = used_messages_id xml in
    List.map (fun (c,_) ->
      if List.mem_assoc c umi
      then let used = List.assoc c umi in
           (c,List.filter (fun i -> not (List.mem i used)) id_list)
      else (c,id_list)
    ) messages in

  (* print free IDs *)
  List.iter (fun (c, l) ->
    Printf.printf "Class : %s \n" c;
      (*group l*)
    List.iter (fun id -> Printf.printf " %d," id) l;
    Printf.printf "\n\n"
  ) free_msg_id




