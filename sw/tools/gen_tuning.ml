(*
 * $Id$
 *
 * XML preprocessing for dynamic tuning
 *  
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

(** Generates code to define dl_setting in external program *)

open Printf
open Xml2h

let rec flatten = fun xml r ->
  if ExtXml.tag_is xml "dl_setting" then
    xml::r
  else
    match Xml.children xml with
      [] -> r
    | x::xs ->
	List.iter (fun y -> assert(ExtXml.tag_is y (Xml.tag x))) xs;
	List.fold_right flatten (x::xs) r


let _ =
  if Array.length Sys.argv <> 2 then
    failwith (Printf.sprintf "Usage: %s input_xml_file(s)" Sys.argv.(0));
  let h_name = "TUNING_H" in
  let xml_file = Sys.argv.(1) in
  
  try
    printf "/* This file has been generated from %s */\n" xml_file;
    printf "/* Please DO NOT EDIT */\n\n";
    
    printf "#ifndef %s\n" h_name;
    printf "#define %s\n\n" h_name;

    let xml = Xml.parse_file xml_file in
    let settings = flatten xml [] in
  
    (** Macro to define variables id *)
    let idx = ref 0 in
    List.iter (fun s ->
      let v = ExtXml.attrib s "var" in
      let up = String.uppercase v in
      begin printf "#define PPRZ_%s %d \n" up !idx end;
      incr idx) 
      settings;

    finish h_name
  with
    Xml.Error e -> prerr_endline (Xml.error e); exit 1
  | Dtd.Prove_error e ->  prerr_endline (Dtd.prove_error e); exit 1
  | Dtd.Parse_error e ->  prerr_endline (Dtd.parse_error e); exit 1
