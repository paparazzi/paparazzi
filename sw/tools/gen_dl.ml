(*
 * $Id$
 *
 * XML preprocessing for datalink protocol
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

let out = stdout

let sizeof = function
    "int32" | "uint32" | "float" -> 4
  | "int16" | "uint16" -> 2
  | "int8" | "uint8" -> 1
  | x -> failwith (sprintf "sizeof: unknown format '%s'" x)

let (+=) = fun r x -> r := !r + x

let get_at = fun offset format ->
  let t = 
    match format with
      "int16" -> "int16_t"
    | "int32" -> "int32_t"
    | "uint16" -> "uint16_t"
    | "uint32" -> "uint32_t"
    | "uint8" -> "uint8_t"
    | "int8" -> "int8_t"
    | "float" -> "float"
    | _ -> failwith (sprintf "get_at: unknown format '%s'" format) in
  sprintf "(*((%s*)(_ubx_payload+%d)))" t offset

let define = fun x y ->
  fprintf out "#define %s %s\n" x y

exception Length_error of Xml.xml*int*int
  

let parse_message = fun m ->
  let msg_name = Xml.attrib m "name" in
  
  fprintf out "\n";
  define (sprintf "DL_%s_ID" msg_name) (Xml.attrib m "ID");
  
  let offset = ref 1 in (** 1 for the tag of the message *)
  let rec parse_field = fun f ->
    match Xml.tag f with
      "field" ->
	let field_name = Xml.attrib f "name"
	and format = Xml.attrib f "type" in
	define (sprintf "DL_%s_%s(_ubx_payload)" msg_name field_name) (get_at !offset format);
	offset += sizeof format
    | x -> failwith ("Unexpected field: " ^ x)
    in

    List.iter parse_field (Xml.children m)
  

let _ =
  if Array.length Sys.argv <> 2 then begin
    failwith (sprintf "Usage: %s <.xml message file>" Sys.argv.(0)) 
  end;
  let xml_file = Sys.argv.(1) in
  try
    let xml = Xml.parse_file xml_file in
    let xml_class = ExtXml.child xml ~select:(fun m -> ExtXml.attrib m "name" = "datalink") "class" in
    fprintf out "/* Generated from %s */\n" xml_file;
    fprintf out "/* Please DO NOT EDIT */\n\n";
    
    List.iter parse_message (Xml.children xml_class)
  with
    Xml.Error (em, ep) ->
      let l = Xml.line ep
      and c1, c2 = Xml.range ep in
      fprintf stderr "File \"%s\", line %d, characters %d-%d:\n" xml_file l c1 c2;
      fprintf stderr "%s\n" (Xml.error_msg em);
      exit 1
  | Length_error (m, l1, l2) ->
      fprintf stderr "File \"%s\", inconsistent length: %d expected, %d found from fields in message:\n %s\n" xml_file l1 l2 (Xml.to_string_fmt m);
      exit 1
  | Dtd.Check_error e ->
      fprintf stderr "File \"%s\", DTD check error: %s\n" xml_file (Dtd.check_error e)
  | Dtd.Prove_error e ->
      fprintf stderr "\nFile \"%s\", DTD check error: %s\n\n" xml_file (Dtd.prove_error e)
