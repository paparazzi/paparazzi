(*
 * $Id$
 *
 * XML preprocessing for UBX protocol
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
    "U4" | "I4" -> 4
  | "U2" | "I2" -> 2
  | "U1" | "I1" -> 1
  | x -> failwith (sprintf "sizeof: unknown format '%s'" x)

let (+=) = fun r x -> r := !r + x

let get_at = fun offset format block_size ->
  let t = 
    match format with
      "I2" -> "int16_t"
    | "I4" -> "int32_t"
    | "U2" -> "uint16_t"
    | "U4" -> "uint32_t"
    | "U1" -> "uint8_t"
    | "I1" -> "int8_t"
    | _ -> failwith (sprintf "get_at: unknown format '%s'" format) in
  let block_offset =
    if block_size = 0 then "" else sprintf "+%d*_ubx_block" block_size in
  sprintf "(*((%s*)(_ubx_payload+%d%s)))" t offset block_offset

let define = fun x y ->
  fprintf out "#define %s %s\n" x y

exception Length_error of Xml.xml*int*int
  

let parse_class = fun c ->
  let class_id = int_of_string (Xml.attrib c "id")
  and class_name = Xml.attrib c "name" in

  fprintf out "\n";
  define (sprintf "UBX_%s_ID" class_name) (Xml.attrib c "ID");

  let parse_message = fun m ->
    let msg_name = Xml.attrib m "name" in

    fprintf out "\n";
    define (sprintf "UBX_%s_%s_ID" class_name msg_name) (Xml.attrib m "ID");

    let offset = ref 0 in
    let rec parse_field = fun block_size f ->
      match Xml.tag f with
	"field" ->
	  let field_name = Xml.attrib f "name"
	  and format = Xml.attrib f "format" in
	  let block_no = if block_size = 0 then "" else ",_ubx_block" in
	  define (sprintf "UBX_%s_%s_%s(_ubx_payload%s)" class_name msg_name field_name block_no) (get_at !offset format block_size);
	  offset += sizeof format
      | "block" ->
	  let s = int_of_string (Xml.attrib f "length") in
	  let o = !offset in
	  List.iter (parse_field s) (Xml.children f);
	  let s' = !offset - o in
	  if s <> s' then raise (Length_error (f, s, s'))
      | x -> failwith ("Unexpected field: " ^ x)
    in

    List.iter (parse_field 0) (Xml.children m);
    try
      let l = int_of_string (Xml.attrib m "length") in
      if l <> !offset then raise (Length_error (m, l, !offset))
    with
      Xml.No_attribute("length") -> ()
  in


  List.iter parse_message (Xml.children c)
  

let _ =
  if Array.length Sys.argv <> 2 then begin
    failwith (sprintf "Usage: %s <.xml ubx protocol file>" Sys.argv.(0)) 
  end;
  let xml_file = Sys.argv.(1) in
  try
    let xml = Xml.parse_file xml_file in
    fprintf out "/* Generated from %s */\n" xml_file;
    fprintf out "/* Please DO NOT EDIT */\n\n";
    
    define "UBX_SYNC1" "0xB5";
    define "UBX_SYNC2" "0x62";
    
    List.iter parse_class (Xml.children xml)
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
