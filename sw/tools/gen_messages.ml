(*
 * $Id$
 *
 * XML preprocessing of messages.xml for downlink protocol
 *  
 * Copyright (C) 2003-2005 Pascal Brisset, Antoine Drouin
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


module Syntax = struct
  type format = string

  type type_name = string

  type _type = 
      Basic of string
    | Array of string * string

  type field = _type  * string * format option

  type fields = field list

  type message = { name : string ; id : int; period : float option; fields : fields }

  type messages = message list

  let parse_type = fun t varname ->
    let n = String.length t in
    if n >=2 && String.sub t (n-2) 2 = "[]" then
      Array (String.sub t 0 (n-2), varname)
    else
      Basic t

  let length_name = fun s -> "nb_"^s

  let assoc_types t =
    try
      List.assoc t Pprz.types
    with
      Not_found -> fprintf stderr "Error: '%s' unknown type\n" t; exit 1

  let rec sizeof = function
      Basic t -> string_of_int (assoc_types t).Pprz.size
    | Array (t, varname) -> sprintf "%s*%s" (length_name varname) (sizeof (Basic t))
  let formatof = fun t -> (assoc_types t).Pprz.format

  let print_format t = function
      None -> printf "(%s)" (formatof t)
    | Some f -> printf "(%s)" f

  open Xml

  let xml_error s = failwith ("Bad XML tag: "^s^ " expected")

  let fprint_fields = fun f l ->
    fprintf f "<";
    List.iter (fun (a, b) -> fprintf f "%s=\"%s\" " a b) l;
    fprintf f ">"
      

  let assoc_or_fail x l =
    let x = String.uppercase x
    and l = List.map (fun (a, v) -> String.uppercase a, v) l in
    try
      List.assoc x l
    with
      Not_found ->
	fprintf stderr "Error: Field '%s' expected in <%a>" x fprint_fields l;
	exit 1

  let of_xml = function
      Element ("message", fields, l) ->
	let name = assoc_or_fail "name" fields
	and id = int_of_string (assoc_or_fail "id" fields) in
	{ id=id; name = name;
	  period = (try Some (float_of_string (List.assoc "period" fields)) with Not_found -> None);
	  fields=List.map (function
	      Element ("field", fields, []) ->
		let id = assoc_or_fail "name" fields
		and type_name = assoc_or_fail "type" fields
		and fmt = try Some (List.assoc "format" fields) with _ -> None in
		let _type = parse_type type_name id in
		
		(_type, id, fmt)
	    | _ -> xml_error "field")
	    l}
    | _ -> xml_error "message with id"
	  

  let read filename class_ = 
    let xml =
      try Xml.parse_file filename with
	Xml.Error (msg, pos) -> fprintf stderr "%s:%d : %s\n" filename (Xml.line pos) (Xml.error_msg msg); exit 1
    in
    match List.filter (fun x -> assert(Xml.tag x="class"); Xml.attrib x "name" = class_) (Xml.children xml) with
      [xml_class] -> List.map of_xml (Xml.children xml_class)
    | [] -> failwith (sprintf "No class '%s' found" class_)
    | _ -> failwith (sprintf "Several class '%s' found" class_)
end

module Gen_onboard = struct
  open Printf
  open Syntax

  let print_avr_field = fun avr_h (t, name, (_f:format option)) ->
    match t with 
      Basic _ ->
	fprintf avr_h "\t  DownlinkPut%sByteByAddr((const uint8_t*)(%s)); \\\n" (sizeof t) name
    | Array (t, varname) ->
	let s = sizeof (Basic t) in
	fprintf avr_h "\t  DownlinkPut1ByteUpdateCs(%s);\\\n" (length_name varname);
	fprintf avr_h "\t  {\\\n\t    int i;\\\n\t    for(i = 0; i < %s; i++) {\\\n" (length_name varname);
	fprintf avr_h "\t      DownlinkPut%sByteByAddr((uint8_t*)(&%s[i])); \\\n" s name;
	fprintf avr_h "\t    }\\\n";
	fprintf avr_h "\t  }\\\n"

  let print_one avr_h = function
      (Array _, s, _) -> fprintf avr_h "%s, %s" (length_name s) s
    | (_, s, _) -> fprintf avr_h "%s" s
  
  let print_avr_macro_names avr_h = function
      [] -> ()
    | f::fields ->
	print_one avr_h f;
	List.iter (fun f -> fprintf avr_h ", "; print_one avr_h f) fields

  let rec size_fields = fun fields size ->
    match fields with
      [] -> size
    | (t, _, _)::fields -> size_fields fields (size ^"+"^sizeof t)

  let size_of_message = fun m -> size_fields m.fields "0"
      
  let print_avr_macro = fun avr_h {name=s; fields = fields} ->
    fprintf avr_h "#define DOWNLINK_SEND_%s(" s;
    print_avr_macro_names avr_h fields;
    fprintf avr_h "){ \\\n";
    let size = (size_fields fields "0") in
    fprintf avr_h "\tif (DownlinkCheckFreeSpace(DownlinkSizeOf(%s))) {\\\n" size;
    fprintf avr_h "\t  DownlinkStartMessage(DL_%s,%s) \\\n" s size;
    List.iter (print_avr_field avr_h) fields;
    fprintf avr_h "\t  DownlinkEndMessage() \\\n";
    fprintf avr_h "\t} \\\n";
    fprintf avr_h "\telse \\\n";
    fprintf avr_h "\t  downlink_nb_ovrn++; \\\n";
    fprintf avr_h "}\n\n"

  let print_null_avr_macro = fun avr_h {name=s; fields = fields} ->
    fprintf avr_h "#define DOWNLINK_SEND_%s(" s;
    print_avr_macro_names avr_h fields;
    fprintf avr_h ") {}\n"

  let print_enum = fun avr_h messages ->
    List.iter (fun m -> fprintf avr_h "#define DL_%s %d\n" m.name m.id) messages;
    fprintf avr_h "#define DL_MSG_NB %d\n\n" (List.length messages)

  let print_lengths_array = fun avr_h messages ->
    let sizes = List.map (fun m -> (m.id, size_of_message m)) messages in
    let max_id = List.fold_right (fun (id, _m) x -> max x id) sizes min_int in
    let n = max_id + 1 in
    fprintf avr_h "#define MSG_LENGTHS {";
    for i = 0 to n - 1 do
      fprintf avr_h "%s," (try "(2+" ^ List.assoc i sizes^")" with Not_found -> "0")
    done;
    fprintf avr_h "}\n\n"

  let print_avr_macros = fun filename avr_h messages ->
    print_enum avr_h messages;
    print_lengths_array avr_h messages;
    List.iter (print_avr_macro avr_h) messages;
    let md5sum = Digest.file filename in
    fprintf avr_h "#define MESSAGES_MD5SUM \"";
    for i = 0 to String.length md5sum - 1 do
      fprintf avr_h "\\%03o" (Char.code md5sum.[i])
    done;
    fprintf avr_h "\"\n"
      
  let print_null_avr_macros = fun avr_h messages ->
    List.iter (print_null_avr_macro avr_h) messages

end

let _ =
  if Array.length Sys.argv <> 3 then begin
    failwith (sprintf "Usage: %s <.xml file> <class_name>" Sys.argv.(0)) 
  end;
  let filename = Sys.argv.(1) in
  let class_name = Sys.argv.(2) in

  let messages = Syntax.read filename class_name in

  let avr_h = stdout in
  Printf.fprintf avr_h "/* Automatically generated from %s */\n" filename;
  Printf.fprintf avr_h "/* Please DO NOT EDIT */\n";
  Printf.fprintf avr_h "#ifdef DOWNLINK\n";
  Gen_onboard.print_avr_macros filename avr_h messages;
  Printf.fprintf avr_h "#else // DOWNLINK\n";
  Gen_onboard.print_null_avr_macros avr_h messages;
  Printf.fprintf avr_h "#endif // DOWNLINK\n"
