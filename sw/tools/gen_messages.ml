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

(** FIXME: Should use the Pprz Module !!! *)

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
    | Array (t, varname) -> sprintf "1+%s*%s" (length_name varname) (sizeof (Basic t))
  let rec nameof = function
      Basic t -> String.capitalize t
    | Array (t, varname) -> failwith "nameof"

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
    try
      let xml_class =
	List.find (fun x -> Xml.attrib x "name" = class_) (Xml.children xml) in
      List.map of_xml (Xml.children xml_class)
    with
      Not_found -> failwith (sprintf "No class '%s' found" class_)
end


let check_alignment = ref true

module Gen_onboard = struct
  open Printf
  open Syntax

  let print_avr_field = fun avr_h (t, name, (_f:format option)) ->
    match t with 
      Basic _ ->
	fprintf avr_h "\t  DownlinkPut%sByAddr((%s)); \\\n" (nameof t) name
    | Array (t, varname) ->
	let _s = sizeof (Basic t) in
	fprintf avr_h "\t  DownlinkPut%sArray(%s, %s); \\\n" (nameof (Basic t)) (length_name varname) name

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
    fprintf avr_h "\t  DownlinkCountBytes(DownlinkSizeOf(%s)); \\\n" size;
    fprintf avr_h "\t  DownlinkStartMessage(\"%s\", DL_%s, %s) \\\n" s s size;
    List.iter (print_avr_field avr_h) fields;
    fprintf avr_h "\t  DownlinkEndMessage() \\\n";
    fprintf avr_h "\t} else \\\n";
    fprintf avr_h "\t  DownlinkOverrun(); \\\n";
    fprintf avr_h "}\n\n"

  let print_null_avr_macro = fun avr_h {name=s; fields = fields} ->
    fprintf avr_h "#define DOWNLINK_SEND_%s(" s;
    print_avr_macro_names avr_h fields;
    fprintf avr_h ") {}\n"

  let print_enum = fun avr_h class_ messages ->
    List.iter (fun m -> fprintf avr_h "#define DL_%s %d\n" m.name m.id) messages;
    fprintf avr_h "#define DL_MSG_%s_NB %d\n\n" class_ (List.length messages)

  let print_lengths_array = fun avr_h class_ messages ->
    let sizes = List.map (fun m -> (m.id, size_of_message m)) messages in
    let max_id = List.fold_right (fun (id, _m) x -> max x id) sizes min_int in
    let n = max_id + 1 in
    fprintf avr_h "#define MSG_%s_LENGTHS {" class_;
    for i = 0 to n - 1 do
      fprintf avr_h "%s," (try "(2+" ^ List.assoc i sizes^")" with Not_found -> "0")
    done;
    fprintf avr_h "}\n\n"

  let print_avr_macros = fun filename avr_h class_ messages ->
    print_enum avr_h class_ messages;
    print_lengths_array avr_h class_ messages;
    List.iter (print_avr_macro avr_h) messages;
    let md5sum = Digest.file filename in
    fprintf avr_h "#define MESSAGES_MD5SUM \"";
    for i = 0 to String.length md5sum - 1 do
      fprintf avr_h "\\%03o" (Char.code md5sum.[i])
    done;
    fprintf avr_h "\"\n"
      
  let print_null_avr_macros = fun avr_h messages ->
    List.iter (print_null_avr_macro avr_h) messages

  let print_get_macros = fun avr_h message ->
    let msg_name = message.name in
    let offset = ref Pprz.offset_fields in
    
    let parse_field = fun (_type, field_name, _format) ->
      if !offset < 0 then
	failwith "FIXME: No field allowed after an array field (print_gen_macro)x";
      let typed = fun o t ->
	let size = (assoc_types t).Pprz.size in
	if !check_alignment && o mod size <> 0 then
	  failwith (sprintf "Wrong alignment of field '%s' in message '%s" field_name msg_name);
	match size with
	  1 -> sprintf "(%s)(*((uint8_t*)_payload+%d))" (assoc_types t).Pprz.inttype o
	| 2 -> sprintf "(%s)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8)" (assoc_types t).Pprz.inttype o o
	| 4 when (assoc_types t).Pprz.inttype = "float" -> 
	    sprintf "({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8|*((uint8_t*)_payload+%d+2)<<16|*((uint8_t*)_payload+%d+3)<<24); _f.f; })" o o o o
	| 4 -> 
	    sprintf "(%s)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8|*((uint8_t*)_payload+%d+2)<<16|*((uint8_t*)_payload+%d+3)<<24)" (assoc_types t).Pprz.inttype o o o o
	| _ -> failwith "unexpected size in Gen_messages.print_get_macros" in
      match _type with 
	Basic t ->
	  fprintf avr_h "#define DL_%s_%s(_payload) (%s)\n" msg_name field_name (typed !offset t);
	  offset := !offset + int_of_string (sizeof _type)
      | Array (t, varname) ->
	  fprintf avr_h "#define DL_%s_%s_length(_payload) (*%s)\n" msg_name field_name (typed !offset "uint8");
	  incr offset;
	  fprintf avr_h "#define DL_%s_%s(_payload) %s\n" msg_name field_name (typed !offset t);
	  offset := -1 (** Mark for no more fields *)
    in
    
    fprintf avr_h "\n";
    List.iter parse_field message.fields

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

  Printf.fprintf avr_h "/* Macros to send and receive messages of class %s */\n" class_name;

  (** Macros for airborne downlink (sending) *)
  if class_name = "telemetry" then begin (** FIXME *)
    Printf.fprintf avr_h "#ifdef DOWNLINK\n";
    check_alignment := false
  end;
  Gen_onboard.print_avr_macros filename avr_h class_name messages;
  if class_name = "telemetry" then begin
    Printf.fprintf avr_h "#else // DOWNLINK\n";
    Gen_onboard.print_null_avr_macros avr_h messages;
    Printf.fprintf avr_h "#endif // DOWNLINK\n"
  end;

  (** Macros for airborne datalink (receiving) *)
  List.iter (Gen_onboard.print_get_macros avr_h) messages

