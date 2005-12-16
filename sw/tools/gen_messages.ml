(*
 * $Id$
 *
 * XML preprocessing for downlink protocol
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


module Syntax = struct
  type format = string

  type type_name = string

  type _type = Basic of string | Array of string * int

  type field = _type  * string * format option

  type fields = field list

  type message = { name : string ; id : int; period : float option; fields : fields }

  type messages = message list

  let lineno = ref 1 (* For syntax error messages *)  

  let assoc_types t =
    try
      List.assoc t Pprz.types
    with
      Not_found -> fprintf stderr "Error: '%s' unknown type\n" t; exit 1

  let rec sizeof = function
      Basic t -> (assoc_types t).Pprz.size
    | Array (t, i) -> i * sizeof (Basic t)
  let glibof = fun t -> (assoc_types t).Pprz.glib_type
  let formatof = fun t -> (assoc_types t).Pprz.format

  let print_format t = function
      None -> printf "(%s)" (formatof t)
    | Some f -> printf "(%s)" f

  let print_field = fun (t, s, f) ->
    begin
      match t with
	Basic t -> printf "%s %s " t s; print_format t f
      | Array(t, i) -> printf "%s %s[%d] " t s i; print_format t f
    end; printf "\n"

  let print_message = fun (s, fields) ->
    printf "%s {\n" s;
    List.iter print_field fields;
    printf "}\n"

  let print_messages = List.iter print_message

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
	      let _type = try Array(type_name, int_of_string (List.assoc "len" fields)) with Not_found -> Basic type_name in
		      
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
	fprintf avr_h "\t  MODEM_PUT_%d_BYTE_BY_ADDR((uint8_t*)(%s)); \\\n" (sizeof t) name
    | Array (t, i) ->
	let s = sizeof (Basic t) in
	fprintf avr_h "\t  {\\\n\t    int i;\\\n\t    for(i = 0; i < %d; i++) {\\\n" i;
	fprintf avr_h "\t      MODEM_PUT_%d_BYTE_BY_ADDR((uint8_t*)(&%s[i])); \\\n" s name;
	fprintf avr_h "\t    }\\\n";
	fprintf avr_h "\t  }\\\n"

  let print_avr_macro_names avr_h = function
      [] -> ()
    | (_, s, _)::fields ->
	fprintf avr_h "%s" s; List.iter (fun (_, s, _) -> fprintf avr_h ", %s" s) fields

  let rec size_fields = fun fields size ->
    match fields with
      [] -> size + 4
    | (t, _, _)::fields -> size_fields fields (size + sizeof(t))

  let size_of_message = fun m -> size_fields m.fields 0    
	  
  let print_avr_macro = fun avr_h {name=s; fields = fields} ->
    fprintf avr_h "#define DOWNLINK_SEND_%s(" s;
    print_avr_macro_names avr_h fields;
    fprintf avr_h "){ \\\n";
    fprintf avr_h "\tif (MODEM_CHECK_FREE_SPACE(%d)) {\\\n" (size_fields fields 0);
    fprintf avr_h "\t  ModemStartMessage(DL_%s) \\\n" s; 
    List.iter (print_avr_field avr_h) fields;
    fprintf avr_h "\t  ModemEndMessage() \\\n";
    fprintf avr_h "\t} \\\n";
    fprintf avr_h "\telse \\\n";
    fprintf avr_h "\t  modem_nb_ovrn++; \\\n";
    fprintf avr_h "}\n\n"

  let print_null_avr_macro = fun avr_h {name=s; fields = fields} ->
    fprintf avr_h "#define DOWNLINK_SEND_%s(" s;
    print_avr_macro_names avr_h fields;
    fprintf avr_h ") {}\n"

  let print_enum = fun avr_h messages ->
    List.iter (fun m -> fprintf avr_h "#define DL_%s %d\n" m.name m.id) messages;
    fprintf avr_h "#define DL_MSG_NB %d\n\n" (List.length messages)

  let print_avr_macros = fun filename avr_h messages ->
    print_enum avr_h messages;
    List.iter (print_avr_macro avr_h) messages;
    let md5sum = Digest.file filename in
    fprintf avr_h "#define MESSAGES_MD5SUM \"";
    for i = 0 to String.length md5sum - 1 do
      fprintf avr_h "\\%03o" (Char.code md5sum.[i])
    done;
    fprintf avr_h "\"\n"
      
  let print_null_avr_macros = fun avr_h messages ->
    List.iter (print_null_avr_macro avr_h) messages

  let freq = 10
  let buffer_length = 5
  let step = 1. /. float freq
  let nb_steps = (256 / freq) * freq

  let is_periodic = fun m -> m.period <> None
  let period_of = fun m ->
    match m.period with Some p -> p | None -> failwith "period_of"
  let morefrequent = fun m1 m2 -> compare (period_of m1) (period_of m2)

  let gen_periodic = fun avr_h messages ->
    let periodic_messages = List.filter is_periodic messages in
    let periodic_messages = List.sort morefrequent periodic_messages in

    let load = Array.create nb_steps 0 in
    let buffer_load = Array.create nb_steps 0 in
    
    let scheduled_messages = 
      List.map
	(fun m ->
	  let p = period_of m in
	  let period_steps = truncate (p /. step) in
	  let start_step = ref 0 in
	  for i = 1 to period_steps - 1 do
	    if (load.(i), buffer_load.(i)) < (load.(!start_step), buffer_load.(!start_step)) then start_step := i
	  done;
	  let s = size_of_message m in
	  for j = 0 to nb_steps/period_steps - 1 do
	    let i = !start_step+j*period_steps in
	    load.(i) <- load.(i) + s;
	    for k = i to i + buffer_length - 1 do
	      let k = (k + nb_steps) mod nb_steps in
	      buffer_load.(k) <- buffer_load.(k) + s
	    done
	  done;
	  (!start_step, period_steps, m))
	periodic_messages in
    
    fprintf avr_h "// Load: intant(buffer)";
    for i = 0 to nb_steps - 1 do
      fprintf avr_h " %d(%d)" load.(i) buffer_load.(i)
    done;
    fprintf avr_h "\n";
 
    fprintf avr_h "#define PeriodicSend() {  /* %dHz */ \\\n" freq;
    fprintf avr_h "  static uint8_t i;\\\n";
    fprintf avr_h "  i++; if (i == %d) i = 0;\\\n" nb_steps;
    List.iter
      (fun (s, p, m) ->
	fprintf avr_h "  if (i %% %d == %d) PERIODIC_SEND_%s();\\\n" p s m.name)
      scheduled_messages;
    fprintf avr_h "}\n"

    
	
end

let _ =
  if Array.length Sys.argv <> 3 then begin
    failwith (sprintf "Usage: %s <.xml file> <class_name>" Sys.argv.(0)) 
  end;
  let filename = Sys.argv.(1) in
  let class_name = Sys.argv.(2) in
  let base = Filename.basename (Filename.chop_extension filename) ^ class_name in

  let messages = Syntax.read filename class_name in

  let avr_h = stdout in
  Printf.fprintf avr_h "/* Automatically generated from %s */\n" filename;
  Printf.fprintf avr_h "/* Please DO NOT EDIT */\n";
  Printf.fprintf avr_h "#ifdef DOWNLINK\n";
  Gen_onboard.print_avr_macros filename avr_h messages;
  Printf.fprintf avr_h "#else // DOWNLINK\n";
  Gen_onboard.print_null_avr_macros avr_h messages;
  Printf.fprintf avr_h "#endif // DOWNLINK\n";
  Gen_onboard.gen_periodic avr_h messages
