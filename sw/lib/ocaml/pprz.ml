(*
 * $Id$
 *
 * Downlink protocol (handling messages.xml)
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


type message_id = int
type class_name = string
type format = string
type _type = string
type value = Int of int | Float of float | String of string | Int32 of int32
type field = {
    _type : _type;
    fformat : format;
  }

type message = {
    name : string;
    fields : (string * field) list
  }

type type_descr = {
    format : string ;
    glib_type : string;
    size : int;
    value : string
  }




let (//) = Filename.concat
let lazy_messages_xml = lazy (Xml.parse_file (Env.paparazzi_src // "conf" // "messages.xml"))
let messages_xml = fun () -> Lazy.force lazy_messages_xml

external float_of_bytes : string -> int -> float = "c_float_of_indexed_bytes"
external int32_of_bytes : string -> int -> int32 = "c_int32_of_indexed_bytes"
let types = [
  ("uint8",  { format = "%u"; glib_type = "guint8";  size = 1; value="42" });
  ("uint16", { format = "%u";  glib_type = "guint16"; size = 2; value="42" });
  ("uint32", { format = "%lu" ;  glib_type = "guint32"; size = 4; value="42" });
  ("int8",   { format = "%d"; glib_type = "gint8";   size = 1; value="42" });
  ("int16",  { format = "%d";  glib_type = "gint16";  size = 2; value="42" });
  ("int32",  { format = "%ld" ;  glib_type = "gint32";  size = 4; value="42" });
  ("float",  { format = "%f" ;  glib_type = "gfloat";  size = 4; value="4.2" })
]

let int_of_string = fun x ->
  try int_of_string x with
    _ -> failwith (sprintf "Pprz.int_of_string: %s" x)

let value = fun t v ->
  match t with 
    "uint8" | "uint16" | "int8" | "int16" -> Int (int_of_string v)
  | "uint32" | "int32" -> Int32 (Int32.of_string v)
  | "float" -> Float (float_of_string v)
  | "string" -> String v
  | _ -> failwith (sprintf "Pprz.value: Unexpected type: %s" t)

let string_of_value = function
    Int x -> string_of_int x
  | Float x -> string_of_float x
  | Int32 x -> Int32.to_string x
  | String s -> s

let size_of_field = fun f -> (List.assoc f._type types).size
let default_format = fun x -> (List.assoc x types).format
let default_value = fun x -> (List.assoc x types).value

let size_of_message = fun message ->
  List.fold_right
    (fun (_, f) s -> size_of_field f + s)
    message.fields
    4

let field_of_xml = fun xml ->
  let t = ExtXml.attrib xml "type" in
  let f = try Xml.attrib xml "format" with _ -> default_format t in
  (ExtXml.attrib xml "name", { _type = t; fformat = f })


(** Table of msg classes indexed by name. Each class is a table of messages
   indexed by ids *)
let lazy_classes =
  lazy
    (let h = Hashtbl.create 13 in
    List.iter
      (fun xml_class ->
	let by_id = Hashtbl.create 13
	and by_name = Hashtbl.create 13 in
	List.iter
	  (fun xml_msg ->
	    try
	      let name = ExtXml.attrib xml_msg "name" in
	      let msg = {
		name = name;
		fields = List.map field_of_xml (Xml.children xml_msg)
	      } in
	      let id = int_of_string (ExtXml.attrib xml_msg "id") (* - 1 !!!!*) in
	      Hashtbl.add by_id id msg;
	      Hashtbl.add by_name name (id, msg)	      
	    with _ ->
	      fprintf stderr "Warning: Ignoring '%s'\n" (Xml.to_string xml_msg))
	  (Xml.children xml_class);
	Hashtbl.add h (ExtXml.attrib xml_class "name") (by_id, by_name)
      )
      (Xml.children (messages_xml ()));
    h)

let classes = fun () -> Lazy.force lazy_classes
    
let magic = fun x -> (Obj.magic x:('a,'b,'c) Pervasives.format)

let value_field = fun buffer index (field:field) ->
  let format = field.fformat in
  match field._type with
    "uint8" -> Int (Char.code buffer.[index])
  | "int8" -> Int (if Char.code buffer.[index] <= 128 then Char.code buffer.[index] else Char.code buffer.[index] - 256)
  | "uint16" -> Int (Char.code buffer.[index] lsl 8 + Char.code buffer.[index+1])
  | "int16" -> Int (if Char.code buffer.[index] lsl 8 + Char.code buffer.[index+1] <= 32768 then Char.code buffer.[index] lsl 8 + Char.code buffer.[index+1] else Char.code buffer.[index] lsl 8 + Char.code buffer.[index+1] - 65536)
  | "float" ->  Float (float_of_bytes buffer index)
  | "int32"  | "uint32" -> Int32 (int32_of_bytes buffer index)
  | _ -> failwith "value_field"

module type CLASS = sig val name : string end

exception Unknown_msg_name of string

module Protocol(Class:CLASS) = struct
  let stx = Char.chr 0x05
  let index_start = fun buf ->
    String.index buf stx

  let messages_by_id, messages_by_name = Hashtbl.find (classes ()) Class.name
  let message_of_id = fun id -> Hashtbl.find messages_by_id (id (*** +1 ***))
  let message_of_name = fun name -> Hashtbl.find messages_by_name name

  let length = fun buf start ->
    let len = String.length buf - start in
    if len >= 2 then
      let id = Char.code buf.[start+1] in
      let msg = message_of_id id in
      let l = size_of_message msg in
      Debug.call 'T' (fun f -> fprintf f "Pprz id=%d len=%d\n" id l);
      l
    else
      raise Serial.Not_enough

  let (+=) = fun r x -> r := (!r + x) land 0xff
  let checksum = fun msg ->
    let l = String.length msg in
    let ck_a = ref 0  and ck_b = ref 0 in
    for i = 1 to l - 3 do
      ck_a += Char.code msg.[i];
      ck_b += !ck_a
    done;
    Debug.call 'T' (fun f -> fprintf f "Pprz ck: %d %d\n" !ck_a (Char.code msg.[l-2]));
    !ck_a = Char.code msg.[l-2] && !ck_b = Char.code msg.[l-1]

  let values_of_bin = fun buffer ->
    let id = Char.code buffer.[1] in
    let message = message_of_id id in
    Debug.call 'T' (fun f -> fprintf f "Pprz.values id=%d\n" id);
    let rec loop = fun index fields ->
      match fields with
	[] -> []
      | (field_name, field_descr)::fs -> 
	  let n = size_of_field field_descr in
	  (field_name, value_field buffer index field_descr) :: loop (index+n) fs in
    (id, loop 2 message.fields)

  let space = Str.regexp "[ \t]+"
  let values_of_string = fun s ->
    match Str.split space s with
      msg_name::args ->
	begin
	  try
	    let msg_id, msg = message_of_name msg_name in
	    let values = List.map2 (fun (field_name, field) v -> (field_name, value field._type v)) msg.fields args in
	    (msg_id, values)
	  with
	    Not_found -> raise (Unknown_msg_name msg_name)
	end
    | [] -> invalid_arg "Pprz.values_of_string"

  let string_of_message = fun msg values ->
    String.concat " "
      (msg.name::
       List.map 
	 (fun (field_name, field) ->
	   try string_of_value (List.assoc field_name values) with Not_found -> default_value field._type)
	 msg.fields)
end
