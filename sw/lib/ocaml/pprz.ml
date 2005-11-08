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
    value : value
  }

type values = (string * value) list




let (//) = Filename.concat
let lazy_messages_xml = lazy (Xml.parse_file (Env.paparazzi_src // "conf" // "messages.xml"))
let messages_xml = fun () -> Lazy.force lazy_messages_xml

external float_of_bytes : string -> int -> float = "c_float_of_indexed_bytes"
external int32_of_bytes : string -> int -> int32 = "c_int32_of_indexed_bytes"
external int8_of_bytes : string -> int -> int = "c_int8_of_indexed_bytes"
external int16_of_bytes : string -> int -> int = "c_int16_of_indexed_bytes"
external sprint_float : string -> int -> float -> unit = "c_sprint_float"
external sprint_int32 : string -> int -> int32 -> unit = "c_sprint_int32"

let types = [
  ("uint8",  { format = "%u"; glib_type = "guint8";  size = 1; value=Int 42 });
  ("uint16", { format = "%u";  glib_type = "guint16"; size = 2; value=Int 42 });
  ("uint32", { format = "%lu" ;  glib_type = "guint32"; size = 4; value=Int 42 });
  ("int8",   { format = "%d"; glib_type = "gint8";   size = 1; value= Int 42 });
  ("int16",  { format = "%d";  glib_type = "gint16";  size = 2; value= Int 42 });
  ("int32",  { format = "%ld" ;  glib_type = "gint32";  size = 4; value=Int 42 });
  ("float",  { format = "%f" ;  glib_type = "gfloat";  size = 4; value=Float 4.2 });
  ("string",  { format = "%s" ;  glib_type = "gchar*";  size = max_int; value=String "42" })
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


let magic = fun x -> (Obj.magic x:('a,'b,'c) Pervasives.format)


let formatted_string_of_value = fun format v ->
  match v with
    Float x -> sprintf (magic format) x
  | v -> string_of_value v

let size_of_field = fun f -> (List.assoc f._type types).size
let default_format = fun x -> try (List.assoc x types).format with Not_found -> failwith (sprintf "Unknwon format '%s'" x)
let default_value = fun x -> (List.assoc x types).value

let payload_size_of_message = fun message ->
  List.fold_right
    (fun (_, f) s -> size_of_field f + s)
    message.fields
    1

let size_of_message = fun m -> 
  payload_size_of_message m + 3 (* STX, CK_A, CK_B *)

let field_of_xml = fun xml ->
  let t = ExtXml.attrib xml "type" in
  let f = try Xml.attrib xml "format" with _ -> default_format t in
  (ExtXml.attrib xml "name", { _type = t; fformat = f })

let string_of_values = fun vs ->
  String.concat " " (List.map (fun (a,v) -> sprintf "%s=%s" a (string_of_value v)) vs)

let assoc = fun a vs -> 
  try List.assoc a vs with Not_found -> 
    failwith (sprintf "Attribute '%s' not found in '%s'" a (string_of_values vs))

let float_assoc = fun (a:string) vs -> 
  match assoc a vs with
    Float x -> x
  | _ -> invalid_arg "Pprz.float_assoc"

let int_assoc = fun (a:string) vs -> 
  match assoc a vs with
    Int x -> x
  | _ -> invalid_arg "Pprz.int_assoc"

let int32_assoc = fun (a:string) vs -> 
  match assoc a vs with
    Int32 x -> int_of_string (string_of_value (Int32 x))
  | _ -> invalid_arg "Pprz.int_assoc"

let string_assoc = fun (a:string) (vs:values) -> string_of_value (assoc a vs)



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
	    let name = ExtXml.attrib xml_msg "name" in
	    let msg = {
	      name = name;
	      fields = List.map field_of_xml (Xml.children xml_msg)
	    } in
	    let id = int_of_string (ExtXml.attrib xml_msg "id") in
	    if Hashtbl.mem by_id id then
	      failwith (sprintf "Duplicated id in messages.xml: %d" id);
	    Hashtbl.add by_id id msg;
	    Hashtbl.add by_name name (id, msg))
	  (Xml.children xml_class);
	Hashtbl.add h (ExtXml.attrib xml_class "name") (by_id, by_name)
      )
      (Xml.children (messages_xml ()));
    h)

let classes = fun () -> Lazy.force lazy_classes
    
let value_field = fun buffer index (field:field) ->
  match field._type with
    "uint8" -> Int (Char.code buffer.[index])
  | "int8" -> Int (int8_of_bytes buffer index)
  | "uint16" -> Int (Char.code buffer.[index+1] lsl 8 + Char.code buffer.[index])
  | "int16" -> Int (int16_of_bytes buffer index)
  | "float" ->  Float (float_of_bytes buffer index)
  | "int32"  | "uint32" -> Int32 (int32_of_bytes buffer index)
  | _ -> failwith "value_field"

let byte = fun x -> Char.chr (x land 0xff)

let sprint_value = fun buf i field_type v ->
  match field_type, v with
    ("int8"|"uint8"), Int x -> buf.[i] <- Char.chr x
  | "float", Float f -> sprint_float buf i f
  | "int32", Int32 x -> sprint_int32 buf i x
  | ("int32" | "uint32"), Int value ->
      assert (field_type <> "uint32" || value >= 0);
      buf.[i+3] <- byte (value asr 24);
      buf.[i+2] <- byte (value lsr 16);
      buf.[i+1] <- byte (value lsr 8);
      buf.[i+0] <- byte value
  | ("int16"|"uint16"), Int value ->
      assert (field_type <> "uint16" || value >= 0);
      buf.[i+1] <- byte (value lsr 8);
      buf.[i+0] <- byte value
  | x, _ -> failwith (sprintf "Pprz.sprint_value (%s)" x)
  
  

module type CLASS = sig val name : string end

exception Unknown_msg_name of string

module Protocol(Class:CLASS) = struct
  let stx = Char.chr 0x05
  let index_start = fun buf ->
    String.index buf stx

  let messages_by_id, messages_by_name = 
    try
      Hashtbl.find (classes ()) Class.name
    with
      Not_found -> failwith (sprintf "Unknown message class: %s" Class.name)
  let message_of_id = fun id -> Hashtbl.find messages_by_id id
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
    Debug.call 'T' (fun f -> fprintf f "Pprz cs: %d %d\n" !ck_a (Char.code msg.[l-2]));
    !ck_a = Char.code msg.[l-2] && !ck_b = Char.code msg.[l-1]

  let values_of_payload = fun buffer ->
    let id = Char.code buffer.[0] in
    let message = message_of_id id in
    Debug.call 'T' (fun f -> fprintf f "Pprz.values id=%d\n" id);
    let rec loop = fun index fields ->
      match fields with
	[] -> []
      | (field_name, field_descr)::fs -> 
	  let n = size_of_field field_descr in
	  (field_name, value_field buffer index field_descr) :: loop (index+n) fs in
    (id, loop 1 message.fields)

  let values_of_bin = fun buffer ->
    values_of_payload (String.sub buffer 1 (String.length buffer - 1))

  let payload_of_values = fun id values ->
    let message = message_of_id id in
    let n = payload_size_of_message message in
    let p = String.make n '#' in
    p.[0] <- Char.chr id;
    let i = ref 1 in
    List.iter
      (fun (field_name, field) ->
	let v =
	  try List.assoc field_name values with
	    Not_found -> default_value field._type in
	sprint_value p !i field._type v;
	i := !i + size_of_field field	
	)
      message.fields;
    p
    

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
	  | Invalid_argument "List.map2" -> failwith (sprintf "Pprz.values_of_string: '%s'" s)
	end
    | [] -> invalid_arg "Pprz.values_of_string"

  let string_of_message = fun msg values ->
    String.concat " "
      (msg.name::
       List.map 
	 (fun (field_name, field) ->
	   let v =
	     try List.assoc field_name values with
	       Not_found ->
		 default_value field._type in
	   formatted_string_of_value field.fformat v)
	 msg.fields)

  let message_send = fun sender msg_name values ->
    let m = snd (message_of_name msg_name) in
    let s = string_of_message m values in
    Ivy.send (sprintf "%s %s" sender s)

  let message_bind = fun ?sender msg_name cb ->
    match sender with
      None ->
	Ivy.bind (fun _ args -> cb args.(0) (snd (values_of_string args.(1)))) (sprintf "^([^ ]*) +(%s .*)" msg_name)
    | Some s ->
	Ivy.bind (fun _ args -> cb s (snd (values_of_string args.(0)))) (sprintf "^%s +(%s .*)" s msg_name)
	  

  let message_answerer = fun sender msg_name cb ->
    let ivy_cb = fun _ args ->
      let asker = args.(0)
      and asker_id = args.(1) in
      let values = cb asker (snd (values_of_string args.(2))) in
      let m = string_of_message (snd (message_of_name msg_name)) values in
      Ivy.send (sprintf "%s %s %s" asker_id sender m) in
    Ivy.bind ivy_cb (sprintf "^([^ ]*) +([^ ]*) +(%s_REQ.*)" msg_name)

  let gen_id = let r = ref 0 in fun () -> incr r; !r
  let message_req = fun sender msg_name values (f:string -> (string * value) list -> unit) ->
    let b = ref (Obj.magic ()) in
    let cb = fun _ args ->
      Ivy.unbind !b;
      f args.(0) (snd (values_of_string args.(1))) in
    let id = sprintf "%d_%d" (Unix.getpid ()) (gen_id ()) in
    let r = sprintf "^%s ([^ ]*) +(%s.*)" id msg_name in
    b := Ivy.bind cb r;
    let msg_name_req = msg_name ^ "_REQ" in
    let m = sprintf "%s %s %s" sender id (string_of_message (snd (message_of_name msg_name_req)) values) in
    Ivy.send m
end
