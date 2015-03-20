(*
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
open Latlong

type message_id = int
type ac_id = int
type class_name = string
type format = string
type _type =
    Scalar of string
  | ArrayType of string
  | FixedArrayType of string * int
type value =
    Int of int | Float of float | String of string | Int32 of int32 | Char of char | Int64 of int64
  | Array of value array
type field = {
  _type : _type;
  fformat : format;
  alt_unit_coef : string;
  enum : string list
}

type link_mode = Forwarded | Broadcasted
type message = {
  name : string; (** Lowercase *)
  fields : (string * field) list;
  link : link_mode option
}

type type_descr = {
  format : string ;
  glib_type : string;
  inttype : string;
  size : int;
  value : value
}

type values = (string * value) list

type payload = string


let separator = ","
let regexp_separator = Str.regexp ","
let split_array = fun s -> Str.split regexp_separator s


let (//) = Filename.concat
let messages_file = Env.paparazzi_src // "conf" // "messages.xml"
let lazy_messages_xml = lazy (Xml.parse_file messages_file)
let messages_xml = fun () -> Lazy.force lazy_messages_xml
let units_file = Env.paparazzi_src // "conf" // "units.xml"

external float_of_bytes : string -> int -> float = "c_float_of_indexed_bytes"
external double_of_bytes : string -> int -> float = "c_double_of_indexed_bytes"
external int8_of_bytes : string -> int -> int = "c_int8_of_indexed_bytes"
external int16_of_bytes : string -> int -> int = "c_int16_of_indexed_bytes"
external int32_of_bytes : string -> int -> int32 = "c_int32_of_indexed_bytes"
external uint32_of_bytes : string -> int -> int64 = "c_uint32_of_indexed_bytes"
external int64_of_bytes : string -> int -> int64 = "c_int64_of_indexed_bytes"
external sprint_float : string -> int -> float -> unit = "c_sprint_float"
external sprint_double : string -> int -> float -> unit = "c_sprint_double"
external sprint_int64 : string -> int -> int64 -> unit = "c_sprint_int64"
external sprint_int32 : string -> int -> int32 -> unit = "c_sprint_int32"
external sprint_int16 : string -> int -> int -> unit = "c_sprint_int16"
external sprint_int8 : string -> int -> int -> unit = "c_sprint_int8"

let types = [
  ("uint8",  { format = "%u"; glib_type = "guint8"; inttype = "uint8_t";  size = 1; value=Int 42 });
  ("uint16", { format = "%u";  glib_type = "guint16"; inttype = "uint16_t"; size = 2; value=Int 42 });
  ("uint32", { format = "%Lu" ;  glib_type = "guint32"; inttype = "uint32_t"; size = 4; value=Int 42 }); (* uint32 should be lu, but doesn't fit into Int32 so Int64 (Lu) is used *)
  ("uint64", { format = "%Lu" ;  glib_type = "guint64"; inttype = "uint64_t"; size = 8; value=Int 42 });
  ("int8",   { format = "%d"; glib_type = "gint8"; inttype = "int8_t";   size = 1; value= Int 42 });
  ("int16",  { format = "%d";  glib_type = "gint16"; inttype = "int16_t";  size = 2; value= Int 42 });
  ("int32",  { format = "%ld" ;  glib_type = "gint32"; inttype = "int32_t";  size = 4; value=Int 42 });
  ("int64",  { format = "%Ld" ;  glib_type = "gint64"; inttype = "int64_t";  size = 8; value=Int 42 });
  ("float",  { format = "%f" ;  glib_type = "gfloat"; inttype = "float";  size = 4; value=Float 4.2 });
  ("double", { format = "%f" ;  glib_type = "gdouble"; inttype = "double";  size = 8; value=Float 4.2 });
  ("char",   { format = "%c" ;  glib_type = "gchar"; inttype = "char";  size = 1; value=Char '*' });
  ("string", { format = "%s" ;  glib_type = "gchar*"; inttype = "char*";  size = max_int; value=String "42" })
]

let is_array_type = fun s ->
  let n = String.length s in
  n >= 2 && String.sub s (n-2) 2 = "[]"

let type_of_array_type = fun s ->
  let n = String.length s in
  String.sub s 0 (n-2)

let is_fixed_array_type = fun s ->
  let type_parts = Str.full_split (Str.regexp "[][]") s in
  match type_parts with
    | [Str.Text _; Str.Delim "["; Str.Text _ ; Str.Delim "]"] -> true
    | _ -> false

let type_of_fixed_array_type = fun s ->
  try
    let type_parts = Str.full_split (Str.regexp "[][]") s in
    match type_parts with
      | [Str.Text ty; Str.Delim "["; Str.Text len ; Str.Delim "]"] -> begin ignore( int_of_string (len)); ty end
      | _ -> failwith("Pprz.type_of_fixed_array_type is not a fixed array type")
  with
    | Failure str -> failwith(sprintf "Pprz.type_of_fixed_array_type: length is not an integer")

let length_of_fixed_array_type = fun s ->
  try
    let type_parts = Str.full_split (Str.regexp "[][]") s in
    match type_parts with
      | [Str.Text ty; Str.Delim "["; Str.Text len ; Str.Delim "]"] -> begin ignore( int_of_string (len)); len end
      | _ -> failwith("Pprz.type_of_fixed_array_type is not a fixed array type")
  with
    | Failure str -> failwith(sprintf "Pprz.type_of_fixed_array_type: length is not an integer")

let int_of_string = fun x ->
  try int_of_string x with
      _ -> try int_of_string ("0x"^x) with (* try hex format in case *)
        _ -> failwith (sprintf "Pprz.int_of_string: %s" x)

let rec value = fun t v ->
  match t with
      Scalar ("uint8" | "uint16" | "int8" | "int16") -> Int (int_of_string v)
    | Scalar "int32" -> Int32 (Int32.of_string v)
    | Scalar "uint32" -> Int64 (Int64.of_string v)
    | Scalar ("uint64" | "int64") -> Int64 (Int64.of_string v)
    | Scalar ("float" | "double") -> Float (float_of_string v)
    | ArrayType "char" | FixedArrayType ("char", _) | Scalar "string" -> String v
    | Scalar "char" -> Char v.[0]
    | ArrayType t' ->
        Array (Array.map (value (Scalar t')) (Array.of_list (split_array v)))
    | FixedArrayType (t',l') ->
        Array (Array.map (value (Scalar t')) (Array.of_list (split_array v)))
    | Scalar t -> failwith (sprintf "Pprz.value: Unexpected type: %s" t)


let rec string_of_value = function
    Int x -> string_of_int x
  | Float x -> string_of_float x
  | Int32 x -> Int32.to_string x
  | Int64 x -> Int64.to_string x
  | Char c -> String.make 1 c
  | String s -> s
  | Array a ->
      let l = (Array.to_list (Array.map string_of_value a)) in
      match a.(0) with
      | Char _ -> "\""^(String.concat "" l)^"\""
      | _ -> String.concat separator l


let rec formatted_string_of_value = fun format v ->
  let f = fun x -> Scanf.format_from_string format x in
  match v with
    | Int x -> sprintf (f "%d") x
    | Float x -> sprintf (f "%f") x
    | Int32 x -> sprintf (f "%ld") x
    | Int64 x -> sprintf (f "%Ld") x
    | Char x -> sprintf (f "%c") x
    | String x ->sprintf "%s" x
    | Array a ->
        let l = (Array.to_list (Array.map (formatted_string_of_value format) a)) in
        match a.(0) with
        | Char _ -> "\""^(String.concat "" l)^"\""
        | _ -> String.concat separator l


let sizeof = fun f ->
  match f with
      Scalar t -> (List.assoc t types).size
    | ArrayType t -> failwith "sizeof: Array"
    | FixedArrayType (t,l) -> failwith "sizeof: Array"
let size_of_field = fun f -> sizeof f._type
let default_format = function Scalar x | ArrayType x | FixedArrayType (x,_) ->
  try (List.assoc x types).format with
      Not_found -> failwith (sprintf "Unknown format '%s'" x)
let default_value = fun x ->
  match x with
      Scalar t -> (List.assoc t types).value
    | ArrayType t -> failwith "default_value: Array"
    | FixedArrayType (t,l) -> failwith "default_value: Array"

let payload_size_of_message = fun message ->
  List.fold_right
    (fun (_, f) s -> size_of_field f + s)
    message.fields
    2 (** + message id + aircraft id *)

exception Unit_conversion_error of string
exception Unknown_conversion of string * string
exception No_automatic_conversion of string * string

let scale_of_units = fun ?auto from_unit to_unit ->
  if (from_unit = to_unit) then
    1.0
  else
    try
      let units_xml = Xml.parse_file units_file in
      (* find the first occurence of matching units or raise Not_found *)
      let _unit = List.find (fun u ->
          (* will raise Xml.No_attribute if not a valid attribute *)
        let f = Xml.attrib u "from"
        and t = Xml.attrib u "to"
        and a = try Some (Xml.attrib u "auto") with _ -> None in
        let a = match auto, a with
          | Some _, None | None, None -> "" (* No auto conversion *)
          | Some t, Some _ | None, Some t -> String.lowercase t (* param auto is used before attribute *)
        in
        if (f = from_unit || a = "display") && (t = to_unit || a = "code") then true else false
      ) (Xml.children units_xml) in
      (* return coef, raise Failure if coef is not a numerical value *)
      float_of_string (Xml.attrib _unit "coef")
    with Xml.File_not_found _ -> raise (Unit_conversion_error ("Parse error of conf/units.xml"))
      | Xml.No_attribute _ | Xml.Not_element _ -> raise (Unit_conversion_error ("File conf/units.xml has errors"))
      | Failure "float_of_string" -> raise (Unit_conversion_error ("Unit coef is not numerical value"))
      | Not_found ->
        if from_unit = "" || to_unit = "" then raise (No_automatic_conversion (from_unit, to_unit))
        else raise (Unknown_conversion (from_unit, to_unit))
      | _ -> raise (Unknown_conversion (from_unit, to_unit))


let alt_unit_coef_of_xml = fun ?auto xml ->
  try Xml.attrib xml "alt_unit_coef"
  with _ ->
    let u = try Xml.attrib xml "unit" with _ -> "" in
    let au = try Xml.attrib xml "alt_unit" with _ -> "" in
    let coef = try string_of_float (match auto with
      | None -> scale_of_units u au
      | Some a -> scale_of_units u au ~auto:a)
      with
        | Unit_conversion_error s -> prerr_endline (sprintf "Unit conversion error: %s" s); flush stderr; "1." (* Use coef 1. *)
        | Unknown_conversion _ -> "1." (* Use coef 1. *)
        | _ -> "1."
    in
    coef

let key_modifiers_of_string = fun key ->
  let key_split = Str.split (Str.regexp "\\+") key in
  let keys = List.map (fun k ->
    match k with
    | "Ctrl" -> "<Control>"
    | "Alt" -> "<Alt>"
    | "Shift" -> "<Shift>"
    | "Meta" -> "<Meta>"
    | x -> x
  ) key_split in
  String.concat "" keys

let pipe_regexp = Str.regexp "|"
let field_of_xml = fun xml ->
  let t = ExtXml.attrib xml "type" in
  let t = if is_array_type t then ArrayType (type_of_array_type t)
          else if is_fixed_array_type t then FixedArrayType (type_of_fixed_array_type t, int_of_string(length_of_fixed_array_type t))
          else Scalar t in
  let f = try Xml.attrib xml "format" with _ -> default_format t in
  let auc = alt_unit_coef_of_xml xml in
  let values = try Str.split pipe_regexp (Xml.attrib xml "values") with _ -> [] in

  ( String.lowercase (ExtXml.attrib xml "name"),
    { _type = t; fformat = f; alt_unit_coef = auc; enum=values })

let string_of_values = fun vs ->
  String.concat " " (List.map (fun (a,v) -> sprintf "%s=%s" a (string_of_value v)) vs)

let assoc = fun a vs ->
  try List.assoc (String.lowercase a) vs with Not_found ->
    failwith (sprintf "Attribute '%s' not found in '%s'" a (string_of_values vs))

let float_assoc = fun (a:string) vs ->
  match assoc a vs with
      Float x -> x
    | _ -> invalid_arg "Pprz.float_assoc"

let int_of_value = fun value ->
  match value with
      Int x -> x
    | Int32 x ->
      let i = Int32.to_int x in
      if Int32.compare x (Int32.of_int i) <> 0 then
        failwith "Pprz.int_assoc: Int32 too large to be converted into an int";
      i
    | Int64 x ->
      let i = Int64.to_int x in
      if Int64.compare x (Int64.of_int i) <> 0 then
        failwith "Pprz.int_assoc: Int64 too large to be converted into an int";
      i
    | _ -> invalid_arg "Pprz.int_assoc"

let int_assoc = fun (a:string) vs ->
  int_of_value (assoc a vs)

let int32_assoc = fun (a:string) vs ->
  match assoc a vs with
      Int32 x -> x
    | _ -> invalid_arg "Pprz.int32_assoc"

let uint32_assoc = fun (a:string) vs ->
  match assoc a vs with
      Int64 x -> x
    | _ -> invalid_arg "Pprz.uint32_assoc"

let int64_assoc = fun (a:string) vs ->
  match assoc a vs with
    Int64 x -> x
  | _ -> invalid_arg "Pprz.int64_assoc"

let string_assoc = fun (a:string) (vs:values) -> string_of_value (assoc a vs)

let char_assoc = fun (a:string) (vs:values) -> (string_of_value (assoc a vs)).[0]

let link_mode_of_string = function
"forwarded" -> Forwarded
  | "broadcasted" -> Broadcasted
  | x -> invalid_arg (sprintf "link_mode_of_string: %s" x)

let parse_class = fun xml_class ->
  let by_id = Hashtbl.create 13
  and by_name = Hashtbl.create 13 in
  List.iter
    (fun xml_msg ->
      let name = ExtXml.attrib xml_msg "name"
      and link =
        try
          Some (link_mode_of_string (Xml.attrib xml_msg "link"))
        with
            Xml.No_attribute("link") -> None
      in
      (* only keep a "field" nodes *)
      let xml_children = List.filter (fun f -> Xml.tag f = "field") (Xml.children xml_msg) in
      let msg = {
        name = name;
        fields = List.map field_of_xml xml_children;
        link = link
      } in
      let id = int_of_string (ExtXml.attrib xml_msg "id") in
      if Hashtbl.mem by_id id then
        failwith (sprintf "Duplicated id in messages.xml: %d" id);
      Hashtbl.add by_id id msg;
      Hashtbl.add by_name name (id, msg))
    (Xml.children xml_class);
  (by_id, by_name)


(** Returns a value and its length *)
let rec value_of_bin = fun buffer index _type ->
  match _type with
      Scalar "uint8" -> Int (Char.code buffer.[index]), sizeof _type
    | Scalar "char" -> Char (buffer.[index]), sizeof _type
    | Scalar "int8" -> Int (int8_of_bytes buffer index), sizeof _type
    | Scalar "uint16" -> Int (Char.code buffer.[index+1] lsl 8 + Char.code buffer.[index]), sizeof _type
    | Scalar "int16" -> Int (int16_of_bytes buffer index), sizeof _type
    | Scalar "float" -> Float (float_of_bytes buffer index), sizeof _type
    | Scalar "double" -> Float (double_of_bytes buffer index), sizeof _type
    | Scalar "int32" -> Int32 (int32_of_bytes buffer index), sizeof _type
    | Scalar "uint32" -> Int64 (uint32_of_bytes buffer index), sizeof _type
    | Scalar ("int64"  | "uint64") -> Int64 (int64_of_bytes buffer index), sizeof _type
    | ArrayType t ->
      (** First get the number of values *)
      let n = int8_of_bytes buffer index in
      let type_of_elt = Scalar t in
      let s = sizeof type_of_elt in
      let size = 1 + n * s in
      (Array (Array.init n
          (fun i -> fst (value_of_bin buffer (index+1+i*s) type_of_elt))), size)
    | FixedArrayType (t,l) ->
      (** First get the number of values *)
      let n = l in
      let type_of_elt = Scalar t in
      let s = sizeof type_of_elt in
      let size = 0 + n * s in
      (Array (Array.init n
         (fun i -> fst (value_of_bin buffer (index+0+i*s) type_of_elt))), size)
    | Scalar "string" ->
      let n = Char.code buffer.[index] in
      (String (String.sub buffer (index+1) n), (1+n))
    | _ -> failwith "value_of_bin"

let value_field = fun buf index field ->
  value_of_bin buf index field._type

let byte = fun x -> Char.chr (x land 0xff)

(** Returns the size of outputed data *)
let rec sprint_value = fun buf i _type v ->
  match _type, v with
      Scalar "uint8", Int x ->
        if x < 0 || x > 0xff then
          failwith (sprintf "Value too large to fit in a uint8: %d" x);
        buf.[i] <- Char.chr x; sizeof _type
    | Scalar "int8", Int x ->
      if x < -0x7f || x > 0x7f then
        failwith (sprintf "Value too large to fit in a int8: %d" x);
      sprint_int8 buf i x; sizeof _type
    | Scalar "float", Float f -> sprint_float buf i f; sizeof _type
    | Scalar "double", Float f -> sprint_double buf i f; sizeof _type
    | Scalar "int32", Int32 x -> sprint_int32 buf i x; sizeof _type
    | Scalar ("int64"|"uint64"|"uint32"), Int64 x -> sprint_int64 buf i x; sizeof _type
    | Scalar "int16", Int x -> sprint_int16 buf i x; sizeof _type
    | Scalar ("int32" | "uint32"), Int value ->
      assert (_type <> Scalar "uint32" || value >= 0);
      buf.[i+3] <- byte (value asr 24);
      buf.[i+2] <- byte (value lsr 16);
      buf.[i+1] <- byte (value lsr 8);
      buf.[i+0] <- byte value;
      sizeof _type
    | Scalar ("int64" | "uint64"), Int value ->
        assert (_type <> Scalar "uint64" || value >= 0);
        buf.[i+7] <- byte (value asr 56);
        buf.[i+6] <- byte (value lsr 48);
        buf.[i+5] <- byte (value lsr 40);
        buf.[i+4] <- byte (value lsr 32);
        buf.[i+3] <- byte (value lsr 24);
        buf.[i+2] <- byte (value lsr 16);
        buf.[i+1] <- byte (value lsr 8);
        buf.[i+0] <- byte value;
        sizeof _type
    | Scalar "uint16", Int value ->
      assert (value >= 0);
      buf.[i+1] <- byte (value lsr 8);
      buf.[i+0] <- byte value;
      sizeof _type
    | ArrayType t, Array values ->
      (** Put the size first, then the values *)
      let n = Array.length values in
      ignore (sprint_value buf i (Scalar "uint8") (Int n));
      let type_of_elt = Scalar t in
      let s = sizeof type_of_elt in
      for j = 0 to n - 1 do
        ignore (sprint_value buf (i+1+j*s) type_of_elt values.(j))
        done;
        1 + n * s
    | FixedArrayType (t,l), Array values ->
        (** Put the size first, then the values *)
        let n = Array.length values in
        let type_of_elt = Scalar t in
        let s = sizeof type_of_elt in
        for j = 0 to n - 1 do
          ignore (sprint_value buf (i+0+j*s) type_of_elt values.(j))
        done;
        0 + n * s
    | Scalar "string", String s ->
      let n = String.length s in
      assert (n < 256);
      (** Put the length first, then the bytes *)
      buf.[i] <- Char.chr n;
      if (i + n >= String.length buf) then
        failwith "Error in sprint_value: message too long";
      String.blit s 0 buf (i+1) n;
      1 + n
    | Scalar "char", Char c ->
        buf.[i] <- c; sizeof _type
    | (Scalar x|ArrayType x), _ -> failwith (sprintf "Pprz.sprint_value (%s)" x)
    | FixedArrayType (x,l), _ -> failwith (sprintf "Pprz.sprint_value (%s)" x)



let hex_of_int_array = function
Array array ->
  let n = Array.length array in
      (* One integer -> 2 chars *)
  let s = String.create (2*n) in
  Array.iteri
    (fun i dec ->
      let x = int_of_value array.(i) in
      assert (0 <= x && x <= 0xff);
      let hex = sprintf "%02x" x in
      String.blit hex 0 s (2*i) 2)
    array;
  s
  | value ->
    failwith (sprintf "Error: expecting array in Pprz.hex_of_int_array, found %s" (string_of_value value))



exception Unknown_msg_name of string * string

module type TRANSPORT_TYPE = sig
  val stx : char
  val offset_length : int
  val offset_payload : int
  val offset_timestamp : int
  val overhead_length : int
end

module PprzTransportBase(SubType:TRANSPORT_TYPE) = struct
  let index_start = fun buf ->
    String.index buf SubType.stx

  let length = fun buf start ->
    let len = String.length buf - start in
    if len > SubType.offset_length then
      let l = Char.code buf.[start+SubType.offset_length] in
      Debug.call 'T' (fun f -> fprintf f "Pprz len=%d\n" l);
      max l SubType.overhead_length (** if l<4 (4=stx+length+ck_a+ck_b), it's not a valid length *)
    else
      raise Serial.Not_enough

  let (+=) = fun r x -> r := (!r + x) land 0xff
  let compute_checksum = fun msg ->
    let l = String.length msg in
    let ck_a = ref 0  and ck_b = ref 0 in
    for i = 1 to l - 3 do
      ck_a += Char.code msg.[i];
      ck_b += !ck_a
    done;
    !ck_a, !ck_b

  let checksum = fun msg ->
    let l = String.length msg in
    let ck_a, ck_b = compute_checksum msg in
    Debug.call 'T' (fun f -> fprintf f "Pprz cs: %d %d | %d %d\n" ck_a (Char.code msg.[l-2]) ck_b (Char.code msg.[l-1]));
    ck_a = Char.code msg.[l-2] && ck_b = Char.code msg.[l-1]

  let payload = fun msg ->
    let l = String.length msg in
    assert(Char.code msg.[SubType.offset_length] = l);
    assert(l >= SubType.overhead_length);
    Serial.payload_of_string (String.sub msg SubType.offset_payload (l-SubType.overhead_length))

  let packet = fun payload ->
    let payload = Serial.string_of_payload payload in
    let n = String.length payload in
    let msg_length = n + SubType.overhead_length in (** + stx, len, ck_a and ck_b *)
    if msg_length >= 256 then
      invalid_arg "Pprz.Transport.packet";
    let m = String.create msg_length in
    String.blit payload 0 m SubType.offset_payload n;
    m.[0] <- SubType.stx;
    m.[SubType.offset_length] <- Char.chr msg_length;
    let (ck_a, ck_b) = compute_checksum m in
    m.[msg_length-2] <- Char.chr ck_a;
    m.[msg_length-1] <- Char.chr ck_b;
    m
end

module PprzTypeStandard = struct
  let stx = Char.chr 0x99
  let offset_length = 1
  let offset_timestamp = -1
  let offset_payload = 2
  let overhead_length = 4
end

module PprzTypeTimestamp = struct
  let stx = Char.chr 0x98
  let offset_length = 1
  let offset_timestamp = 2
  let offset_payload = 6
  let overhead_length = 8
end

module Transport = PprzTransportBase (PprzTypeStandard)
module TransportExtended = PprzTransportBase (PprzTypeTimestamp)

let offset_ac_id = 0
let offset_msg_id = 1
let offset_fields = 2

module type CLASS_Xml = sig
  val xml : Xml.xml
  val name : string
end

module type CLASS = sig
  val name : string
end

module type MESSAGES = sig
  val messages : (message_id, message) Hashtbl.t
  val message_of_id : message_id -> message
  val message_of_name : string ->  message_id * message
  val values_of_payload : Serial.payload -> message_id * ac_id * values
  (** [values_of_bin payload] Parses a raw payload, returns the
      message id, the A/C id and the list of (field_name, value) *)

  val payload_of_values : message_id -> ac_id -> values -> Serial.payload
  (** [payload_of_values id ac_id vs] Returns a payload *)

  val values_of_string : string -> message_id * values
  (** May raise [(Unknown_msg_name msg_name)] *)

  val string_of_message : ?sep:string -> message -> values -> string
  (** [string_of_message ?sep msg values] Default [sep] is space *)

  val message_send : ?timestamp:float -> ?link_id:int -> string -> string -> values -> unit
  (** [message_send sender link_id msg_name values] *)

  val message_bind : ?sender:string -> ?timestamp:bool -> string -> (string -> values -> unit) -> Ivy.binding
  (** [message_bind ?sender msg_name callback] *)

  val message_answerer : string -> string -> (string -> values -> values) -> Ivy.binding
  (** [message_answerer sender msg_name callback] *)

  val message_req : string -> string -> values -> (string -> values -> unit) -> unit
(** [message_answerer sender msg_name values receiver] Sends a request on the Ivy bus for the specified message. On reception, [receiver] will be applied on [sender_name] and expected values. *)
end



module MessagesOfXml(Class:CLASS_Xml) = struct
  let max_length = 256
  let messages_by_id, messages_by_name =
    try
      let select = fun x -> Xml.attrib x "name" = Class.name in
      let xml_class = try ExtXml.child Class.xml ~select "msg_class" with Not_found -> ExtXml.child Class.xml ~select "class" in
      parse_class xml_class
    with
        Not_found -> failwith (sprintf "Unknown message class: %s" Class.name)
  let messages = messages_by_id
  let message_of_id = fun id -> try Hashtbl.find messages_by_id id with Not_found -> fprintf stderr "message_of_id :%d\n%!" id; raise Not_found
  let message_of_name = fun name ->
    try
      Hashtbl.find messages_by_name name
    with
        Not_found -> raise (Unknown_msg_name (name, Class.name))


  let values_of_payload = fun buffer ->
    let buffer = Serial.string_of_payload buffer in
    try
      let id = Char.code buffer.[offset_msg_id] in
      let ac_id = Char.code buffer.[offset_ac_id] in
      let message = message_of_id id in
      Debug.call 'T' (fun f -> fprintf f "Pprz.values id=%d\n" id);
      let rec loop = fun index fields ->
        match fields with
            [] ->
              if index = String.length buffer then
                []
              else
                failwith (sprintf "Pprz.values_of_payload, too many bytes in message %s: %s" message.name (Debug.xprint buffer))
          | (field_name, field_descr)::fs ->
            let (value, n) = value_field buffer index field_descr in
            (field_name, value) :: loop (index+n) fs in
      (id, ac_id, loop offset_fields message.fields)
    with
        Invalid_argument("index out of bounds") ->
          failwith (sprintf "Pprz.values_of_payload, wrong argument: %s" (Debug.xprint buffer))


  let payload_of_values = fun id ac_id values ->
    let message = message_of_id id in

    (** The actual length is computed from the values *)
    let p = String.make max_length '#' in

    p.[offset_msg_id] <- Char.chr id;
    p.[offset_ac_id] <- Char.chr ac_id;
    let i = ref offset_fields in
    List.iter
      (fun (field_name, field) ->
        let v =
          try List.assoc field_name values with
              Not_found -> default_value field._type in
        let size = sprint_value p !i field._type v in
        i := !i + size
      )
      message.fields;

    (** Cut to the actual length *)
    let p = String.sub p 0 !i in
    Serial.payload_of_string p


  let space = Str.regexp "[ \t]+"
  let array_sep = Str.regexp "[\"|]" (* also search for old separator '|' for backward compatibility *)
  let values_of_string = fun s ->
    (* split arguments and arrays *)
    let array_split = Str.full_split array_sep s in
    let rec loop = fun fields ->
      match fields with
      | [] -> []
      | (Str.Delim "\"")::((Str.Text l)::[Str.Delim "\""]) | (Str.Delim "|")::((Str.Text l)::[Str.Delim "|"]) -> [l]
      | (Str.Delim "\"")::((Str.Text l)::((Str.Delim "\"")::xs)) | (Str.Delim "|")::((Str.Text l)::((Str.Delim "|")::xs)) -> [l] @ (loop xs)
      | [Str.Text x] -> Str.split space x
      | (Str.Text x)::xs -> (Str.split space x) @ (loop xs)
      | (Str.Delim _)::_ -> failwith "Pprz.values_of_string: incorrect array delimiter"
    in
    let msg_split = loop array_split in
    match msg_split with
        msg_name::args ->
          begin
            try
              let msg_id, msg = message_of_name msg_name in
              let values = List.map2 (fun (field_name, field) v -> (field_name, value field._type v)) msg.fields args in
              (msg_id, values)
            with
                Invalid_argument "List.map2" -> failwith (sprintf "Pprz.values_of_string: incorrect number of fields in '%s'" s)
          end
      | [] -> invalid_arg (sprintf "Pprz.values_of_string: %s" s)

  let string_of_message = fun ?(sep=" ") msg values ->
    (** Check that the values are compatible with this message *)
    List.iter
      (fun (k, _) ->
        if not (List.mem_assoc k msg.fields)
        then invalid_arg (sprintf "Pprz.string_of_message: unknown field '%s' in message '%s'" k msg.name))
      values;

    String.concat sep
      (msg.name::
         List.map
     (fun (field_name, field) ->
       let v =
         try List.assoc field_name values with
             Not_found ->
               default_value field._type in
       formatted_string_of_value field.fformat v)
     msg.fields)

  let message_send = fun ?timestamp ?link_id sender msg_name values ->
    let m = snd (message_of_name msg_name) in
    let s = string_of_message m values in
    let timestamp_string =
      match timestamp with
          None -> ""
        | Some x -> sprintf "%f " x in
    let msg = sprintf "%s%s %s" timestamp_string sender s in
    let n = String.length msg in
    if n > 1000 then (** FIXME: to prevent Ivy bug on long message *)
      fprintf stderr "Discarding long ivy message (%d bytes)\n%!" n
    else
      match link_id with
        None -> Ivy.send msg
      | Some the_link_id -> begin
        let index = ref 0 in
        let modified_msg = String.copy msg in
        let func = fun c ->
          match c with
            ' ' -> begin
            String.set modified_msg !index ';';
            index := !index + 1
            end
          | x -> index := !index + 1; in
        String.iter func modified_msg;
        Ivy.send ( Printf.sprintf "redlink TELEMETRY_MESSAGE %s %i %s" sender the_link_id modified_msg);
      end

  let message_bind = fun ?sender ?(timestamp=false) msg_name cb ->
    let tsregexp, tsoffset = if timestamp then "([0-9]+\\.[0-9]+ )?", 1 else "", 0 in
    match sender with
        None ->
          Ivy.bind
            (fun _ args ->
              let values = try snd (values_of_string args.(1+tsoffset)) with exc -> prerr_endline (Printexc.to_string exc); [] in
              cb args.(tsoffset) values)
            (sprintf "^%s([^ ]*) +(%s( .*|$))" tsregexp msg_name)
      | Some s ->
        Ivy.bind
          (fun _ args ->
            let values = try snd (values_of_string args.(tsoffset)) with  exc -> prerr_endline (Printexc.to_string exc); [] in
            cb s values)
          (sprintf "^%s%s +(%s( .*|$))" tsregexp s msg_name)

  let message_answerer = fun sender msg_name cb ->
    let ivy_cb = fun _ args ->
      let asker = args.(0)
      and asker_id = args.(1) in
      try (** Against [cb] exceptions *)
        let values = cb asker (snd (values_of_string args.(2))) in
        let m = string_of_message (snd (message_of_name msg_name)) values in
        Ivy.send (sprintf "%s %s %s" asker_id sender m)
      with
          exc -> fprintf stderr "Pprz.answerer %s:%s: %s\n%!" sender msg_name (Printexc.to_string exc)
    in
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

module Messages(Class:CLASS) = struct
  include MessagesOfXml(struct
    let xml = messages_xml ()
    let name = Class.name
  end)
end
